#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <array>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

struct MapNode {
    int x,y,z;//节点坐标
    double g,h,f;//代价
    MapNode* parent;//指向父节点
    MapNode(int x, int y, int z)
        : x(x), y(y), z(z), g(0), h(0), f(0),parent(nullptr) {
    }
};
//定义比较规则
struct CompareNodes {
    bool operator()(const MapNode* a, const MapNode* b) {
        return a->f > b->f;
    }
};
//定义三维网格地图
using GridMap3D = std::vector<std::vector<std::vector<int>>>;


const int MAX_X = 20;
const int MAX_Y = 20;
const int MAX_Z = 20;
std::vector<MapNode*> astar(const GridMap3D& map,MapNode* start, MapNode* goal){
    std::priority_queue<MapNode*,std::vector<MapNode*>,CompareNodes> open_list;
    std::vector<std::vector<std::vector<bool>>> closed_list(MAX_X,std::vector<std::vector<bool>>(MAX_Y,std::vector<bool>(MAX_Z,false)));
    open_list.push(start);
    while(!open_list.empty()){
        MapNode* current = open_list.top();
        open_list.pop();
        if (closed_list[current->x][current->y][current->z]) {
            continue; 
        }
        if(current->x == goal->x && current->y == goal->y && current->z == goal->z){
            std::vector<MapNode*> path;
            MapNode* trace = current;
            while(trace!=nullptr){
                path.push_back(trace);
                trace = trace->parent;
            }
            std::reverse(path.begin(),path.end());
            return path;
        }
        closed_list[current->x][current->y][current->z] = true;
        for(int dx=-1;dx<=1;dx++){
            for(int dy=-1;dy<=1;dy++){
                for(int dz=-1;dz<=1;dz++){
                    if(dx==0 && dy==0 && dz==0) continue;// 如果 dx, dy, dz 都是 0，说明是当前节点自己，直接跳过
                    int nx = current->x + dx;// 计算邻居的三维绝对坐标
                    int ny = current->y + dy;
                    int nz = current->z + dz;
                    if(nx<0 || nx>=MAX_X || ny<0 || ny>=MAX_Y || nz<0 || nz>=MAX_Z) continue;//1. 检查是否越界（飞出地图边缘）
                    if(map[nx][ny][nz] == 1) continue;// 2. 检查是否撞墙 (假设 map 中 1 代表障碍物，0 代表空地)
                    if(closed_list[nx][ny][nz]) continue;// 3. 检查是否走回头路 (已经在 closed_list 中)
                    double g_cost = current->g + std::sqrt(dx*dx + dy*dy + dz*dz);// 实际走过的距离代价 g。直线是 1，平面对角线是 √2 (1.414)，空间对角线是 √3 (1.732)
                    double h_cost = std::sqrt((nx-goal->x)*(nx-goal->x)+(ny-goal->y)*(ny-goal->y)+(nz-goal->z)*(nz-goal->z));// 预估到终点的距离代价 h (这里使用三维欧几里得距离)
                    MapNode* neighbor = new MapNode(nx,ny,nz);// 实例化这个合法的邻居节点
                    neighbor->g = g_cost;
                    neighbor->h = h_cost;
                    neighbor->f = g_cost + h_cost;
                    neighbor->parent = current;// 设置父节点指针
                    open_list.push(neighbor);
                }
            }
        }
}
    return std::vector<MapNode*>(); // 如果 open_list 为空了还没找到路径，说明无解，返回空路径
}







class AstarNode : public rclcpp::Node {
public:
    AstarNode() : Node("Astar_node"), offboard_setpoint_counter_(0), waypoint_index_(0) {
        
        // QoS 设置为 Best Effort，适配 PX4 DDS 默认配置
        auto qos = rclcpp::SensorDataQoS();

        // 发布者
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        // 初始化路径可视化发布者
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        // 订阅者（获取当前位置）
        local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos, 
            std::bind(&AstarNode::position_callback, this, std::placeholders::_1));


        
       

        // 创建10Hz的定时器，Offboard模式要求发送频率不低于2Hz
        timer_ = this->create_wall_timer(100ms, std::bind(&AstarNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "飞行节点已启动，准备起飞...");
    }

private:
    void timer_callback() {
        if(!path_planned_) {
                RCLCPP_WARN(this->get_logger(), "尚未规划出路径，无法切换到 Offboard 模式！");
                return;
            }
        if (offboard_setpoint_counter_ == 10) {
            
            // 在切换到 Offboard 模式之前，必须已经发布了一段时间的 setpoint
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // 1: custom mode, 6: offboard
            this->arm();
        }

        // 发送 Offboard 心跳信号
        publish_offboard_control_mode();
        // 发送当前航点期望位置
        publish_trajectory_setpoint();

        //每次定时器触发，都更新一下时间戳并把路径发给 RViz！
        planned_path_msg_.header.stamp = this->get_clock()->now();
        path_publisher_->publish(planned_path_msg_);

        if (offboard_setpoint_counter_ < 11) {
            offboard_setpoint_counter_++;
        }
    }

    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "收到 local position: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
        if(!has_home_position_) {
            home_pos_ = {msg->x, msg->y, msg->z};
            has_home_position_ = true;
            RCLCPP_INFO(this->get_logger(), "已获取起点位置: [%.2f, %.2f, %.2f]", home_pos_[0], home_pos_[1], home_pos_[2]);
    
            plan_path();
            return;
        }
        if(!path_planned_) {
            RCLCPP_INFO(this->get_logger(), "正在规划路径...");
            plan_path();
            return;
        }




        if (waypoint_index_ >= waypoints_.size()) return; // 已完成所有航点

        // 获取当前航点
        auto target = waypoints_[waypoint_index_];
        
        // 计算当前位置与目标航点的三维距离
        float dx = msg->x - target[0];
        float dy = msg->y - target[1];
        float dz = msg->z - target[2];
        float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

        // 如果距离小于0.5米，认为到达航点，切换到下一个
        if (distance < 0.5f) {
            waypoint_index_++;
            if (waypoint_index_ < waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "到达航点! 前往下一个航点: [%.1f, %.1f, %.1f]", 
                            waypoints_[waypoint_index_][0], waypoints_[waypoint_index_][1], waypoints_[waypoint_index_][2]);
            } else {
                RCLCPP_INFO(this->get_logger(), "到达终点！开始悬停。");
            }
        }
    }

    void plan_path(){
        RCLCPP_INFO(this->get_logger(), "开始构建 3D 地图并运行 A* 算法...");
        // 1. 初始化 20x20x20 的地图并造墙 
        GridMap3D map(20, std::vector<std::vector<int>>(20, std::vector<int>(20, 0)));
      for (int y = 0; y <= 18; ++y) {
    if (y >= 6 && y <= 10) continue; // 真实的物理缺口
    for (int z = 0; z <= 12; ++z) {
        map[10][y][z] = 1; // 真实的墙只在 X=10
    }
}

// 第二步：算法层面的“自动膨胀”（Inflation）
// 遍历地图，凡是发现墙的地方，就把周围 2 米的格子全都标记为“高风险区”或“墙”
GridMap3D inflated_map = map; // 复制一份地图
int safe_radius = 2; // 安全半径 2 米
for (int x = 0; x < 20; x++) {
    for (int y = 0; y < 20; y++) {
        for (int z = 0; z < 20; z++) {
            if (map[x][y][z] == 1) { // 发现真墙
                // 向外膨胀
                for(int dx = -safe_radius; dx <= safe_radius; dx++) {
                    for(int dy = -safe_radius; dy <= safe_radius; dy++) {
                        int nx = x + dx, ny = y + dy;
                        if(nx >=0 && nx < 20 && ny >=0 && ny < 20) {
                            inflated_map[nx][ny][z] = 1; // 涂黑周围区域
                        }
                    }
                }
            }
        }
    }
}
        //获取起点的绝对网格坐标
        int start_x = std::max(0, std::min(MAX_X - 1, (int)std::round(home_pos_[0])));
        int start_y = std::max(0, std::min(MAX_Y - 1, (int)std::round(home_pos_[1])));
        int start_z = 2; // 强制起飞高度设定为 Z=2 米

        MapNode* start = new MapNode(start_x, start_y, start_z);
        MapNode* goal  = new MapNode(18, 18, 5); // 设定终点
        
        std::vector<MapNode*> astar_path = astar(inflated_map, start, goal);
        
        if (astar_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "A* 规划失败！找不到路径！");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "A* 规划成功！路径长度为 %zu 个格子。", astar_path.size());

        // 4. 将 A* 输出的格子路径转换回物理坐标，并存入 waypoints_
       
        waypoints_.clear();
        constexpr float kRes = 1.0f; // 1个格子 = 1米

        for (MapNode* node : astar_path) {
            // 网格索引直接等于物理米数
            float x = static_cast<float>(node->x) * kRes;
            float y = static_cast<float>(node->y) * kRes;
            
            // Z 轴依然是以 home_pos_[2] (真实地面) 为基准，向上为负
            float z = home_pos_[2] - static_cast<float>(node->z) * kRes;

            waypoints_.push_back({x, y, z});
        }
        //打印所有航点
        for (size_t i = 0; i < waypoints_.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "waypoint[%zu]: [%.2f, %.2f, %.2f]",
                i, waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
        }

        // ==========================================
        // 生成供 RViz 2 渲染的 3D 路径 (存入全局变量)
        // ==========================================
        planned_path_msg_.header.frame_id = "map"; 
        planned_path_msg_.poses.clear(); // 清空旧数据

        for (const auto& wp : waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            
            // PX4(NED) 转 RViz(ENU)
            pose.pose.position.x = wp[1];  
            pose.pose.position.y = wp[0];  
            pose.pose.position.z = -wp[2]; 
            
            planned_path_msg_.poses.push_back(pose);
        }

        path_planned_ = true;
    }
    void arm() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "发送解锁指令 (Arming)");
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true; // 我们采用位置控制
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        
        int idx = (waypoint_index_ < waypoints_.size()) ? waypoint_index_ : waypoints_.size() - 1;
        
        // 1. 设置期望位置 (NED 坐标系)
        msg.position = {waypoints_[idx][0], waypoints_[idx][1], waypoints_[idx][2]};
        
        // 2. 致命错误修复：必须将不需要控制的量设置为 NaN！
        // 这样飞控就会自动生成平滑的速度和加速度曲线，而不是强行将其压制为 0
        msg.velocity = {std::nanf(""), std::nanf(""), std::nanf("")};
        msg.acceleration = {std::nanf(""), std::nanf(""), std::nanf("")};
        msg.jerk = {std::nanf(""), std::nanf(""), std::nanf("")};
        
        // 同样，如果不指定偏航角速度，也设为 NaN
        msg.yaw = std::nanf("");  
        msg.yawspeed = std::nanf("");

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    uint64_t offboard_setpoint_counter_;
    size_t waypoint_index_;
    std::vector<std::array<float, 3>> waypoints_;

    bool has_home_position_ = false;//是否获取到了无人机起点
    bool path_planned_ = false;//是否已经规划好了路径
    std::array<float, 3> home_pos_;//无人机起点坐标

    // 用来存储生成的路径，方便持续发布
    nav_msgs::msg::Path planned_path_msg_;

    // 新增：网格到真实坐标的映射
    static constexpr float kRes = 1.0f;          // 1个格子 = 1米
std::array<float, 3> grid_origin_world_{};    // 网格(0,0,0)对应的世界坐标
};

int main(int argc, char *argv[]) {
    std::cout << "Starting Square Flight Node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AstarNode>());
    rclcpp::shutdown();
    return 0;
}