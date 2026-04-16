#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <Eigen/Geometry>


using namespace std::chrono_literals;

class TakeoffNode : public rclcpp::Node {
public:
    TakeoffNode() : Node("takeoff_node"), offboard_counter_(0) {
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        vehicle_attitude_sub_=this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(), [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg){vehicle_attitude_callback(msg);});
        timer_ = this->create_wall_timer(100ms, [this]() { timer_callback(); });
        RCLCPP_INFO(this->get_logger(), "Takeoff node started");
    }

private:
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        Eigen::Quaternionf q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        auto euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
        current_yaw_ = euler(0); // 更新当前偏航角
    }
    void timer_callback() {
        // 1. 持续发送 Offboard 控制模式心跳
        px4_msgs::msg::OffboardControlMode offboard_msg{};
        offboard_msg.position = false;
        offboard_msg.velocity = true;
        offboard_msg.acceleration = false;
        offboard_msg.attitude = false;
        offboard_msg.body_rate = false;
        offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(offboard_msg);

        // 2. 发布期望位置 (NED坐标系，z 负向上)
        float forward = 2.0f; // 前向速度
        float right = 0.0f;   // 右向速度
        float vel_north, vel_east;
        body2ned(forward, right, vel_north, vel_east);
        px4_msgs::msg::TrajectorySetpoint setpoint{};
        setpoint.position = {NAN, NAN, NAN};
        setpoint.velocity = {vel_north, vel_east, -1.0};
        setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(setpoint);

        // 3. 发送解锁和模式切换指令（按计数执行）
        if (offboard_counter_ == 10) {
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            RCLCPP_INFO(this->get_logger(), "Arm command sent");
        } else if (offboard_counter_ == 20) {
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); // mode: Offboard
            RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
        }

        offboard_counter_++;
    }

    void publish_vehicle_command(uint16_t cmd, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.command = cmd;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;// 目标系统ID，通常无人机本体为 1
        msg.target_component = 1;// 目标组件ID，通常飞控核心为 1
        msg.source_system = 1;// 来源系统ID，机载电脑通常自定义为 1 或其他
        msg.source_component = 1; // 来源组件ID
        msg.from_external = true;// 关键标志：标记此命令来自外部设备（如机载电脑）
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    void body2ned(float body_x, float body_y,float &ned_x, float &ned_y) {
        Eigen::Matrix2f R;
	        R << std::cos(current_yaw_), -std::sin(current_yaw_),
	             std::sin(current_yaw_),  std::cos(current_yaw_);
	        
	        Eigen::Vector2f vel_body_2d(body_x, body_y);
	        Eigen::Vector2f vel_ned_2d = R * vel_body_2d;
	 
	        ned_x = vel_ned_2d.x();
	        ned_y = vel_ned_2d.y();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    int offboard_counter_;
    float current_yaw_ = 0.0f; // 假设无人机初始朝向为北（yaw=0），实际应用中可通过订阅姿态信息更新此值
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TakeoffNode>());
    rclcpp::shutdown();
    return 0;
}