ROS 2 + PX4  3D A* 全局规划与避障系统

基于 ROS 2 和 PX4 的无人机三维全局路径规划与避障系统。本项目在 Gazebo 仿真环境中使用自定义的 3D A* 算法，实现了无人机的自主起飞、简单环境避障飞行，并集成了 RViz 2 进行实时的 3D 路径可视化。
一、核心特性 (Key Features)

     A* 全局规划器*：在 3D 空间中动态计算最优路径，支持全向对角线飞行。

     障碍物膨胀 ：算法内置代价地图逻辑，自动为物理墙体生成安全缓冲区，消除无人机由于高速惯性导致的“甩尾撞墙”风险。

    PX4 Offboard控制：通过发送无速度/加速度约束的位置设定点，利用 PX4 底层轨迹生成器飞出符合动力学的平滑曲线。

     RViz 2 实时可视化：精准处理 NED（北东地）与 ENU（东北天）坐标系的转换，在起飞前即可于 RViz 2 中渲染出的绿色 3D 规划航线。

    Gazebo 环境：内置精确标定的墙壁障碍物模型，实现“物理仿真”与“数学网格”的 1:1 坐标对齐。

二、环境依赖 (Prerequisites)

本项目在以下环境中测试通过：

    OS: Ubuntu 24.04

    ROS 2: Jazzy

    PX4 Autopilot: v1.14 / main branch

    Middleware: micro_ros_agent

    Simulation: Gazebo Classic / Gazebo Harmonic (视 PX4 配置而定)

三、核心文件结构 (File Structure)
Plaintext

px4_offboard/
├── CMakeLists.txt              # 现代 CMake 配置，链接 Eigen3 及 ROS 2 消息包
├── package.xml                 # 包含 px4_msgs, nav_msgs, geometry_msgs 等依赖
├── src/
│   └── astar.cpp               # A* 算法与 ROS 2 Offboard 控制节点核心代码
└── worlds/
    └── custom_maze.sdf         # (可选) 包含两堵墙及缝隙的 Gazebo 物理世界文件

四、编译与安装 (Build & Install)

    将本包克隆或放置到你的 ROS 2 工作空间（如 ~/ros2_ws/src）下。

    编译项目：

Bash

cd ~/ros2_ws
colcon build --packages-select px4_offboard
source install/setup.bash

五、运行指南 (Usage)

请严格按照以下顺序启动各个终端：

Terminal 1: 启动 PX4 SITL 与 Gazebo 仿真
Bash

cd ~/PX4-Autopilot
make px4_sitl gz_x500 

Terminal 2: 启动 micro_ros_agent (通信桥梁)
Bash

source ~/micro_ros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -port 8888 -v 4

Terminal 3: 启动 RViz 2 可视化
Bash

rviz2

(提示：在 RViz 2 中将 Fixed Frame 设置为 map，并通过 Add -> By display type -> nav_msgs/Path 添加 /planned_path 话题)

Terminal 4: 运行自主导航节点
Bash

cd ~/ros2_ws
source install/setup.bash
ros2 run px4_offboard astar

六、运行效果预演

节点启动后，你将观察到以下现象：

    获取坐标：无人机接入里程计，获取真实世界起点坐标。

    空间膨胀与规划：终端打印 A* 规划成功！。

    路径可视化：RViz 2 中瞬间出现一条绿色的 3D 避障折线。

    自主飞行：Gazebo 中的无人机自动切入 Offboard 模式，解锁起飞，绕过墙壁并悬停在终点。

七、注意事项 (Notes)
1.本示例中的 Gazebo 物理世界文件 custom_maze.sdf 仅包含两堵墙及缝隙，请根据实际需求进行修改。具体修改方式如下：
  在终端中运行code ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf进入编辑模式，将所需障碍物添加到文件中。

2.本示例中的 A* 规划器仅支持全向对角线飞行，请根据实际需求进行修改。

3.本示例中的 A* 规划器仅支持 3D 空间，请根据实际需求进行修改。

4.本示例中的 A* 规划器仅支持 3D 障碍物，请根据实际需求进行修改。

5.本示例中的障碍物硬编码在A* 规划器中，并未接入感知模块，请根据实际需求进行修改！