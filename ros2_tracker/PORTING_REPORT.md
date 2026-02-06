# Elastic-Tracker ROS1 → ROS2 移植报告

## 1. 项目概述

本项目将 Elastic-Tracker（基于 ROS1 Noetic 的双机器人跟踪系统）完整移植到 ROS2 Humble，采用 FSM（有限状态机）+ 组合模式替代原有的 Nodelet 架构。移植后的 `ros2_tracker` 工作空间完全自包含，包含 18 个 ROS2 包，可独立编译运行。

### 1.1 原始系统功能

- **跟踪者（drone0）**：搭载深度相机的无人机，通过 EKF 估计目标位置，在三维障碍物环境中规划跟踪轨迹
- **目标（target）**：地面小车，使用 A* 导航到用户指定的目标点
- 规划管线：A* 前端搜索 → 多面体走廊生成（EllipsoidDecomp3D）→ MINCO 轨迹优化

---

## 2. 架构对比

### 2.1 ROS1 架构（Nodelet）

```
planning_nodelet (Nodelet)
├── 内联 OccGridMap 建图
├── 内联 Env A* 搜索
├── 内联 Prediction 目标预测
├── 内联 TrajOpt 轨迹优化
└── 内联 Visualization 可视化

mapping_nodelet (Nodelet)     — 独立 depth→gridmap
target_ekf_sim_node (Node)    — 独立 EKF
traj_server (Node)            — 轨迹执行
```

- 通过 `nodelet::Nodelet` 实现零拷贝数据共享
- `planning_nodelet` 既处理 tracking 模式也处理 navigation（fake）模式
- 各 Nodelet 通过 `nodelet_manager` 运行在同一进程

### 2.2 ROS2 架构（FSM + 组合）

```
tracker_node (rclcpp::Node)
└── TrackingFSM
    ├── OccGridMapper      — 3D 栅格建图（depth / global_map 双模式）
    ├── TargetEkf          — 目标 EKF 估计（sim / real 双模式）
    ├── env::Env           — A* 搜索 + 走廊生成 + 可见性路径
    ├── prediction::Predict — 目标运动预测
    ├── traj_opt::TrajOpt  — MINCO 轨迹优化
    └── visualization::Visualization — RViz2 可视化

traj_server (rclcpp::Node)   — 独立轨迹执行节点
```

- 所有子模块通过组合持有 `rclcpp::Node::SharedPtr`，不继承 Node
- `TrackingFSM` 统一管理状态转换（INIT → WAIT_TARGET → TRACKING ↔ REPLAN）
- 同一可执行文件支持 tracking 和 navigation 两种模式

### 2.3 架构决策理由

| 决策 | 理由 |
|------|------|
| FSM 替代 Nodelet | ROS2 无 Nodelet，且 FSM 模式更清晰地表达状态转换逻辑 |
| 组合替代继承 | 子模块通过 `init(node)` 接收 Node 指针，避免多节点开销 |
| 单可执行文件双模式 | tracking 和 navigation 共享 mapper/planner 代码，通过参数切换 |
| Header-only Visualization | 原始 ROS1 代码即为 header-only，模板函数必须在头文件中定义 |

---

## 3. 包清单

### 3.1 完整包列表（18 个）

| 包名 | 来源 | 说明 |
|------|------|------|
| **tracker_fsm** | 新建（从 ROS1 移植） | 核心 FSM 包：tracker_node + traj_server |
| **traj_opt** | 从 ROS1 移植 | MINCO 轨迹优化库 |
| **object_detection_msgs** | 从 ROS1 移植 | 目标检测消息（5 个 .msg） |
| **decomp_ros_msgs** | 从 ROS1 移植 | 多面体分解消息（4 个 .msg） |
| **decomp_ros_utils** | 从 ROS1 移植 | RViz2 插件 + 分解数据转换 |
| **quadrotor_msgs** | 从 ros2_ego 复制 + 扩展 | 新增 OccMap3d / PolyTraj / ReplanState |
| **so3_quadrotor_simulator** | 从 ros2_ego 复制 | 四旋翼动力学仿真器 |
| **so3_control** | 从 ros2_ego 复制 | SO3 姿态控制器 |
| **mockamap** | 从 ros2_ego 复制 | 随机地图生成器 |
| **local_sensing** | 从 ros2_ego 复制 | 深度图渲染（pcl_render_node） |
| **odom_visualization** | 从 ros2_ego 复制 + 扩展 | 新增 odom_visualization_car 可执行文件 |
| **pose_utils** | 从 ros2_ego 复制 | 位姿工具库 |
| **uav_utils** | 从 ros2_ego 复制 | 无人机工具库 |
| **cmake_utils** | 从 ros2_ego 复制 | CMake 工具 |
| **waypoint_generator** | 从 ros2_ego 复制 | 航点生成器 |
| **poscmd_2_odom** | 从 ros2_ego 复制 | PositionCommand 转 Odometry |
| **map_generator** | 从 ros2_ego 复制 | 地图生成工具 |
| **multi_map_server** | 从 ros2_ego 复制 | 多地图服务 |

### 3.2 自定义消息汇总

**quadrotor_msgs（扩展）：**
- `OccMap3d.msg` — 3D 占据栅格地图
- `PolyTraj.msg` — 多项式轨迹（`time` → `builtin_interfaces/Time`）
- `ReplanState.msg` — 重规划状态

**object_detection_msgs（新建）：**
- `BoundingBox.msg`（`Class` 字段重命名为 `class_name` 避免 C++ 关键字冲突）
- `BoundingBoxes.msg`, `CarPosition.msg`, `MotorAngle.msg`, `ObjectCount.msg`

**decomp_ros_msgs（新建）：**
- `Ellipsoid.msg`（`E` 字段重命名为 `e` 遵循 ROS2 命名规范）
- `EllipsoidArray.msg`, `Polyhedron.msg`, `PolyhedronArray.msg`

---

## 4. 文件变更对照表

### 4.1 核心 Tracker 模块

| ROS1 源文件 | ROS2 对应文件 | 变更类型 |
|-------------|---------------|----------|
| `planning/planning/src/planning_nodelet.cpp` | `tracker/tracker_fsm/src/tracking_fsm.cpp` | 重写（Nodelet → FSM） |
| `planning/planning/src/traj_server.cpp` | `tracker/tracker_fsm/src/traj_server.cpp` | 移植 |
| `planning/planning/include/env/env.hpp` | `tracker/tracker_fsm/include/tracker_fsm/env.hpp` | 移植（保留 A* 搜索逻辑） |
| `planning/planning/include/prediction/prediction.hpp` | `tracker/tracker_fsm/include/tracker_fsm/prediction.hpp` | 移植 |
| `planning/planning/include/visualization/visualization.hpp` | `tracker/tracker_fsm/include/tracker_fsm/tracking_visualization.h` | 移植（header-only） |
| `mapping/src/mapping.cc` | `tracker/tracker_fsm/src/mapping.cc` | 移植（核心逻辑不变） |
| `mapping/include/mapping/mapping.h` | `tracker/tracker_fsm/include/tracker_fsm/mapping.h` | 移植 |
| `mapping/src/mapping_nodelet.cpp` | `tracker/tracker_fsm/src/occ_grid_mapper.cpp` | 重写（Nodelet → 组合模块） |
| `detection/target_ekf/src/target_ekf_sim_node.cpp` | `tracker/tracker_fsm/src/target_ekf.cpp` | 重写（独立节点 → 组合模块） |
| — | `tracker/tracker_fsm/src/tracker_node.cpp` | 新建（ROS2 入口） |

### 4.2 轨迹优化库

| ROS1 源文件 | ROS2 对应文件 | 变更类型 |
|-------------|---------------|----------|
| `planning/traj_opt/src/traj_opt.cc` | `tracker/traj_opt/src/traj_opt.cc` | 移植（参数 API 替换） |
| `planning/traj_opt/src/traj_opt_fake.cc` | `tracker/traj_opt/src/traj_opt_fake.cc` | 移植 |
| `planning/traj_opt/include/traj_opt/*.hpp` | `tracker/traj_opt/include/traj_opt/*.hpp` | 复制（纯数学，无 ROS 依赖） |

### 4.3 RViz2 插件

| ROS1 源文件 | ROS2 对应文件 | 变更类型 |
|-------------|---------------|----------|
| `DecompROS/decomp_ros_utils/src/*_display.cpp` | `DecompROS/decomp_ros_utils/src/*_display.cpp` | 移植（rviz → rviz_common） |
| `DecompROS/decomp_ros_utils/src/*_visual.cpp` | `DecompROS/decomp_ros_utils/src/*_visual.cpp` | 重写（MeshShape → ManualObject） |
| `DecompROS/decomp_ros_utils/include/*/data_ros_utils.h` | `DecompROS/decomp_ros_utils/include/*/data_ros_utils.h` | 移植 |

### 4.4 仿真器扩展

| 变更 | 文件 | 说明 |
|------|------|------|
| 新增 | `odom_visualization/src/odom_visualization_car.cpp` | 小车可视化（Z=0, 2D 旋转） |
| 复制 | `odom_visualization/meshes/f250.dae`, `car.dae` | 从 ROS1 复制的 mesh 文件 |

---

## 5. ROS1 → ROS2 API 变更记录

### 5.1 核心 API 转换

| ROS1 API | ROS2 API |
|----------|----------|
| `ros::NodeHandle nh` | `rclcpp::Node::SharedPtr node_` |
| `ros::Publisher pub` | `rclcpp::Publisher<MsgType>::SharedPtr pub_` |
| `ros::Subscriber sub` | `rclcpp::Subscription<MsgType>::SharedPtr sub_` |
| `ros::Timer timer` | `rclcpp::TimerBase::SharedPtr timer_` |
| `nh.param("name", var, default)` | `node_->declare_parameter("name", default)` + `node_->get_parameter("name", var)` |
| `ros::Time::now()` | `node_->now()` |
| `ROS_INFO(...)` | `RCLCPP_INFO(node_->get_logger(), ...)` |
| `ROS_WARN(...)` | `RCLCPP_WARN(node_->get_logger(), ...)` |
| `ROS_ERROR(...)` | `RCLCPP_ERROR(node_->get_logger(), ...)` |
| `nodelet::Nodelet` | 普通类 + `init(rclcpp::Node::SharedPtr)` |

### 5.2 消息和 TF

| ROS1 | ROS2 |
|------|------|
| `nav_msgs::Odometry` | `nav_msgs::msg::Odometry` |
| `sensor_msgs::Image` | `sensor_msgs::msg::Image` |
| `geometry_msgs::PoseStamped` | `geometry_msgs::msg::PoseStamped` |
| `time` 字段（msg 定义） | `builtin_interfaces/Time` |
| `tf::TransformBroadcaster` | `tf2_ros::TransformBroadcaster` |
| `tf::StampedTransform` | `geometry_msgs::msg::TransformStamped` |

### 5.3 Message Filters

| ROS1 | ROS2 |
|------|------|
| `message_filters::Subscriber<M>(nh, topic, queue)` | `message_filters::Subscriber<M>(node_, topic, rmw_qos)` |
| `Synchronizer<ApproximateTime<...>>` | 相同 API，传入 `node_->get_clock()` |

### 5.4 构建系统

| ROS1 (catkin) | ROS2 (ament) |
|---------------|--------------|
| `catkin_package()` | `ament_package()` |
| `find_package(catkin REQUIRED ...)` | `find_package(ament_cmake REQUIRED)` |
| `catkin_INCLUDE_DIRS` | `ament_target_dependencies()` |
| `add_message_files()` | `rosidl_generate_interfaces()` |

### 5.5 RViz 插件 API

| ROS1 (rviz) | ROS2 (rviz_common / rviz_rendering) |
|-------------|-------------------------------------|
| `rviz::MessageFilterDisplay<M>` | `rviz_common::MessageFilterDisplay<M>` |
| `rviz::Shape` | `rviz_rendering::Shape` |
| `rviz::MeshShape` | `Ogre::ManualObject`（ROS2 Humble 无 MeshShape） |
| `rviz::BillboardLine` | `rviz_rendering::BillboardLine` |
| `rviz::FloatProperty` | `rviz_common::properties::FloatProperty` |
| `CLASS_LOADER_REGISTER_CLASS` | `PLUGINLIB_EXPORT_CLASS` |

---

## 6. 构建说明

### 6.1 系统依赖

- Ubuntu 22.04 + ROS2 Humble
- 编译器：GCC 11+ (C++17)
- 系统库：Eigen3, PCL, OpenCV, OGRE

### 6.2 ROS2 依赖

```bash
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-pcl-conversions \
  ros-humble-message-filters \
  ros-humble-tf2-ros \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-rendering \
  ros-humble-pluginlib \
  libpcl-dev \
  libeigen3-dev
```

### 6.3 编译命令

```bash
cd /path/to/ros2_tracker

# 全量编译
colcon build

# 单包编译（调试用）
colcon build --packages-select tracker_fsm

# 编译后加载环境
source install/setup.bash
```

### 6.4 编译顺序

编译器自动解析依赖，推荐顺序：

```
第 1 批（并行）：quadrotor_msgs, object_detection_msgs, decomp_ros_msgs
第 2 批（并行）：traj_opt, decomp_ros_utils, pose_utils, cmake_utils
第 3 批（依赖 1-2 批）：tracker_fsm
第 4 批（独立）：uav_simulator 各包
```

---

## 7. 运行说明

### 7.1 启动仿真

```bash
# 终端 1：启动完整仿真（tracker + target + 仿真器 + 地图）
source install/setup.bash
ros2 launch tracker_fsm tracking_sim.launch.py

# 终端 2：启动 RViz2（可选）
source install/setup.bash
ros2 launch tracker_fsm rviz.launch.py
```

### 7.2 仿真操作流程

1. 启动仿真后，等待所有节点初始化完成
2. 在 RViz2 中使用 **2D Nav Goal** 工具给 target 发送导航目标（topic: `/move_base_simple/goal`）
3. Target 收到目标后开始 A* 导航
4. 使用 **Publish Point** 工具发送跟踪触发信号（topic: `/triger`）
5. Tracker 开始跟踪 target

### 7.3 Launch 文件启动的节点

| 命名空间 | 节点 | 功能 |
|----------|------|------|
| — | `mockamap_node` | 全局地图生成 |
| drone0 | `tracker_node` | 跟踪 FSM（mode=tracker） |
| drone0 | `traj_server` | 轨迹执行 |
| drone0 | `so3_quadrotor_simulator` | 四旋翼仿真 |
| drone0 | `so3_control_node` | SO3 控制器 |
| drone0 | `pcl_render_node` | 深度图渲染 |
| drone0 | `odom_visualization` | 无人机可视化（f250.dae） |
| target | `tracker_node` | 导航 FSM（mode=target） |
| target | `traj_server` | 轨迹执行 |
| target | `so3_quadrotor_simulator` | 四旋翼仿真 |
| target | `so3_control_node` | SO3 控制器 |
| target | `odom_visualization_car` | 小车可视化（car.dae） |

### 7.4 关键参数

配置文件位于 `tracker_fsm/config/`：

- `tracker_sim.yaml`：跟踪者参数（规划频率、跟踪距离、轨迹优化权重、相机内参、建图分辨率等）
- `nav_sim.yaml`：目标机器人参数（导航规划参数、全局地图建图参数）

---

## 8. 移植中遇到的问题及解决方案

### 8.1 C++17 兼容性：`filterLess` 比较运算符

**问题**：`geoutils.hpp` 中 `struct filterLess::operator()` 缺少 `const` 限定符，C++17 标准下编译失败。

**解决**：添加 `const` 限定符。

### 8.2 静态库链接共享库（-fPIC）

**问题**：`traj_opt` 编译为静态库（`.a`），但 `tracker_fsm_lib` 是共享库（`.so`），链接时报错缺少 Position-Independent Code。

**解决**：将 `traj_opt` 改为 `SHARED` 库。

### 8.3 RViz2 MeshShape 不可用

**问题**：ROS2 Humble 的 `rviz_rendering` 中没有 `MeshShape` 类（ROS1 rviz 有），导致 `decomp_ros_utils` 的 `mesh_visual` 编译失败。

**解决**：使用 `Ogre::ManualObject` 和 `Ogre::MaterialPtr` 重写 mesh 渲染逻辑。

### 8.4 processMessage 签名不匹配

**问题**：`rviz_common::MessageFilterDisplay` 的 `processMessage` 回调参数类型为 `ConstSharedPtr`（按值传递），而移植代码使用了引用传递。

**解决**：修改所有 Display 类的 `processMessage` 签名为 `ConstSharedPtr msg`（无 `&`），添加 `override` 关键字。

### 8.5 模板函数链接错误

**问题**：`tracking_visualization.cpp` 中的模板函数定义在 `.cpp` 文件中，显式实例化仅覆盖 `std::string` 类型，但实际调用使用字符串字面量（`const char[]`），导致 `undefined reference`。

**解决**：将所有模板函数定义移回头文件（header-only 模式），与原始 ROS1 的 `visualization.hpp` 一致。

### 8.6 declare_parameter 返回值类型

**问题**：`node->declare_parameter<T>()` 返回参数值而非 `bool`，在 `&&` 条件表达式中与布尔类型不兼容。

**解决**：分离 `declare_parameter()` 和 `get_parameter()` 为独立语句，实现 `declare_if_not` 辅助函数避免重复声明。

### 8.7 decomp_basis 头文件目录结构

**问题**：复制 `decomp_basis` 头文件时产生了错误的嵌套目录结构，导致 `#include <decomp_basis/data_type.h>` 找不到文件。

**解决**：删除错误结构，重新以正确的扁平结构复制头文件到 `include/decomp_basis/`。

---

## 9. 未移植的功能

| 功能 | 原始文件 | 原因 | 优先级 |
|------|----------|------|--------|
| wr_msg 调试回放 | `include/wr_msg/wr_msg.hpp` | 仅用于 debug bag 回放，使用 ROS1 序列化 API | 低 |
| position_cmd_to_twist | `scripts/position_cmd_to_twist.py` | 仅用于真实硬件（Twist 控制接口） | 中（硬件适配时） |
| CUDA 深度渲染 | `local_sensing/` | 用户确认使用 CPU 版本 | N/A |
| Real 模式 EKF | `target_ekf_node.cpp` | YOLO 检测输入版本，sim 优先 | 中（真实硬件时） |
| play_bag_node | `src/play_bag_node.cpp` | 调试工具 | 低 |
| test_node | `src/test_node.cpp` | 测试工具 | 低 |
| mapping_vis_node | `mapping/src/mapping_vis_node.cpp` | 独立建图可视化节点，可后续按需添加 | 低 |

---

## 10. 后续工作建议

### 10.1 仿真验证

- [ ] 在带障碍物环境中完成完整的 tracker-target 跟踪测试
- [ ] 验证 RViz2 中多面体走廊和轨迹的正确显示
- [ ] 调整仿真参数（规划频率、跟踪距离、优化权重）

### 10.2 功能完善

- [ ] 添加 `mapping_vis_node` 用于 RViz2 中显示 3D 栅格地图
- [ ] 创建 `sim.rviz` 配置文件预设 DecompROS 插件面板
- [ ] 添加 NavigationFSM 分离实现（当前 tracking 和 navigation 逻辑合并在 TrackingFSM 中）

### 10.3 真实硬件适配

- [ ] 移植 Real 模式 EKF（YOLO BoundingBox 输入）
- [ ] 移植 `position_cmd_to_twist.py`（Twist 控制接口）
- [ ] 适配真实深度相机驱动（替换 `pcl_render_node`）
- [ ] 添加硬件安全保护（紧急停止、高度限制等）

### 10.4 性能优化

- [ ] 考虑使用 `rclcpp::executors::MultiThreadedExecutor` 并行处理回调
- [ ] 评估 `intra_process_comms` 替代 Nodelet 零拷贝
- [ ] 建图模块的定时器频率调优

---

## 11. 目录结构总览

```
ros2_tracker/
├── PORTING_REPORT.md                          # 本报告
├── src/
│   ├── tracker/                               # ====== 跟踪核心 ======
│   │   ├── tracker_fsm/                       #   主包
│   │   │   ├── include/tracker_fsm/
│   │   │   │   ├── tracking_fsm.h             #   FSM 主类
│   │   │   │   ├── mapping.h                  #   RingBuffer 栅格地图
│   │   │   │   ├── occ_grid_mapper.h          #   建图组合模块
│   │   │   │   ├── target_ekf.h               #   EKF 组合模块
│   │   │   │   ├── tracking_visualization.h   #   可视化（header-only）
│   │   │   │   ├── env.hpp                    #   A* + 走廊 + 可见性
│   │   │   │   └── prediction.hpp             #   目标预测
│   │   │   ├── src/
│   │   │   │   ├── tracker_node.cpp           #   main() 入口
│   │   │   │   ├── tracking_fsm.cpp           #   FSM 实现
│   │   │   │   ├── mapping.cc                 #   核心建图逻辑
│   │   │   │   ├── occ_grid_mapper.cpp        #   建图模块实现
│   │   │   │   ├── target_ekf.cpp             #   EKF 模块实现
│   │   │   │   └── traj_server.cpp            #   轨迹执行节点
│   │   │   ├── launch/
│   │   │   │   ├── tracking_sim.launch.py     #   完整仿真启动
│   │   │   │   └── rviz.launch.py             #   RViz2 启动
│   │   │   ├── config/
│   │   │   │   ├── tracker_sim.yaml           #   跟踪者参数
│   │   │   │   └── nav_sim.yaml               #   目标机器人参数
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   ├── traj_opt/                          #   MINCO 轨迹优化库
│   │   └── object_detection_msgs/             #   检测消息定义
│   ├── planning/
│   │   └── DecompROS/
│   │       ├── decomp_ros_msgs/               #   多面体消息
│   │       └── decomp_ros_utils/              #   RViz2 插件
│   └── uav_simulator/                         #   仿真器（从 ros2_ego 复制）
│       ├── so3_quadrotor_simulator/
│       ├── so3_control/
│       ├── mockamap/
│       ├── local_sensing/
│       └── Utils/
│           ├── quadrotor_msgs/                #   扩展消息
│           ├── odom_visualization/            #   含 car 可视化
│           └── ...
├── build/                                     #   编译产物
├── install/                                   #   安装目录
└── log/                                       #   编译日志
```

---

*报告生成日期：2026-02-06*
*ROS2 版本：Humble Hawksbill*
*编译环境：Ubuntu 22.04, GCC 11, C++17*
