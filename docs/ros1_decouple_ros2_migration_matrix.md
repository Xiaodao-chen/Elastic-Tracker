# ROS1 解耦 & ROS2 迁移分级矩阵（Elastic-Tracker）

本文基于仓库 `src/` 的扫描结果（`scan_src_ros1_build_and_cpp_deps.json`）与关键源码抽查，总结：

- **哪些包可以“直接”转 ROS2**（以机械替换为主，风险可控）
- **哪些包需要“额外操作/重构”**（nodelet/RViz 插件/dynamic_reconfigure/CUDA/tf1 等）
- **建议的“先解耦再迁移”拆分方式**（把算法 core 与 ROS interface 分离，做到不依赖 ROS1 也能编译核心库）

> 约定：这里的“解耦 ROS1”指 **算法/数据结构核心代码不再依赖 `roscpp`/`ros::*`/`sensor_msgs` 等 ROS1 API**；ROS 相关发布订阅、参数、TF、消息转换放在 wrapper 层。

---

## 分级定义

- **L0（可直接转）**：包结构简单，无 nodelet / plugin / dynamic_reconfigure / tf1 深耦合；迁移主要是 `catkin→ament` + `roscpp→rclcpp` + 少量 API 适配。
- **L1（需要中等改造）**：存在 **轻度 ROS 耦合的库**（例如库里直接读 `ros::NodeHandle` 参数、使用 `ROS_*` 宏、或把 ROS msg 作为库 API），建议先抽 core。
- **L2（需要额外操作/重构）**：nodelet→components、RViz1→RViz2 插件、dynamic_reconfigure→ROS2 参数回调、CUDA/Qt/Ogre 等；需要单独方案。

---

## 包级别结论（按难度分组）

### L0：可直接转 ROS2（机械迁移为主）

- **`quadrotor_msgs`（msg 包）**
  - **现状**：ROS1 `message_generation` + `.msg`
  - **ROS2**：改为 `rosidl_generate_interfaces`
  - **额外操作**：低
- **`object_detection_msgs`（msg 包）**
  - 同上
- **`pose_utils`（数学/工具库）**
  - **现状**：代码本身不依赖 ROS（纯 Armadillo 数学），但包用 catkin 构建
  - **ROS2**：可直接做成 `ament_cmake` 的纯 C++ 库（只依赖 Armadillo）

> 注：`traj_opt` “看起来像算法库”，但目前在库 API 中直接使用 ROS1（见下方 L1）。

### L1：建议先解耦 core，再迁 ROS2（中等改造）

- **`traj_opt`（轨迹优化库）**
  - **发现的 ROS1 耦合点**：
    - 头文件 `include/traj_opt/traj_opt.h` 直接 `#include <ros/ros.h>`，公开成员包含 `ros::NodeHandle nh_`
    - 实现里有 `nh.getParam(...)`、`ROS_ERROR/ROS_WARN_STREAM`
  - **建议解耦方式**：
    - 把参数读取改为传入 `TrajOptParams`（纯 struct）
    - 把日志宏替换为 `std::cerr`/异常/可注入 logger
    - 让核心库只依赖 Eigen/标准库；ROS2 wrapper 再负责读参数（`rclcpp::Node`）
- **`mapping`（含一个核心库 + nodelet + nodes）**
  - **发现的 ROS1 耦合点**：
    - `mapping/src/mapping.cc` 的 `occ2pc(sensor_msgs::PointCloud2&)` 把 ROS msg 作为库 API（应挪到 wrapper）
  - **建议解耦方式**：
    - core：保留 `OccGridMap` 的占据更新、inflate 等纯算法
    - wrapper：负责 `sensor_msgs::PointCloud2` / `pcl::toROSMsg` 等转换与发布
- **`target_ekf`（两个 node）**
  - **耦合点**：`ros::Timer`、全局 publisher、message_filters 同步
  - **ROS2**：总体可直接迁，但需用 ROS2 的 `message_filters`/`rclcpp::TimerBase` 替换；属于“中等改造”
- **`mockamap`（点云地图发布 node）**
  - **耦合点**：`ros::Rate`、`ros::NodeHandle::param`、`ROS_INFO`
  - **ROS2**：较直接；`pcl::toROSMsg` 在 ROS2 仍可用（取决于 `pcl_conversions` ROS2 包）
- **`odom_visualization`（可视化 nodes）**
  - **耦合点**：tf1 broadcaster、大量 ROS msg marker 发布
  - **ROS2**：可迁，但一般需要把 `tf`→`tf2_ros`，marker API 基本一致；属中等改造

### L2：需要额外操作/重构（优先单独立项）

- **nodelet 相关包（必须处理 nodelet→ROS2 components 或改为普通 node）**
  - `mapping`（`mapping_nodelet.cpp`）
  - `planning`（`planning_nodelet.cpp`）
  - `so3_controller`（`so3_controller_nodelet.cpp`）
  - `so3_quadrotor`（`so3_quadrotor_nodelet.cpp`）
  - **额外操作**：
    - ROS2 无 nodelet：需改为 `rclcpp_components`（组件）或普通 `rclcpp::Node` 可执行程序
    - `nodelet_plugin.xml` 需替换为 ROS2 的组件插件描述（常见 `plugin.xml`）
- **`decomp_ros_utils`（RViz 插件 + Qt/Ogre）**
  - **现状**：RViz1 plugin（`rviz::BillboardLine` 等），Qt4/Qt5 条件编译，`catkin_simple`
  - **额外操作**：
    - 迁移到 RViz2 插件体系（依赖 `rviz_common` / `rviz_rendering`，Qt5 为主）
    - CMake/插件描述文件也需更换
- **`local_sensing_node`（动态重配置 + CUDA 分支 + tf1 + image_transport）**
  - **额外操作**：
    - `dynamic_reconfigure` → ROS2 parameters 回调/生命周期节点
    - tf1 → tf2
    - CUDA/自定义 `.cu` 构建在 ament 下要重新整理（通常用 `FindCUDA` 或 `CUDAToolkit` + target 属性）
    - 这包是高风险/高工作量，建议最后迁或单独拆项目
- **`planning/DecompROS/*`（catkin_simple 生态）**
  - `catkin_simple` 在 ROS2 通常不使用，需要去除并改成标准 ament

---

## “不依赖 ROS1 编译”的推荐拆分（核心思想）

为了让代码在没有 ROS1 环境也能构建/测试，建议逐包做两层：

- **`*_core`（纯 C++ 库）**：算法、数学、数据结构；不包含 `ros/ros.h`、不暴露 `sensor_msgs` 等
- **`*_ros`（ROS2 wrapper 包）**：rclcpp 节点、参数、TF、消息转换、发布订阅、launch

优先顺序建议：

1. **先迁 msg 包**：`quadrotor_msgs`、`object_detection_msgs`
2. **抽 core**：`pose_utils`（几乎直接）、`traj_opt`（去 NodeHandle/logging）、`mapping`（去 msg 转换）
3. **迁普通 nodes**：`target_ekf`、`mockamap`、`odom_visualization`
4. **处理 nodelet**：改 components 或 node
5. **最后处理**：`decomp_ros_utils` RViz 插件、`local_sensing_node`（dynamic_reconfigure/CUDA）

---

## 具体“额外操作”清单（命中即加工作量）

- **nodelet**：`nodelet::Nodelet`、`nodelet_plugin.xml`
- **tf1**：`tf/transform_broadcaster.h`、`tf::TransformBroadcaster`
- **dynamic_reconfigure**：`dynamic_reconfigure/server.h`、`generate_dynamic_reconfigure_options`
- **RViz1 插件**：`rviz::`、`Ogre::`、`plugin_description.xml`
- **ros/package.h / roslib**：ROS1 特有 API
- **CUDA**：`.cu` 与 `CUDA_ADD_LIBRARY`

