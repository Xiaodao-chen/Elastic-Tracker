cmake_minimum_required(VERSION 3.8)
project(tracker_fsm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(object_detection_msgs REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(decomp_ros_utils REQUIRED)
find_package(traj_opt REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Create shared library tracker_fsm_lib
add_library(tracker_fsm_lib SHARED
  src/mapping.cc
  src/occ_grid_mapper.cpp
  src/target_ekf.cpp
  src/tracking_fsm.cpp
)

ament_target_dependencies(tracker_fsm_lib
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  quadrotor_msgs
  object_detection_msgs
  decomp_ros_msgs
  decomp_ros_utils
  traj_opt
  message_filters
  cv_bridge
  pcl_conversions
  tf2_ros
  Eigen3
  PCL
)

# Create executable tracker_node
add_executable(tracker_node
  src/tracker_node.cpp
)

ament_target_dependencies(tracker_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  quadrotor_msgs
  object_detection_msgs
  decomp_ros_msgs
  decomp_ros_utils
  traj_opt
  message_filters
  cv_bridge
  pcl_conversions
  tf2_ros
  Eigen3
  PCL
)

target_link_libraries(tracker_node
  tracker_fsm_lib
)

# Create executable traj_server
add_executable(traj_server
  src/traj_server.cpp
)

ament_target_dependencies(traj_server
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
  quadrotor_msgs
  std_msgs
  traj_opt
)

# Set RPATH so executables can find libtracker_fsm_lib.so without LD_LIBRARY_PATH
# Executables are in lib/tracker_fsm/, library is in lib/ — relative path is "../"
set_target_properties(tracker_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
  BUILD_WITH_INSTALL_RPATH FALSE
)
set_target_properties(traj_server PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
  BUILD_WITH_INSTALL_RPATH FALSE
)

# Install shared library to lib/ so it's on LD_LIBRARY_PATH
install(
  TARGETS tracker_fsm_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables to lib/${PROJECT_NAME}/ (standard ROS2 convention)
install(
  TARGETS tracker_node traj_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
