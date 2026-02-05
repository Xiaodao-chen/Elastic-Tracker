cmake_minimum_required(VERSION 3.12)
project(planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(mapping_core REQUIRED)
find_package(traj_opt_core REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(decomp_ros_utils REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(planning_component SHARED
  src/planning_component.cpp
)
target_compile_features(planning_component PUBLIC cxx_std_17)
target_include_directories(planning_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(planning_component Eigen3::Eigen)
ament_target_dependencies(planning_component
  rclcpp rclcpp_components std_msgs geometry_msgs nav_msgs visualization_msgs quadrotor_msgs mapping_core traj_opt_core
  decomp_ros_msgs decomp_ros_utils
)
rclcpp_components_register_nodes(planning_component "planning::PlanningComponent")

add_executable(planning_node src/planning_main.cpp)
target_compile_features(planning_node PUBLIC cxx_std_17)
target_link_libraries(planning_node planning_component)
ament_target_dependencies(planning_node rclcpp)

install(TARGETS planning_component planning_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(traj_server_node src/traj_server_node_ros2.cpp)
target_compile_features(traj_server_node PUBLIC cxx_std_17)
target_link_libraries(traj_server_node Eigen3::Eigen)
ament_target_dependencies(traj_server_node
  rclcpp std_msgs quadrotor_msgs traj_opt_core
)

install(TARGETS traj_server_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

