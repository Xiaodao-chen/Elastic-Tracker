cmake_minimum_required(VERSION 3.12)
project(mapping)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(mapping_core REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(mapping_component SHARED
  src/mapping_component.cpp
)
target_compile_features(mapping_component PUBLIC cxx_std_17)
target_include_directories(mapping_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mapping_core_INCLUDE_DIRS}
)
target_link_libraries(mapping_component Eigen3::Eigen ${mapping_core_LIBRARIES})
ament_target_dependencies(mapping_component
  rclcpp rclcpp_components message_filters sensor_msgs nav_msgs geometry_msgs quadrotor_msgs mapping_core
  cv_bridge pcl_conversions pcl_ros
)
rclcpp_components_register_nodes(mapping_component "mapping::MappingComponent")

add_executable(mapping_node src/mapping_main.cpp)
target_compile_features(mapping_node PUBLIC cxx_std_17)
target_link_libraries(mapping_node mapping_component)
ament_target_dependencies(mapping_node rclcpp)

install(TARGETS mapping_component mapping_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(mapping_vis_node src/mapping_vis_node_ros2.cpp)
target_compile_features(mapping_vis_node PUBLIC cxx_std_17)
target_link_libraries(mapping_vis_node Eigen3::Eigen ${mapping_core_LIBRARIES})
ament_target_dependencies(mapping_vis_node
  rclcpp sensor_msgs quadrotor_msgs mapping_core pcl_conversions pcl_ros
)

add_executable(visualize_history_path_node src/visualize_history_path_node_ros2.cpp)
target_compile_features(visualize_history_path_node PUBLIC cxx_std_17)
target_link_libraries(visualize_history_path_node Eigen3::Eigen)
ament_target_dependencies(visualize_history_path_node
  rclcpp nav_msgs geometry_msgs visualization_msgs quadrotor_msgs
)

install(TARGETS mapping_vis_node visualize_history_path_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

