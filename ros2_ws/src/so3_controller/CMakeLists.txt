cmake_minimum_required(VERSION 3.12)
project(so3_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(so3_controller_component SHARED
  src/so3_controller_component.cpp
)
target_compile_features(so3_controller_component PUBLIC cxx_std_17)
target_include_directories(so3_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(so3_controller_component Eigen3::Eigen)
ament_target_dependencies(so3_controller_component
  rclcpp rclcpp_components nav_msgs sensor_msgs quadrotor_msgs tf2 tf2_geometry_msgs
)
rclcpp_components_register_nodes(so3_controller_component "so3_controller::SO3ControllerComponent")

add_executable(so3_controller_node src/so3_controller_main.cpp)
target_compile_features(so3_controller_node PUBLIC cxx_std_17)
target_link_libraries(so3_controller_node so3_controller_component)
ament_target_dependencies(so3_controller_node rclcpp)

install(TARGETS so3_controller_component so3_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

