cmake_minimum_required(VERSION 3.12)
project(so3_quadrotor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(so3_quadrotor_component SHARED
  src/so3_quadrotor_component.cpp
)
target_compile_features(so3_quadrotor_component PUBLIC cxx_std_17)
target_include_directories(so3_quadrotor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(so3_quadrotor_component Eigen3::Eigen)
ament_target_dependencies(so3_quadrotor_component
  rclcpp rclcpp_components quadrotor_msgs nav_msgs sensor_msgs visualization_msgs geometry_msgs tf2 tf2_ros
)
rclcpp_components_register_nodes(so3_quadrotor_component "so3_quadrotor::So3QuadrotorComponent")

add_executable(so3_quadrotor_node src/so3_quadrotor_main.cpp)
target_compile_features(so3_quadrotor_node PUBLIC cxx_std_17)
target_link_libraries(so3_quadrotor_node so3_quadrotor_component)
ament_target_dependencies(so3_quadrotor_node rclcpp)

install(TARGETS so3_quadrotor_component so3_quadrotor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

