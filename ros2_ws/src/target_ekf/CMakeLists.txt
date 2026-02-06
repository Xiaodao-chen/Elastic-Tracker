cmake_minimum_required(VERSION 3.12)
project(target_ekf)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(object_detection_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(target_ekf_node src/target_ekf_node.cpp)
target_compile_features(target_ekf_node PUBLIC cxx_std_17)
ament_target_dependencies(target_ekf_node
  rclcpp message_filters nav_msgs geometry_msgs object_detection_msgs
)
target_link_libraries(target_ekf_node Eigen3::Eigen)

add_executable(target_ekf_sim_node src/target_ekf_sim_node.cpp)
target_compile_features(target_ekf_sim_node PUBLIC cxx_std_17)
ament_target_dependencies(target_ekf_sim_node
  rclcpp message_filters nav_msgs
)
target_link_libraries(target_ekf_sim_node Eigen3::Eigen)

add_executable(fake_target_pub_node src/fake_target_pub_node.cpp)
target_compile_features(fake_target_pub_node PUBLIC cxx_std_17)
ament_target_dependencies(fake_target_pub_node
  rclcpp nav_msgs
)

install(TARGETS
  target_ekf_node
  target_ekf_sim_node
  fake_target_pub_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

