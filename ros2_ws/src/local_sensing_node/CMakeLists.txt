cmake_minimum_required(VERSION 3.12)
project(local_sensing_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(pcl_render_node src/pcl_render_node_ros2.cpp)
target_link_libraries(pcl_render_node Eigen3::Eigen ${OpenCV_LIBS})
ament_target_dependencies(pcl_render_node
  rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge pcl_conversions pcl_ros
)

install(TARGETS pcl_render_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

