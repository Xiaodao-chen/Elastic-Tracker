cmake_minimum_required(VERSION 3.12)
project(odom_visualization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(pose_utils REQUIRED)

find_package(Armadillo REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(odom_visualization_node src/odom_visualization_node_ros2.cpp)
target_compile_features(odom_visualization_node PUBLIC cxx_std_17)
target_include_directories(odom_visualization_node PRIVATE ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(odom_visualization_node ${ARMADILLO_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(odom_visualization_node
  rclcpp geometry_msgs sensor_msgs nav_msgs visualization_msgs tf2 tf2_ros ament_index_cpp quadrotor_msgs pose_utils
)

add_executable(odom_visualization_car_node src/odom_visualization_car_node_ros2.cpp)
target_compile_features(odom_visualization_car_node PUBLIC cxx_std_17)
target_include_directories(odom_visualization_car_node PRIVATE ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(odom_visualization_car_node ${ARMADILLO_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(odom_visualization_car_node
  rclcpp geometry_msgs sensor_msgs nav_msgs visualization_msgs tf2 tf2_ros ament_index_cpp quadrotor_msgs pose_utils
)

install(TARGETS
  odom_visualization_node
  odom_visualization_car_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()

