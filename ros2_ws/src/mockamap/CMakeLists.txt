cmake_minimum_required(VERSION 3.12)
project(mockamap)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io kdtree)

add_executable(mockamap_node
  src/mockamap.cpp
  src/maps.cpp
  src/perlinnoise.cpp
)

target_compile_features(mockamap_node PUBLIC cxx_std_17)
target_include_directories(mockamap_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(mockamap_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
ament_target_dependencies(mockamap_node
  rclcpp sensor_msgs pcl_conversions
)

install(TARGETS mockamap_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()

