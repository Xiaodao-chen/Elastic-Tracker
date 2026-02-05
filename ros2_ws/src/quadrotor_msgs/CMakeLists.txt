cmake_minimum_required(VERSION 3.12)
project(quadrotor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES geometry_msgs nav_msgs builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

