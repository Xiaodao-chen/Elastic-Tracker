cmake_minimum_required(VERSION 3.12)
project(uav_simulator)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

