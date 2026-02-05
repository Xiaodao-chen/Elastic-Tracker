cmake_minimum_required(VERSION 3.12)
project(traj_opt)

find_package(ament_cmake REQUIRED)
find_package(traj_opt_core REQUIRED)

# Compatibility wrapper: downstream code can `find_package(traj_opt)` and link `traj_opt`.
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE traj_opt_core)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(traj_opt_core)
ament_package()

