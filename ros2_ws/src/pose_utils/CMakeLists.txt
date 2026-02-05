cmake_minimum_required(VERSION 3.12)
project(pose_utils)

find_package(ament_cmake REQUIRED)

find_package(Armadillo REQUIRED)

add_library(${PROJECT_NAME} src/pose_utils.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC armadillo)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

