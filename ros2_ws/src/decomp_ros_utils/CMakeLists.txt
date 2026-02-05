cmake_minimum_required(VERSION 3.12)
project(decomp_ros_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
option(DECOMP_ROS_UTILS_BUILD_RVIZ_PLUGINS "Build RViz2 display plugins" ON)

if(DECOMP_ROS_UTILS_BUILD_RVIZ_PLUGINS)
  add_definitions(-DQT_NO_KEYWORDS)
  set(CMAKE_AUTOMOC ON)

  find_package(pluginlib REQUIRED)
  find_package(rviz_common REQUIRED)
  find_package(rviz_rendering REQUIRED)
  find_package(OGRE REQUIRED)

  # Some ROS2 Humble builds of rviz_rendering do not ship MeshShape.
  include(CheckIncludeFileCXX)
  check_include_file_cxx("rviz_rendering/objects/mesh_shape.hpp" HAVE_RVIZ_MESH_SHAPE_HPP)
  if(NOT HAVE_RVIZ_MESH_SHAPE_HPP)
    message(WARNING
      "rviz_rendering/objects/mesh_shape.hpp not found. "
      "Skipping decomp_rviz_plugins build. "
      "Set -DDECOMP_ROS_UTILS_BUILD_RVIZ_PLUGINS=OFF to silence this warning.")
    set(DECOMP_ROS_UTILS_BUILD_RVIZ_PLUGINS OFF)
  endif()
endif()

if(DECOMP_ROS_UTILS_BUILD_RVIZ_PLUGINS)
  find_package(decomp_ros_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  find_package(Eigen3 REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Widgets)

  add_library(decomp_rviz_plugins SHARED
    src/bound_visual.cpp
    src/mesh_visual.cpp
    src/vector_visual.cpp
    src/ellipsoid_array_visual.cpp
    src/ellipsoid_array_display.cpp
    src/polyhedron_array_display.cpp
  )

  target_include_directories(decomp_rviz_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OGRE_INCLUDE_DIRS}
  )

  ament_target_dependencies(decomp_rviz_plugins
    pluginlib
    rviz_common
    rviz_rendering
    decomp_ros_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
  )

  target_link_libraries(decomp_rviz_plugins
    Qt5::Widgets
    Eigen3::Eigen
  )

  # RViz2 looks for display plugins in the 'rviz_common' plugin category.
  pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

  install(TARGETS decomp_rviz_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  ament_export_libraries(decomp_rviz_plugins)
  ament_export_dependencies(
    pluginlib
    rviz_common
    rviz_rendering
    decomp_ros_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    Eigen3
  )
endif()

# Header-only utilities are used by other packages (e.g. planning), so always export/install them.
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

ament_package()

