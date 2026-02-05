// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from decomp_ros_msgs:msg/Polyhedron.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__STRUCT_H_
#define DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'points'
// Member 'normals'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Polyhedron in the package decomp_ros_msgs.
typedef struct decomp_ros_msgs__msg__Polyhedron
{
  geometry_msgs__msg__Point__Sequence points;
  /// norm is an outer vector
  geometry_msgs__msg__Point__Sequence normals;
} decomp_ros_msgs__msg__Polyhedron;

// Struct for a sequence of decomp_ros_msgs__msg__Polyhedron.
typedef struct decomp_ros_msgs__msg__Polyhedron__Sequence
{
  decomp_ros_msgs__msg__Polyhedron * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} decomp_ros_msgs__msg__Polyhedron__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__STRUCT_H_
