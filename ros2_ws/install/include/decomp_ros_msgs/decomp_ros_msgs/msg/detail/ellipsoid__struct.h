// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from decomp_ros_msgs:msg/Ellipsoid.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "decomp_ros_msgs/msg/ellipsoid.h"


#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__STRUCT_H_
#define DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Ellipsoid in the package decomp_ros_msgs.
typedef struct decomp_ros_msgs__msg__Ellipsoid
{
  double d[3];
  double e[9];
} decomp_ros_msgs__msg__Ellipsoid;

// Struct for a sequence of decomp_ros_msgs__msg__Ellipsoid.
typedef struct decomp_ros_msgs__msg__Ellipsoid__Sequence
{
  decomp_ros_msgs__msg__Ellipsoid * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} decomp_ros_msgs__msg__Ellipsoid__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__STRUCT_H_
