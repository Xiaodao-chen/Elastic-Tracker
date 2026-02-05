// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from decomp_ros_msgs:msg/EllipsoidArray.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__STRUCT_H_
#define DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'ellipsoids'
#include "decomp_ros_msgs/msg/detail/ellipsoid__struct.h"

/// Struct defined in msg/EllipsoidArray in the package decomp_ros_msgs.
typedef struct decomp_ros_msgs__msg__EllipsoidArray
{
  std_msgs__msg__Header header;
  decomp_ros_msgs__msg__Ellipsoid__Sequence ellipsoids;
} decomp_ros_msgs__msg__EllipsoidArray;

// Struct for a sequence of decomp_ros_msgs__msg__EllipsoidArray.
typedef struct decomp_ros_msgs__msg__EllipsoidArray__Sequence
{
  decomp_ros_msgs__msg__EllipsoidArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} decomp_ros_msgs__msg__EllipsoidArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__STRUCT_H_
