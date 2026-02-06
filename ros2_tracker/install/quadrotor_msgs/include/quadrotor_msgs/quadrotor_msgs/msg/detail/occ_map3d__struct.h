// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/OccMap3d.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__STRUCT_H_

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
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/OccMap3d in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__OccMap3d
{
  std_msgs__msg__Header header;
  float resolution;
  int16_t size_x;
  int16_t size_y;
  int16_t size_z;
  int16_t offset_x;
  int16_t offset_y;
  int16_t offset_z;
  rosidl_runtime_c__int8__Sequence data;
} quadrotor_msgs__msg__OccMap3d;

// Struct for a sequence of quadrotor_msgs__msg__OccMap3d.
typedef struct quadrotor_msgs__msg__OccMap3d__Sequence
{
  quadrotor_msgs__msg__OccMap3d * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__OccMap3d__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__STRUCT_H_
