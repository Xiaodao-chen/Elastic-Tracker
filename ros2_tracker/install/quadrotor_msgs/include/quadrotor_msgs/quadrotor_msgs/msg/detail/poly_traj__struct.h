// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/PolyTraj.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__POLY_TRAJ__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__POLY_TRAJ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start_time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'hover_p'
// Member 'coef_x'
// Member 'coef_y'
// Member 'coef_z'
// Member 'duration'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/PolyTraj in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__PolyTraj
{
  int16_t drone_id;
  int32_t traj_id;
  builtin_interfaces__msg__Time start_time;
  bool hover;
  float yaw;
  rosidl_runtime_c__float__Sequence hover_p;
  uint8_t order;
  rosidl_runtime_c__float__Sequence coef_x;
  rosidl_runtime_c__float__Sequence coef_y;
  rosidl_runtime_c__float__Sequence coef_z;
  rosidl_runtime_c__float__Sequence duration;
} quadrotor_msgs__msg__PolyTraj;

// Struct for a sequence of quadrotor_msgs__msg__PolyTraj.
typedef struct quadrotor_msgs__msg__PolyTraj__Sequence
{
  quadrotor_msgs__msg__PolyTraj * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__PolyTraj__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__POLY_TRAJ__STRUCT_H_
