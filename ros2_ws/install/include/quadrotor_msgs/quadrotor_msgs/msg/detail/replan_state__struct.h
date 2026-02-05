// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/ReplanState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadrotor_msgs/msg/replan_state.h"


#ifndef QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__STRUCT_H_

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
// Member 'ini_state'
// Member 'path'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'target'
#include "nav_msgs/msg/detail/odometry__struct.h"
// Member 'occmap'
#include "quadrotor_msgs/msg/detail/occ_map3d__struct.h"
// Member 'replan_stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/ReplanState in the package quadrotor_msgs.
/**
  * all information for a replan
 */
typedef struct quadrotor_msgs__msg__ReplanState
{
  std_msgs__msg__Header header;
  int16_t state;
  rosidl_runtime_c__double__Sequence ini_state;
  nav_msgs__msg__Odometry target;
  quadrotor_msgs__msg__OccMap3d occmap;
  /// other temporal variables
  rosidl_runtime_c__double__Sequence path;
  builtin_interfaces__msg__Time replan_stamp;
} quadrotor_msgs__msg__ReplanState;

// Struct for a sequence of quadrotor_msgs__msg__ReplanState.
typedef struct quadrotor_msgs__msg__ReplanState__Sequence
{
  quadrotor_msgs__msg__ReplanState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__ReplanState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__STRUCT_H_
