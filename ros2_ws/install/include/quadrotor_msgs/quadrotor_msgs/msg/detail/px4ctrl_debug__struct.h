// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/Px4ctrlDebug.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__STRUCT_H_

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

/// Struct defined in msg/Px4ctrlDebug in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__Px4ctrlDebug
{
  std_msgs__msg__Header header;
  double des_v_x;
  double des_v_y;
  double des_v_z;
  double fb_a_x;
  double fb_a_y;
  double fb_a_z;
  double des_a_x;
  double des_a_y;
  double des_a_z;
  double des_q_x;
  double des_q_y;
  double des_q_z;
  double des_q_w;
  double des_thr;
  double thr2acc;
  double thr_scale_compensate;
  double voltage;
  double err_axisang_x;
  double err_axisang_y;
  double err_axisang_z;
  double err_axisang_ang;
  double fb_rate_x;
  double fb_rate_y;
  double fb_rate_z;
} quadrotor_msgs__msg__Px4ctrlDebug;

// Struct for a sequence of quadrotor_msgs__msg__Px4ctrlDebug.
typedef struct quadrotor_msgs__msg__Px4ctrlDebug__Sequence
{
  quadrotor_msgs__msg__Px4ctrlDebug * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__Px4ctrlDebug__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__STRUCT_H_
