// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/TakeoffLand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadrotor_msgs/msg/takeoff_land.h"


#ifndef QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'TAKEOFF'.
enum
{
  quadrotor_msgs__msg__TakeoffLand__TAKEOFF = 1
};

/// Constant 'LAND'.
enum
{
  quadrotor_msgs__msg__TakeoffLand__LAND = 2
};

/// Struct defined in msg/TakeoffLand in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__TakeoffLand
{
  uint8_t takeoff_land_cmd;
} quadrotor_msgs__msg__TakeoffLand;

// Struct for a sequence of quadrotor_msgs__msg__TakeoffLand.
typedef struct quadrotor_msgs__msg__TakeoffLand__Sequence
{
  quadrotor_msgs__msg__TakeoffLand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__TakeoffLand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__STRUCT_H_
