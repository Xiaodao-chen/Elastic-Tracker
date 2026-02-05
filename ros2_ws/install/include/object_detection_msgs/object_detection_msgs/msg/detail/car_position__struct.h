// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from object_detection_msgs:msg/CarPosition.idl
// generated code does not contain a copyright notice

#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__CAR_POSITION__STRUCT_H_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__CAR_POSITION__STRUCT_H_

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

/// Struct defined in msg/CarPosition in the package object_detection_msgs.
typedef struct object_detection_msgs__msg__CarPosition
{
  std_msgs__msg__Header header;
  double xmean;
  double ymean;
  double zmean;
} object_detection_msgs__msg__CarPosition;

// Struct for a sequence of object_detection_msgs__msg__CarPosition.
typedef struct object_detection_msgs__msg__CarPosition__Sequence
{
  object_detection_msgs__msg__CarPosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} object_detection_msgs__msg__CarPosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__CAR_POSITION__STRUCT_H_
