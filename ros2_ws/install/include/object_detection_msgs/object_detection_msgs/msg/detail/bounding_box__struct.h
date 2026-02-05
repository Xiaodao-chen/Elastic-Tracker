// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from object_detection_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BoundingBox in the package object_detection_msgs.
typedef struct object_detection_msgs__msg__BoundingBox
{
  double probability;
  int64_t xmin;
  int64_t ymin;
  int64_t xmax;
  int64_t ymax;
  int16_t id;
  rosidl_runtime_c__String class_name;
} object_detection_msgs__msg__BoundingBox;

// Struct for a sequence of object_detection_msgs__msg__BoundingBox.
typedef struct object_detection_msgs__msg__BoundingBox__Sequence
{
  object_detection_msgs__msg__BoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} object_detection_msgs__msg__BoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
