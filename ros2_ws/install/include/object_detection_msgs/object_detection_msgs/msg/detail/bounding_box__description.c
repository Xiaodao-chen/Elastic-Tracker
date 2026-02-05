// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from object_detection_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#include "object_detection_msgs/msg/detail/bounding_box__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_object_detection_msgs
const rosidl_type_hash_t *
object_detection_msgs__msg__BoundingBox__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9c, 0x78, 0x6c, 0xba, 0xde, 0x83, 0x83, 0x7f,
      0xec, 0xc3, 0x90, 0xa8, 0xd8, 0x4a, 0x02, 0xfd,
      0xc3, 0xd6, 0x2f, 0xe4, 0xf2, 0x72, 0xba, 0xcf,
      0xc6, 0x20, 0x25, 0x28, 0xf2, 0x8c, 0xac, 0xfc,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char object_detection_msgs__msg__BoundingBox__TYPE_NAME[] = "object_detection_msgs/msg/BoundingBox";

// Define type names, field names, and default values
static char object_detection_msgs__msg__BoundingBox__FIELD_NAME__probability[] = "probability";
static char object_detection_msgs__msg__BoundingBox__FIELD_NAME__xmin[] = "xmin";
static char object_detection_msgs__msg__BoundingBox__FIELD_NAME__ymin[] = "ymin";
static char object_detection_msgs__msg__BoundingBox__FIELD_NAME__xmax[] = "xmax";
static char object_detection_msgs__msg__BoundingBox__FIELD_NAME__ymax[] = "ymax";
static char object_detection_msgs__msg__BoundingBox__FIELD_NAME__id[] = "id";
static char object_detection_msgs__msg__BoundingBox__FIELD_NAME__class_name[] = "class_name";

static rosidl_runtime_c__type_description__Field object_detection_msgs__msg__BoundingBox__FIELDS[] = {
  {
    {object_detection_msgs__msg__BoundingBox__FIELD_NAME__probability, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__BoundingBox__FIELD_NAME__xmin, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__BoundingBox__FIELD_NAME__ymin, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__BoundingBox__FIELD_NAME__xmax, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__BoundingBox__FIELD_NAME__ymax, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__BoundingBox__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__BoundingBox__FIELD_NAME__class_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
object_detection_msgs__msg__BoundingBox__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {object_detection_msgs__msg__BoundingBox__TYPE_NAME, 37, 37},
      {object_detection_msgs__msg__BoundingBox__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 probability\n"
  "int64 xmin\n"
  "int64 ymin\n"
  "int64 xmax\n"
  "int64 ymax\n"
  "int16 id\n"
  "string class_name";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
object_detection_msgs__msg__BoundingBox__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {object_detection_msgs__msg__BoundingBox__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 91, 91},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
object_detection_msgs__msg__BoundingBox__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *object_detection_msgs__msg__BoundingBox__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
