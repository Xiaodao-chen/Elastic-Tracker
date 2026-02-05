// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from decomp_ros_msgs:msg/Ellipsoid.idl
// generated code does not contain a copyright notice

#include "decomp_ros_msgs/msg/detail/ellipsoid__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
const rosidl_type_hash_t *
decomp_ros_msgs__msg__Ellipsoid__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x67, 0x95, 0xe3, 0x68, 0x8a, 0x08, 0x60, 0x61,
      0x32, 0x26, 0x47, 0x69, 0x4e, 0x5c, 0x2b, 0x52,
      0xa8, 0xa9, 0x6e, 0x96, 0x8a, 0x3b, 0x1e, 0x76,
      0x5d, 0x6a, 0x10, 0xef, 0xe0, 0x5c, 0xd1, 0x00,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char decomp_ros_msgs__msg__Ellipsoid__TYPE_NAME[] = "decomp_ros_msgs/msg/Ellipsoid";

// Define type names, field names, and default values
static char decomp_ros_msgs__msg__Ellipsoid__FIELD_NAME__d[] = "d";
static char decomp_ros_msgs__msg__Ellipsoid__FIELD_NAME__e[] = "e";

static rosidl_runtime_c__type_description__Field decomp_ros_msgs__msg__Ellipsoid__FIELDS[] = {
  {
    {decomp_ros_msgs__msg__Ellipsoid__FIELD_NAME__d, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {decomp_ros_msgs__msg__Ellipsoid__FIELD_NAME__e, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      9,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
decomp_ros_msgs__msg__Ellipsoid__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {decomp_ros_msgs__msg__Ellipsoid__TYPE_NAME, 29, 29},
      {decomp_ros_msgs__msg__Ellipsoid__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64[3] d\n"
  "float64[9] e\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
decomp_ros_msgs__msg__Ellipsoid__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {decomp_ros_msgs__msg__Ellipsoid__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 27, 27},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
decomp_ros_msgs__msg__Ellipsoid__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *decomp_ros_msgs__msg__Ellipsoid__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
