// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quadrotor_msgs:msg/TakeoffLand.idl
// generated code does not contain a copyright notice

#include "quadrotor_msgs/msg/detail/takeoff_land__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
const rosidl_type_hash_t *
quadrotor_msgs__msg__TakeoffLand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1f, 0x32, 0x87, 0xfc, 0x68, 0x03, 0xf3, 0xe5,
      0xaf, 0xdd, 0xd8, 0x37, 0x4f, 0x1e, 0x10, 0x4e,
      0x5d, 0xed, 0x2d, 0x04, 0x1f, 0x27, 0x04, 0x9f,
      0x1a, 0xf9, 0x15, 0xf9, 0xad, 0xa7, 0x84, 0xbd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char quadrotor_msgs__msg__TakeoffLand__TYPE_NAME[] = "quadrotor_msgs/msg/TakeoffLand";

// Define type names, field names, and default values
static char quadrotor_msgs__msg__TakeoffLand__FIELD_NAME__takeoff_land_cmd[] = "takeoff_land_cmd";

static rosidl_runtime_c__type_description__Field quadrotor_msgs__msg__TakeoffLand__FIELDS[] = {
  {
    {quadrotor_msgs__msg__TakeoffLand__FIELD_NAME__takeoff_land_cmd, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quadrotor_msgs__msg__TakeoffLand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quadrotor_msgs__msg__TakeoffLand__TYPE_NAME, 30, 30},
      {quadrotor_msgs__msg__TakeoffLand__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 TAKEOFF = 1\n"
  "uint8 LAND = 2\n"
  "uint8 takeoff_land_cmd";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quadrotor_msgs__msg__TakeoffLand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quadrotor_msgs__msg__TakeoffLand__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 55, 55},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quadrotor_msgs__msg__TakeoffLand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quadrotor_msgs__msg__TakeoffLand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
