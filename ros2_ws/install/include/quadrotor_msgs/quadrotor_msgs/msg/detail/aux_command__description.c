// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quadrotor_msgs:msg/AuxCommand.idl
// generated code does not contain a copyright notice

#include "quadrotor_msgs/msg/detail/aux_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
const rosidl_type_hash_t *
quadrotor_msgs__msg__AuxCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe6, 0xc5, 0x9b, 0xa9, 0xd6, 0x8b, 0xec, 0x6c,
      0x95, 0xe3, 0xde, 0x21, 0xa2, 0x6e, 0xc4, 0xa9,
      0x04, 0x91, 0xd4, 0x79, 0x4a, 0x38, 0x32, 0xfe,
      0x3d, 0x79, 0x42, 0xdd, 0x55, 0x4a, 0x52, 0xf1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char quadrotor_msgs__msg__AuxCommand__TYPE_NAME[] = "quadrotor_msgs/msg/AuxCommand";

// Define type names, field names, and default values
static char quadrotor_msgs__msg__AuxCommand__FIELD_NAME__current_yaw[] = "current_yaw";
static char quadrotor_msgs__msg__AuxCommand__FIELD_NAME__kf_correction[] = "kf_correction";
static char quadrotor_msgs__msg__AuxCommand__FIELD_NAME__angle_corrections[] = "angle_corrections";
static char quadrotor_msgs__msg__AuxCommand__FIELD_NAME__enable_motors[] = "enable_motors";
static char quadrotor_msgs__msg__AuxCommand__FIELD_NAME__use_external_yaw[] = "use_external_yaw";

static rosidl_runtime_c__type_description__Field quadrotor_msgs__msg__AuxCommand__FIELDS[] = {
  {
    {quadrotor_msgs__msg__AuxCommand__FIELD_NAME__current_yaw, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__AuxCommand__FIELD_NAME__kf_correction, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__AuxCommand__FIELD_NAME__angle_corrections, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__AuxCommand__FIELD_NAME__enable_motors, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__AuxCommand__FIELD_NAME__use_external_yaw, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quadrotor_msgs__msg__AuxCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quadrotor_msgs__msg__AuxCommand__TYPE_NAME, 29, 29},
      {quadrotor_msgs__msg__AuxCommand__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 current_yaw\n"
  "float64 kf_correction\n"
  "float64[2] angle_corrections # Trims for roll, pitch\n"
  "bool enable_motors\n"
  "bool use_external_yaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quadrotor_msgs__msg__AuxCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quadrotor_msgs__msg__AuxCommand__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 136, 136},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quadrotor_msgs__msg__AuxCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quadrotor_msgs__msg__AuxCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
