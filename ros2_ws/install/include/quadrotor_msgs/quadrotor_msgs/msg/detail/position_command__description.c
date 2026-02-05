// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quadrotor_msgs:msg/PositionCommand.idl
// generated code does not contain a copyright notice

#include "quadrotor_msgs/msg/detail/position_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
const rosidl_type_hash_t *
quadrotor_msgs__msg__PositionCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc8, 0xa7, 0x57, 0x6f, 0x2b, 0x55, 0x07, 0x9a,
      0x5a, 0x4a, 0x8f, 0x93, 0xbc, 0x9d, 0x87, 0x54,
      0x11, 0x9e, 0x41, 0xdb, 0xfc, 0x0d, 0x47, 0x77,
      0xe8, 0x08, 0xce, 0x8e, 0xc8, 0xea, 0x1d, 0x73,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/point__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char quadrotor_msgs__msg__PositionCommand__TYPE_NAME[] = "quadrotor_msgs/msg/PositionCommand";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__header[] = "header";
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__position[] = "position";
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__velocity[] = "velocity";
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__acceleration[] = "acceleration";
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__jerk[] = "jerk";
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__yaw[] = "yaw";
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__yaw_dot[] = "yaw_dot";
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__kx[] = "kx";
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__kv[] = "kv";
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__trajectory_id[] = "trajectory_id";
static char quadrotor_msgs__msg__PositionCommand__FIELD_NAME__trajectory_flag[] = "trajectory_flag";

static rosidl_runtime_c__type_description__Field quadrotor_msgs__msg__PositionCommand__FIELDS[] = {
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__acceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__jerk, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__yaw_dot, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__kx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__kv, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__trajectory_id, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PositionCommand__FIELD_NAME__trajectory_flag, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription quadrotor_msgs__msg__PositionCommand__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quadrotor_msgs__msg__PositionCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quadrotor_msgs__msg__PositionCommand__TYPE_NAME, 34, 34},
      {quadrotor_msgs__msg__PositionCommand__FIELDS, 11, 11},
    },
    {quadrotor_msgs__msg__PositionCommand__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "geometry_msgs/Point position\n"
  "geometry_msgs/Vector3 velocity\n"
  "geometry_msgs/Vector3 acceleration\n"
  "geometry_msgs/Vector3 jerk\n"
  "float64 yaw\n"
  "float64 yaw_dot\n"
  "float64[3] kx\n"
  "float64[3] kv \n"
  "\n"
  "uint32 trajectory_id\n"
  "\n"
  "uint8 TRAJECTORY_STATUS_EMPTY = 0\n"
  "uint8 TRAJECTORY_STATUS_READY = 1\n"
  "uint8 TRAJECTORY_STATUS_COMPLETED = 3\n"
  "uint8 TRAJECTROY_STATUS_ABORT = 4\n"
  "uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5\n"
  "uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6\n"
  "uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7\n"
  "\n"
  "# Its ID number will start from 1, allowing you comparing it with 0.\n"
  "uint8 trajectory_flag";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quadrotor_msgs__msg__PositionCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quadrotor_msgs__msg__PositionCommand__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 580, 580},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quadrotor_msgs__msg__PositionCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quadrotor_msgs__msg__PositionCommand__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
