// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quadrotor_msgs:msg/PolyTraj.idl
// generated code does not contain a copyright notice

#include "quadrotor_msgs/msg/detail/poly_traj__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
const rosidl_type_hash_t *
quadrotor_msgs__msg__PolyTraj__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x53, 0x52, 0x95, 0xb0, 0xd2, 0x05, 0x3d, 0xce,
      0xfb, 0x4e, 0x49, 0xf0, 0xc8, 0x89, 0x78, 0x8d,
      0x5c, 0xab, 0x3e, 0x07, 0x24, 0x0f, 0x92, 0xb5,
      0x0f, 0xcf, 0xff, 0xdc, 0xe8, 0x78, 0x05, 0x2b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char quadrotor_msgs__msg__PolyTraj__TYPE_NAME[] = "quadrotor_msgs/msg/PolyTraj";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__drone_id[] = "drone_id";
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__traj_id[] = "traj_id";
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__start_time[] = "start_time";
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__hover[] = "hover";
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__yaw[] = "yaw";
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__hover_p[] = "hover_p";
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__order[] = "order";
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__coef_x[] = "coef_x";
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__coef_y[] = "coef_y";
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__coef_z[] = "coef_z";
static char quadrotor_msgs__msg__PolyTraj__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field quadrotor_msgs__msg__PolyTraj__FIELDS[] = {
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__drone_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__traj_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__start_time, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__hover, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__hover_p, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__order, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__coef_x, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__coef_y, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__coef_z, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__PolyTraj__FIELD_NAME__duration, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription quadrotor_msgs__msg__PolyTraj__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quadrotor_msgs__msg__PolyTraj__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quadrotor_msgs__msg__PolyTraj__TYPE_NAME, 27, 27},
      {quadrotor_msgs__msg__PolyTraj__FIELDS, 11, 11},
    },
    {quadrotor_msgs__msg__PolyTraj__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int16 drone_id\n"
  "int32 traj_id\n"
  "builtin_interfaces/Time start_time\n"
  "\n"
  "bool hover\n"
  "float32 yaw\n"
  "float32[] hover_p\n"
  "\n"
  "uint8 order\n"
  "float32[] coef_x\n"
  "float32[] coef_y\n"
  "float32[] coef_z\n"
  "float32[] duration";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quadrotor_msgs__msg__PolyTraj__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quadrotor_msgs__msg__PolyTraj__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 189, 189},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quadrotor_msgs__msg__PolyTraj__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quadrotor_msgs__msg__PolyTraj__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
