// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quadrotor_msgs:msg/ReplanState.idl
// generated code does not contain a copyright notice

#include "quadrotor_msgs/msg/detail/replan_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
const rosidl_type_hash_t *
quadrotor_msgs__msg__ReplanState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc6, 0xf2, 0x8c, 0xa3, 0x3c, 0xf7, 0xad, 0x6c,
      0xaa, 0x51, 0x80, 0x44, 0x9e, 0xce, 0x30, 0x49,
      0x45, 0x4a, 0xa9, 0x98, 0xde, 0xf2, 0x14, 0xa5,
      0x62, 0x35, 0x03, 0xbb, 0xaf, 0xe1, 0x18, 0x18,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/twist_with_covariance__functions.h"
#include "quadrotor_msgs/msg/detail/occ_map3d__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "geometry_msgs/msg/detail/pose__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"
#include "nav_msgs/msg/detail/odometry__functions.h"
#include "geometry_msgs/msg/detail/pose_with_covariance__functions.h"
#include "geometry_msgs/msg/detail/twist__functions.h"

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
static const rosidl_type_hash_t geometry_msgs__msg__Pose__EXPECTED_HASH = {1, {
    0xd5, 0x01, 0x95, 0x4e, 0x94, 0x76, 0xce, 0xa2,
    0x99, 0x69, 0x84, 0xe8, 0x12, 0x05, 0x4b, 0x68,
    0x02, 0x6a, 0xe0, 0xbf, 0xae, 0x78, 0x9d, 0x9a,
    0x10, 0xb2, 0x3d, 0xaf, 0x35, 0xcc, 0x90, 0xfa,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__PoseWithCovariance__EXPECTED_HASH = {1, {
    0x9a, 0x7c, 0x0f, 0xd2, 0x34, 0xb7, 0xf4, 0x5c,
    0x60, 0x98, 0x74, 0x5e, 0xcc, 0xcd, 0x77, 0x3c,
    0xa1, 0x08, 0x56, 0x70, 0xe6, 0x41, 0x07, 0x13,
    0x53, 0x97, 0xae, 0xe3, 0x1c, 0x02, 0xe1, 0xbb,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Twist__EXPECTED_HASH = {1, {
    0x9c, 0x45, 0xbf, 0x16, 0xfe, 0x09, 0x83, 0xd8,
    0x0e, 0x3c, 0xfe, 0x75, 0x0d, 0x68, 0x35, 0x84,
    0x3d, 0x26, 0x5a, 0x9a, 0x6c, 0x46, 0xbd, 0x2e,
    0x60, 0x9f, 0xcd, 0xdd, 0xe6, 0xfb, 0x8d, 0x2a,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__TwistWithCovariance__EXPECTED_HASH = {1, {
    0x49, 0xf5, 0x74, 0xf0, 0x33, 0xf0, 0x95, 0xd8,
    0xb6, 0xcd, 0x1b, 0xea, 0xca, 0x5c, 0xa7, 0x92,
    0x5e, 0x29, 0x6e, 0x84, 0xaf, 0x17, 0x16, 0xd1,
    0x6c, 0x89, 0xd3, 0x8b, 0x05, 0x9c, 0x8c, 0x18,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
static const rosidl_type_hash_t nav_msgs__msg__Odometry__EXPECTED_HASH = {1, {
    0x3c, 0xc9, 0x7d, 0xc7, 0xfb, 0x75, 0x02, 0xf8,
    0x71, 0x44, 0x62, 0xc5, 0x26, 0xd3, 0x69, 0xe3,
    0x5b, 0x60, 0x3c, 0xfc, 0x34, 0xd9, 0x46, 0xe3,
    0xf2, 0xed, 0xa2, 0x76, 0x6d, 0xfe, 0xc6, 0xe0,
  }};
static const rosidl_type_hash_t quadrotor_msgs__msg__OccMap3d__EXPECTED_HASH = {1, {
    0xe0, 0x08, 0x6a, 0x17, 0x87, 0x3d, 0xd0, 0x87,
    0xad, 0x37, 0x2c, 0xaa, 0xa6, 0x05, 0x7c, 0x1e,
    0x46, 0xc0, 0x7b, 0x5b, 0x0c, 0x3d, 0x1e, 0xc0,
    0x29, 0xa1, 0xef, 0x20, 0x4c, 0x13, 0x44, 0x50,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char quadrotor_msgs__msg__ReplanState__TYPE_NAME[] = "quadrotor_msgs/msg/ReplanState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Pose__TYPE_NAME[] = "geometry_msgs/msg/Pose";
static char geometry_msgs__msg__PoseWithCovariance__TYPE_NAME[] = "geometry_msgs/msg/PoseWithCovariance";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char geometry_msgs__msg__Twist__TYPE_NAME[] = "geometry_msgs/msg/Twist";
static char geometry_msgs__msg__TwistWithCovariance__TYPE_NAME[] = "geometry_msgs/msg/TwistWithCovariance";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";
static char nav_msgs__msg__Odometry__TYPE_NAME[] = "nav_msgs/msg/Odometry";
static char quadrotor_msgs__msg__OccMap3d__TYPE_NAME[] = "quadrotor_msgs/msg/OccMap3d";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char quadrotor_msgs__msg__ReplanState__FIELD_NAME__header[] = "header";
static char quadrotor_msgs__msg__ReplanState__FIELD_NAME__state[] = "state";
static char quadrotor_msgs__msg__ReplanState__FIELD_NAME__ini_state[] = "ini_state";
static char quadrotor_msgs__msg__ReplanState__FIELD_NAME__target[] = "target";
static char quadrotor_msgs__msg__ReplanState__FIELD_NAME__occmap[] = "occmap";
static char quadrotor_msgs__msg__ReplanState__FIELD_NAME__path[] = "path";
static char quadrotor_msgs__msg__ReplanState__FIELD_NAME__replan_stamp[] = "replan_stamp";

static rosidl_runtime_c__type_description__Field quadrotor_msgs__msg__ReplanState__FIELDS[] = {
  {
    {quadrotor_msgs__msg__ReplanState__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__ReplanState__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__ReplanState__FIELD_NAME__ini_state, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__ReplanState__FIELD_NAME__target, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nav_msgs__msg__Odometry__TYPE_NAME, 21, 21},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__ReplanState__FIELD_NAME__occmap, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {quadrotor_msgs__msg__OccMap3d__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__ReplanState__FIELD_NAME__path, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__ReplanState__FIELD_NAME__replan_stamp, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription quadrotor_msgs__msg__ReplanState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__PoseWithCovariance__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Twist__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__TwistWithCovariance__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {nav_msgs__msg__Odometry__TYPE_NAME, 21, 21},
    {NULL, 0, 0},
  },
  {
    {quadrotor_msgs__msg__OccMap3d__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quadrotor_msgs__msg__ReplanState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quadrotor_msgs__msg__ReplanState__TYPE_NAME, 30, 30},
      {quadrotor_msgs__msg__ReplanState__FIELDS, 7, 7},
    },
    {quadrotor_msgs__msg__ReplanState__REFERENCED_TYPE_DESCRIPTIONS, 11, 11},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__PoseWithCovariance__EXPECTED_HASH, geometry_msgs__msg__PoseWithCovariance__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__PoseWithCovariance__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Twist__EXPECTED_HASH, geometry_msgs__msg__Twist__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Twist__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__TwistWithCovariance__EXPECTED_HASH, geometry_msgs__msg__TwistWithCovariance__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = geometry_msgs__msg__TwistWithCovariance__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_msgs__msg__Odometry__EXPECTED_HASH, nav_msgs__msg__Odometry__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = nav_msgs__msg__Odometry__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quadrotor_msgs__msg__OccMap3d__EXPECTED_HASH, quadrotor_msgs__msg__OccMap3d__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = quadrotor_msgs__msg__OccMap3d__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[10].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# all information for a replan\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "int16 state\n"
  "float64[] ini_state\n"
  "nav_msgs/Odometry target\n"
  "quadrotor_msgs/OccMap3d occmap\n"
  "\n"
  "# other temporal variables\n"
  "float64[] path\n"
  "builtin_interfaces/Time replan_stamp";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quadrotor_msgs__msg__ReplanState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quadrotor_msgs__msg__ReplanState__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 223, 223},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quadrotor_msgs__msg__ReplanState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[12];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 12, 12};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quadrotor_msgs__msg__ReplanState__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__PoseWithCovariance__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Twist__get_individual_type_description_source(NULL);
    sources[7] = *geometry_msgs__msg__TwistWithCovariance__get_individual_type_description_source(NULL);
    sources[8] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[9] = *nav_msgs__msg__Odometry__get_individual_type_description_source(NULL);
    sources[10] = *quadrotor_msgs__msg__OccMap3d__get_individual_type_description_source(NULL);
    sources[11] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
