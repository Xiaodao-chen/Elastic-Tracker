// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from decomp_ros_msgs:msg/Polyhedron.idl
// generated code does not contain a copyright notice

#include "decomp_ros_msgs/msg/detail/polyhedron__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
const rosidl_type_hash_t *
decomp_ros_msgs__msg__Polyhedron__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xed, 0x45, 0xe8, 0x11, 0xfc, 0x3c, 0xef, 0xfe,
      0xaf, 0x71, 0x2b, 0x2b, 0x4b, 0x0b, 0x1f, 0xec,
      0x95, 0x17, 0xc7, 0x29, 0x6d, 0x1f, 0xd1, 0xbc,
      0x4b, 0xea, 0x69, 0xa8, 0xc0, 0xd5, 0x1c, 0xbf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/point__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
#endif

static char decomp_ros_msgs__msg__Polyhedron__TYPE_NAME[] = "decomp_ros_msgs/msg/Polyhedron";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";

// Define type names, field names, and default values
static char decomp_ros_msgs__msg__Polyhedron__FIELD_NAME__points[] = "points";
static char decomp_ros_msgs__msg__Polyhedron__FIELD_NAME__normals[] = "normals";

static rosidl_runtime_c__type_description__Field decomp_ros_msgs__msg__Polyhedron__FIELDS[] = {
  {
    {decomp_ros_msgs__msg__Polyhedron__FIELD_NAME__points, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {decomp_ros_msgs__msg__Polyhedron__FIELD_NAME__normals, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription decomp_ros_msgs__msg__Polyhedron__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
decomp_ros_msgs__msg__Polyhedron__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {decomp_ros_msgs__msg__Polyhedron__TYPE_NAME, 30, 30},
      {decomp_ros_msgs__msg__Polyhedron__FIELDS, 2, 2},
    },
    {decomp_ros_msgs__msg__Polyhedron__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "geometry_msgs/Point[] points\n"
  "geometry_msgs/Point[] normals  # norm is an outer vector\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
decomp_ros_msgs__msg__Polyhedron__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {decomp_ros_msgs__msg__Polyhedron__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 87, 87},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
decomp_ros_msgs__msg__Polyhedron__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *decomp_ros_msgs__msg__Polyhedron__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
