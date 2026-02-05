// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from decomp_ros_msgs:msg/Polyhedron.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "decomp_ros_msgs/msg/detail/polyhedron__rosidl_typesupport_introspection_c.h"
#include "decomp_ros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "decomp_ros_msgs/msg/detail/polyhedron__functions.h"
#include "decomp_ros_msgs/msg/detail/polyhedron__struct.h"


// Include directives for member types
// Member `points`
// Member `normals`
#include "geometry_msgs/msg/point.h"
// Member `points`
// Member `normals`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  decomp_ros_msgs__msg__Polyhedron__init(message_memory);
}

void decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_fini_function(void * message_memory)
{
  decomp_ros_msgs__msg__Polyhedron__fini(message_memory);
}

size_t decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__size_function__Polyhedron__points(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_const_function__Polyhedron__points(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_function__Polyhedron__points(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__fetch_function__Polyhedron__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_const_function__Polyhedron__points(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__assign_function__Polyhedron__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_function__Polyhedron__points(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__resize_function__Polyhedron__points(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

size_t decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__size_function__Polyhedron__normals(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_const_function__Polyhedron__normals(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_function__Polyhedron__normals(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__fetch_function__Polyhedron__normals(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_const_function__Polyhedron__normals(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__assign_function__Polyhedron__normals(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_function__Polyhedron__normals(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__resize_function__Polyhedron__normals(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_message_member_array[2] = {
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs__msg__Polyhedron, points),  // bytes offset in struct
    NULL,  // default value
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__size_function__Polyhedron__points,  // size() function pointer
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_const_function__Polyhedron__points,  // get_const(index) function pointer
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_function__Polyhedron__points,  // get(index) function pointer
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__fetch_function__Polyhedron__points,  // fetch(index, &value) function pointer
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__assign_function__Polyhedron__points,  // assign(index, value) function pointer
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__resize_function__Polyhedron__points  // resize(index) function pointer
  },
  {
    "normals",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs__msg__Polyhedron, normals),  // bytes offset in struct
    NULL,  // default value
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__size_function__Polyhedron__normals,  // size() function pointer
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_const_function__Polyhedron__normals,  // get_const(index) function pointer
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__get_function__Polyhedron__normals,  // get(index) function pointer
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__fetch_function__Polyhedron__normals,  // fetch(index, &value) function pointer
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__assign_function__Polyhedron__normals,  // assign(index, value) function pointer
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__resize_function__Polyhedron__normals  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_message_members = {
  "decomp_ros_msgs__msg",  // message namespace
  "Polyhedron",  // message name
  2,  // number of fields
  sizeof(decomp_ros_msgs__msg__Polyhedron),
  decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_message_member_array,  // message members
  decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_init_function,  // function to initialize message memory (memory has to be allocated)
  decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_message_type_support_handle = {
  0,
  &decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_decomp_ros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, decomp_ros_msgs, msg, Polyhedron)() {
  decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_message_type_support_handle.typesupport_identifier) {
    decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &decomp_ros_msgs__msg__Polyhedron__rosidl_typesupport_introspection_c__Polyhedron_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
