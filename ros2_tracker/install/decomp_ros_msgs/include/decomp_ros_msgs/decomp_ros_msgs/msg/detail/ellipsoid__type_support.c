// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from decomp_ros_msgs:msg/Ellipsoid.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "decomp_ros_msgs/msg/detail/ellipsoid__rosidl_typesupport_introspection_c.h"
#include "decomp_ros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "decomp_ros_msgs/msg/detail/ellipsoid__functions.h"
#include "decomp_ros_msgs/msg/detail/ellipsoid__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  decomp_ros_msgs__msg__Ellipsoid__init(message_memory);
}

void decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_fini_function(void * message_memory)
{
  decomp_ros_msgs__msg__Ellipsoid__fini(message_memory);
}

size_t decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__size_function__Ellipsoid__d(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_const_function__Ellipsoid__d(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_function__Ellipsoid__d(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__fetch_function__Ellipsoid__d(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_const_function__Ellipsoid__d(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__assign_function__Ellipsoid__d(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_function__Ellipsoid__d(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__size_function__Ellipsoid__e(
  const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_const_function__Ellipsoid__e(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_function__Ellipsoid__e(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__fetch_function__Ellipsoid__e(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_const_function__Ellipsoid__e(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__assign_function__Ellipsoid__e(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_function__Ellipsoid__e(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_message_member_array[2] = {
  {
    "d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs__msg__Ellipsoid, d),  // bytes offset in struct
    NULL,  // default value
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__size_function__Ellipsoid__d,  // size() function pointer
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_const_function__Ellipsoid__d,  // get_const(index) function pointer
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_function__Ellipsoid__d,  // get(index) function pointer
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__fetch_function__Ellipsoid__d,  // fetch(index, &value) function pointer
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__assign_function__Ellipsoid__d,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "e",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs__msg__Ellipsoid, e),  // bytes offset in struct
    NULL,  // default value
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__size_function__Ellipsoid__e,  // size() function pointer
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_const_function__Ellipsoid__e,  // get_const(index) function pointer
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__get_function__Ellipsoid__e,  // get(index) function pointer
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__fetch_function__Ellipsoid__e,  // fetch(index, &value) function pointer
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__assign_function__Ellipsoid__e,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_message_members = {
  "decomp_ros_msgs__msg",  // message namespace
  "Ellipsoid",  // message name
  2,  // number of fields
  sizeof(decomp_ros_msgs__msg__Ellipsoid),
  decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_message_member_array,  // message members
  decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_init_function,  // function to initialize message memory (memory has to be allocated)
  decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_message_type_support_handle = {
  0,
  &decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_decomp_ros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, decomp_ros_msgs, msg, Ellipsoid)() {
  if (!decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_message_type_support_handle.typesupport_identifier) {
    decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &decomp_ros_msgs__msg__Ellipsoid__rosidl_typesupport_introspection_c__Ellipsoid_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
