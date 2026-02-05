// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quadrotor_msgs:msg/OccMap3d.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quadrotor_msgs/msg/detail/occ_map3d__rosidl_typesupport_introspection_c.h"
#include "quadrotor_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quadrotor_msgs/msg/detail/occ_map3d__functions.h"
#include "quadrotor_msgs/msg/detail/occ_map3d__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quadrotor_msgs__msg__OccMap3d__init(message_memory);
}

void quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_fini_function(void * message_memory)
{
  quadrotor_msgs__msg__OccMap3d__fini(message_memory);
}

size_t quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__size_function__OccMap3d__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__int8__Sequence * member =
    (const rosidl_runtime_c__int8__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__get_const_function__OccMap3d__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int8__Sequence * member =
    (const rosidl_runtime_c__int8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__get_function__OccMap3d__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int8__Sequence * member =
    (rosidl_runtime_c__int8__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__fetch_function__OccMap3d__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int8_t * item =
    ((const int8_t *)
    quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__get_const_function__OccMap3d__data(untyped_member, index));
  int8_t * value =
    (int8_t *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__assign_function__OccMap3d__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int8_t * item =
    ((int8_t *)
    quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__get_function__OccMap3d__data(untyped_member, index));
  const int8_t * value =
    (const int8_t *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__resize_function__OccMap3d__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int8__Sequence * member =
    (rosidl_runtime_c__int8__Sequence *)(untyped_member);
  rosidl_runtime_c__int8__Sequence__fini(member);
  return rosidl_runtime_c__int8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__OccMap3d, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "resolution",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__OccMap3d, resolution),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "size_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__OccMap3d, size_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "size_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__OccMap3d, size_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "size_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__OccMap3d, size_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "offset_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__OccMap3d, offset_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "offset_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__OccMap3d, offset_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "offset_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__OccMap3d, offset_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__OccMap3d, data),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__size_function__OccMap3d__data,  // size() function pointer
    quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__get_const_function__OccMap3d__data,  // get_const(index) function pointer
    quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__get_function__OccMap3d__data,  // get(index) function pointer
    quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__fetch_function__OccMap3d__data,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__assign_function__OccMap3d__data,  // assign(index, value) function pointer
    quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__resize_function__OccMap3d__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_message_members = {
  "quadrotor_msgs__msg",  // message namespace
  "OccMap3d",  // message name
  9,  // number of fields
  sizeof(quadrotor_msgs__msg__OccMap3d),
  false,  // has_any_key_member_
  quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_message_member_array,  // message members
  quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_init_function,  // function to initialize message memory (memory has to be allocated)
  quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_message_type_support_handle = {
  0,
  &quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_message_members,
  get_message_typesupport_handle_function,
  &quadrotor_msgs__msg__OccMap3d__get_type_hash,
  &quadrotor_msgs__msg__OccMap3d__get_type_description,
  &quadrotor_msgs__msg__OccMap3d__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quadrotor_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quadrotor_msgs, msg, OccMap3d)() {
  quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_message_type_support_handle.typesupport_identifier) {
    quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quadrotor_msgs__msg__OccMap3d__rosidl_typesupport_introspection_c__OccMap3d_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
