// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quadrotor_msgs:msg/ReplanState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quadrotor_msgs/msg/detail/replan_state__rosidl_typesupport_introspection_c.h"
#include "quadrotor_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quadrotor_msgs/msg/detail/replan_state__functions.h"
#include "quadrotor_msgs/msg/detail/replan_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `ini_state`
// Member `path`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `target`
#include "nav_msgs/msg/odometry.h"
// Member `target`
#include "nav_msgs/msg/detail/odometry__rosidl_typesupport_introspection_c.h"
// Member `occmap`
#include "quadrotor_msgs/msg/occ_map3d.h"
// Member `occmap`
#include "quadrotor_msgs/msg/detail/occ_map3d__rosidl_typesupport_introspection_c.h"
// Member `replan_stamp`
#include "builtin_interfaces/msg/time.h"
// Member `replan_stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quadrotor_msgs__msg__ReplanState__init(message_memory);
}

void quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_fini_function(void * message_memory)
{
  quadrotor_msgs__msg__ReplanState__fini(message_memory);
}

size_t quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__size_function__ReplanState__ini_state(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_const_function__ReplanState__ini_state(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_function__ReplanState__ini_state(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__fetch_function__ReplanState__ini_state(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_const_function__ReplanState__ini_state(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__assign_function__ReplanState__ini_state(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_function__ReplanState__ini_state(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__resize_function__ReplanState__ini_state(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__size_function__ReplanState__path(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_const_function__ReplanState__path(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_function__ReplanState__path(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__fetch_function__ReplanState__path(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_const_function__ReplanState__path(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__assign_function__ReplanState__path(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_function__ReplanState__path(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__resize_function__ReplanState__path(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__ReplanState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__ReplanState, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ini_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__ReplanState, ini_state),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__size_function__ReplanState__ini_state,  // size() function pointer
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_const_function__ReplanState__ini_state,  // get_const(index) function pointer
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_function__ReplanState__ini_state,  // get(index) function pointer
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__fetch_function__ReplanState__ini_state,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__assign_function__ReplanState__ini_state,  // assign(index, value) function pointer
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__resize_function__ReplanState__ini_state  // resize(index) function pointer
  },
  {
    "target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__ReplanState, target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "occmap",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__ReplanState, occmap),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__ReplanState, path),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__size_function__ReplanState__path,  // size() function pointer
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_const_function__ReplanState__path,  // get_const(index) function pointer
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__get_function__ReplanState__path,  // get(index) function pointer
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__fetch_function__ReplanState__path,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__assign_function__ReplanState__path,  // assign(index, value) function pointer
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__resize_function__ReplanState__path  // resize(index) function pointer
  },
  {
    "replan_stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__ReplanState, replan_stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_members = {
  "quadrotor_msgs__msg",  // message namespace
  "ReplanState",  // message name
  7,  // number of fields
  sizeof(quadrotor_msgs__msg__ReplanState),
  quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_member_array,  // message members
  quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_init_function,  // function to initialize message memory (memory has to be allocated)
  quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_type_support_handle = {
  0,
  &quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quadrotor_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quadrotor_msgs, msg, ReplanState)() {
  quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Odometry)();
  quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quadrotor_msgs, msg, OccMap3d)();
  quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_type_support_handle.typesupport_identifier) {
    quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quadrotor_msgs__msg__ReplanState__rosidl_typesupport_introspection_c__ReplanState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
