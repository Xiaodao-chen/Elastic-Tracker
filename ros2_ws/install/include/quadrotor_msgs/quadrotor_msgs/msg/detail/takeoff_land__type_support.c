// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quadrotor_msgs:msg/TakeoffLand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quadrotor_msgs/msg/detail/takeoff_land__rosidl_typesupport_introspection_c.h"
#include "quadrotor_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quadrotor_msgs/msg/detail/takeoff_land__functions.h"
#include "quadrotor_msgs/msg/detail/takeoff_land__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quadrotor_msgs__msg__TakeoffLand__init(message_memory);
}

void quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_fini_function(void * message_memory)
{
  quadrotor_msgs__msg__TakeoffLand__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_message_member_array[1] = {
  {
    "takeoff_land_cmd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__TakeoffLand, takeoff_land_cmd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_message_members = {
  "quadrotor_msgs__msg",  // message namespace
  "TakeoffLand",  // message name
  1,  // number of fields
  sizeof(quadrotor_msgs__msg__TakeoffLand),
  false,  // has_any_key_member_
  quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_message_member_array,  // message members
  quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_init_function,  // function to initialize message memory (memory has to be allocated)
  quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_message_type_support_handle = {
  0,
  &quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_message_members,
  get_message_typesupport_handle_function,
  &quadrotor_msgs__msg__TakeoffLand__get_type_hash,
  &quadrotor_msgs__msg__TakeoffLand__get_type_description,
  &quadrotor_msgs__msg__TakeoffLand__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quadrotor_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quadrotor_msgs, msg, TakeoffLand)() {
  if (!quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_message_type_support_handle.typesupport_identifier) {
    quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quadrotor_msgs__msg__TakeoffLand__rosidl_typesupport_introspection_c__TakeoffLand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
