// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from decomp_ros_msgs:msg/PolyhedronArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "decomp_ros_msgs/msg/detail/polyhedron_array__rosidl_typesupport_introspection_c.h"
#include "decomp_ros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "decomp_ros_msgs/msg/detail/polyhedron_array__functions.h"
#include "decomp_ros_msgs/msg/detail/polyhedron_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `polyhedrons`
#include "decomp_ros_msgs/msg/polyhedron.h"
// Member `polyhedrons`
#include "decomp_ros_msgs/msg/detail/polyhedron__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  decomp_ros_msgs__msg__PolyhedronArray__init(message_memory);
}

void decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_fini_function(void * message_memory)
{
  decomp_ros_msgs__msg__PolyhedronArray__fini(message_memory);
}

size_t decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__size_function__PolyhedronArray__polyhedrons(
  const void * untyped_member)
{
  const decomp_ros_msgs__msg__Polyhedron__Sequence * member =
    (const decomp_ros_msgs__msg__Polyhedron__Sequence *)(untyped_member);
  return member->size;
}

const void * decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__get_const_function__PolyhedronArray__polyhedrons(
  const void * untyped_member, size_t index)
{
  const decomp_ros_msgs__msg__Polyhedron__Sequence * member =
    (const decomp_ros_msgs__msg__Polyhedron__Sequence *)(untyped_member);
  return &member->data[index];
}

void * decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__get_function__PolyhedronArray__polyhedrons(
  void * untyped_member, size_t index)
{
  decomp_ros_msgs__msg__Polyhedron__Sequence * member =
    (decomp_ros_msgs__msg__Polyhedron__Sequence *)(untyped_member);
  return &member->data[index];
}

void decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__fetch_function__PolyhedronArray__polyhedrons(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const decomp_ros_msgs__msg__Polyhedron * item =
    ((const decomp_ros_msgs__msg__Polyhedron *)
    decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__get_const_function__PolyhedronArray__polyhedrons(untyped_member, index));
  decomp_ros_msgs__msg__Polyhedron * value =
    (decomp_ros_msgs__msg__Polyhedron *)(untyped_value);
  *value = *item;
}

void decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__assign_function__PolyhedronArray__polyhedrons(
  void * untyped_member, size_t index, const void * untyped_value)
{
  decomp_ros_msgs__msg__Polyhedron * item =
    ((decomp_ros_msgs__msg__Polyhedron *)
    decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__get_function__PolyhedronArray__polyhedrons(untyped_member, index));
  const decomp_ros_msgs__msg__Polyhedron * value =
    (const decomp_ros_msgs__msg__Polyhedron *)(untyped_value);
  *item = *value;
}

bool decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__resize_function__PolyhedronArray__polyhedrons(
  void * untyped_member, size_t size)
{
  decomp_ros_msgs__msg__Polyhedron__Sequence * member =
    (decomp_ros_msgs__msg__Polyhedron__Sequence *)(untyped_member);
  decomp_ros_msgs__msg__Polyhedron__Sequence__fini(member);
  return decomp_ros_msgs__msg__Polyhedron__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs__msg__PolyhedronArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "polyhedrons",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs__msg__PolyhedronArray, polyhedrons),  // bytes offset in struct
    NULL,  // default value
    decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__size_function__PolyhedronArray__polyhedrons,  // size() function pointer
    decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__get_const_function__PolyhedronArray__polyhedrons,  // get_const(index) function pointer
    decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__get_function__PolyhedronArray__polyhedrons,  // get(index) function pointer
    decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__fetch_function__PolyhedronArray__polyhedrons,  // fetch(index, &value) function pointer
    decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__assign_function__PolyhedronArray__polyhedrons,  // assign(index, value) function pointer
    decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__resize_function__PolyhedronArray__polyhedrons  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_message_members = {
  "decomp_ros_msgs__msg",  // message namespace
  "PolyhedronArray",  // message name
  2,  // number of fields
  sizeof(decomp_ros_msgs__msg__PolyhedronArray),
  decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_message_member_array,  // message members
  decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_init_function,  // function to initialize message memory (memory has to be allocated)
  decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_message_type_support_handle = {
  0,
  &decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_decomp_ros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, decomp_ros_msgs, msg, PolyhedronArray)() {
  decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, decomp_ros_msgs, msg, Polyhedron)();
  if (!decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_message_type_support_handle.typesupport_identifier) {
    decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &decomp_ros_msgs__msg__PolyhedronArray__rosidl_typesupport_introspection_c__PolyhedronArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
