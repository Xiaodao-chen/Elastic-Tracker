// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from decomp_ros_msgs:msg/EllipsoidArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "decomp_ros_msgs/msg/detail/ellipsoid_array__functions.h"
#include "decomp_ros_msgs/msg/detail/ellipsoid_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace decomp_ros_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void EllipsoidArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) decomp_ros_msgs::msg::EllipsoidArray(_init);
}

void EllipsoidArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<decomp_ros_msgs::msg::EllipsoidArray *>(message_memory);
  typed_message->~EllipsoidArray();
}

size_t size_function__EllipsoidArray__ellipsoids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<decomp_ros_msgs::msg::Ellipsoid> *>(untyped_member);
  return member->size();
}

const void * get_const_function__EllipsoidArray__ellipsoids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<decomp_ros_msgs::msg::Ellipsoid> *>(untyped_member);
  return &member[index];
}

void * get_function__EllipsoidArray__ellipsoids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<decomp_ros_msgs::msg::Ellipsoid> *>(untyped_member);
  return &member[index];
}

void fetch_function__EllipsoidArray__ellipsoids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const decomp_ros_msgs::msg::Ellipsoid *>(
    get_const_function__EllipsoidArray__ellipsoids(untyped_member, index));
  auto & value = *reinterpret_cast<decomp_ros_msgs::msg::Ellipsoid *>(untyped_value);
  value = item;
}

void assign_function__EllipsoidArray__ellipsoids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<decomp_ros_msgs::msg::Ellipsoid *>(
    get_function__EllipsoidArray__ellipsoids(untyped_member, index));
  const auto & value = *reinterpret_cast<const decomp_ros_msgs::msg::Ellipsoid *>(untyped_value);
  item = value;
}

void resize_function__EllipsoidArray__ellipsoids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<decomp_ros_msgs::msg::Ellipsoid> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember EllipsoidArray_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs::msg::EllipsoidArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "ellipsoids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<decomp_ros_msgs::msg::Ellipsoid>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs::msg::EllipsoidArray, ellipsoids),  // bytes offset in struct
    nullptr,  // default value
    size_function__EllipsoidArray__ellipsoids,  // size() function pointer
    get_const_function__EllipsoidArray__ellipsoids,  // get_const(index) function pointer
    get_function__EllipsoidArray__ellipsoids,  // get(index) function pointer
    fetch_function__EllipsoidArray__ellipsoids,  // fetch(index, &value) function pointer
    assign_function__EllipsoidArray__ellipsoids,  // assign(index, value) function pointer
    resize_function__EllipsoidArray__ellipsoids  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers EllipsoidArray_message_members = {
  "decomp_ros_msgs::msg",  // message namespace
  "EllipsoidArray",  // message name
  2,  // number of fields
  sizeof(decomp_ros_msgs::msg::EllipsoidArray),
  false,  // has_any_key_member_
  EllipsoidArray_message_member_array,  // message members
  EllipsoidArray_init_function,  // function to initialize message memory (memory has to be allocated)
  EllipsoidArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t EllipsoidArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &EllipsoidArray_message_members,
  get_message_typesupport_handle_function,
  &decomp_ros_msgs__msg__EllipsoidArray__get_type_hash,
  &decomp_ros_msgs__msg__EllipsoidArray__get_type_description,
  &decomp_ros_msgs__msg__EllipsoidArray__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace decomp_ros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<decomp_ros_msgs::msg::EllipsoidArray>()
{
  return &::decomp_ros_msgs::msg::rosidl_typesupport_introspection_cpp::EllipsoidArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, decomp_ros_msgs, msg, EllipsoidArray)() {
  return &::decomp_ros_msgs::msg::rosidl_typesupport_introspection_cpp::EllipsoidArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
