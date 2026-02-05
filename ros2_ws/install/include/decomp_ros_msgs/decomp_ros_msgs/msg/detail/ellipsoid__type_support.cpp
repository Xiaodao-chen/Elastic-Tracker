// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from decomp_ros_msgs:msg/Ellipsoid.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "decomp_ros_msgs/msg/detail/ellipsoid__struct.hpp"
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

void Ellipsoid_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) decomp_ros_msgs::msg::Ellipsoid(_init);
}

void Ellipsoid_fini_function(void * message_memory)
{
  auto typed_message = static_cast<decomp_ros_msgs::msg::Ellipsoid *>(message_memory);
  typed_message->~Ellipsoid();
}

size_t size_function__Ellipsoid__d(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__Ellipsoid__d(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__Ellipsoid__d(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__Ellipsoid__d(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Ellipsoid__d(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Ellipsoid__d(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Ellipsoid__d(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__Ellipsoid__e(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__Ellipsoid__e(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__Ellipsoid__e(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__Ellipsoid__e(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Ellipsoid__e(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Ellipsoid__e(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Ellipsoid__e(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Ellipsoid_message_member_array[2] = {
  {
    "d",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs::msg::Ellipsoid, d),  // bytes offset in struct
    nullptr,  // default value
    size_function__Ellipsoid__d,  // size() function pointer
    get_const_function__Ellipsoid__d,  // get_const(index) function pointer
    get_function__Ellipsoid__d,  // get(index) function pointer
    fetch_function__Ellipsoid__d,  // fetch(index, &value) function pointer
    assign_function__Ellipsoid__d,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "e",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs::msg::Ellipsoid, e),  // bytes offset in struct
    nullptr,  // default value
    size_function__Ellipsoid__e,  // size() function pointer
    get_const_function__Ellipsoid__e,  // get_const(index) function pointer
    get_function__Ellipsoid__e,  // get(index) function pointer
    fetch_function__Ellipsoid__e,  // fetch(index, &value) function pointer
    assign_function__Ellipsoid__e,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Ellipsoid_message_members = {
  "decomp_ros_msgs::msg",  // message namespace
  "Ellipsoid",  // message name
  2,  // number of fields
  sizeof(decomp_ros_msgs::msg::Ellipsoid),
  Ellipsoid_message_member_array,  // message members
  Ellipsoid_init_function,  // function to initialize message memory (memory has to be allocated)
  Ellipsoid_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Ellipsoid_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Ellipsoid_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace decomp_ros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<decomp_ros_msgs::msg::Ellipsoid>()
{
  return &::decomp_ros_msgs::msg::rosidl_typesupport_introspection_cpp::Ellipsoid_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, decomp_ros_msgs, msg, Ellipsoid)() {
  return &::decomp_ros_msgs::msg::rosidl_typesupport_introspection_cpp::Ellipsoid_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
