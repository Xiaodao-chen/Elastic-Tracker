// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from decomp_ros_msgs:msg/Polyhedron.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "decomp_ros_msgs/msg/detail/polyhedron__struct.hpp"
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

void Polyhedron_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) decomp_ros_msgs::msg::Polyhedron(_init);
}

void Polyhedron_fini_function(void * message_memory)
{
  auto typed_message = static_cast<decomp_ros_msgs::msg::Polyhedron *>(message_memory);
  typed_message->~Polyhedron();
}

size_t size_function__Polyhedron__points(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Polyhedron__points(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__Polyhedron__points(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__Polyhedron__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Point *>(
    get_const_function__Polyhedron__points(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__Polyhedron__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Point *>(
    get_function__Polyhedron__points(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__Polyhedron__points(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Polyhedron__normals(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Polyhedron__normals(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__Polyhedron__normals(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__Polyhedron__normals(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Point *>(
    get_const_function__Polyhedron__normals(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__Polyhedron__normals(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Point *>(
    get_function__Polyhedron__normals(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__Polyhedron__normals(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Polyhedron_message_member_array[2] = {
  {
    "points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs::msg::Polyhedron, points),  // bytes offset in struct
    nullptr,  // default value
    size_function__Polyhedron__points,  // size() function pointer
    get_const_function__Polyhedron__points,  // get_const(index) function pointer
    get_function__Polyhedron__points,  // get(index) function pointer
    fetch_function__Polyhedron__points,  // fetch(index, &value) function pointer
    assign_function__Polyhedron__points,  // assign(index, value) function pointer
    resize_function__Polyhedron__points  // resize(index) function pointer
  },
  {
    "normals",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(decomp_ros_msgs::msg::Polyhedron, normals),  // bytes offset in struct
    nullptr,  // default value
    size_function__Polyhedron__normals,  // size() function pointer
    get_const_function__Polyhedron__normals,  // get_const(index) function pointer
    get_function__Polyhedron__normals,  // get(index) function pointer
    fetch_function__Polyhedron__normals,  // fetch(index, &value) function pointer
    assign_function__Polyhedron__normals,  // assign(index, value) function pointer
    resize_function__Polyhedron__normals  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Polyhedron_message_members = {
  "decomp_ros_msgs::msg",  // message namespace
  "Polyhedron",  // message name
  2,  // number of fields
  sizeof(decomp_ros_msgs::msg::Polyhedron),
  Polyhedron_message_member_array,  // message members
  Polyhedron_init_function,  // function to initialize message memory (memory has to be allocated)
  Polyhedron_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Polyhedron_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Polyhedron_message_members,
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
get_message_type_support_handle<decomp_ros_msgs::msg::Polyhedron>()
{
  return &::decomp_ros_msgs::msg::rosidl_typesupport_introspection_cpp::Polyhedron_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, decomp_ros_msgs, msg, Polyhedron)() {
  return &::decomp_ros_msgs::msg::rosidl_typesupport_introspection_cpp::Polyhedron_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
