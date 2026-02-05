// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from decomp_ros_msgs:msg/PolyhedronArray.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "decomp_ros_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "decomp_ros_msgs/msg/detail/polyhedron_array__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace decomp_ros_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_decomp_ros_msgs
cdr_serialize(
  const decomp_ros_msgs::msg::PolyhedronArray & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_decomp_ros_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  decomp_ros_msgs::msg::PolyhedronArray & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_decomp_ros_msgs
get_serialized_size(
  const decomp_ros_msgs::msg::PolyhedronArray & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_decomp_ros_msgs
max_serialized_size_PolyhedronArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_decomp_ros_msgs
cdr_serialize_key(
  const decomp_ros_msgs::msg::PolyhedronArray & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_decomp_ros_msgs
get_serialized_size_key(
  const decomp_ros_msgs::msg::PolyhedronArray & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_decomp_ros_msgs
max_serialized_size_key_PolyhedronArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace decomp_ros_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_decomp_ros_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, decomp_ros_msgs, msg, PolyhedronArray)();

#ifdef __cplusplus
}
#endif

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
