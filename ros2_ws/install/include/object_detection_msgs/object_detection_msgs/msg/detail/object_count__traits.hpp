// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from object_detection_msgs:msg/ObjectCount.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "object_detection_msgs/msg/object_count.hpp"


#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT_COUNT__TRAITS_HPP_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT_COUNT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "object_detection_msgs/msg/detail/object_count__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace object_detection_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectCount & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: count
  {
    out << "count: ";
    rosidl_generator_traits::value_to_yaml(msg.count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectCount & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "count: ";
    rosidl_generator_traits::value_to_yaml(msg.count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectCount & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace object_detection_msgs

namespace rosidl_generator_traits
{

[[deprecated("use object_detection_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const object_detection_msgs::msg::ObjectCount & msg,
  std::ostream & out, size_t indentation = 0)
{
  object_detection_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use object_detection_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const object_detection_msgs::msg::ObjectCount & msg)
{
  return object_detection_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<object_detection_msgs::msg::ObjectCount>()
{
  return "object_detection_msgs::msg::ObjectCount";
}

template<>
inline const char * name<object_detection_msgs::msg::ObjectCount>()
{
  return "object_detection_msgs/msg/ObjectCount";
}

template<>
struct has_fixed_size<object_detection_msgs::msg::ObjectCount>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<object_detection_msgs::msg::ObjectCount>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<object_detection_msgs::msg::ObjectCount>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT_COUNT__TRAITS_HPP_
