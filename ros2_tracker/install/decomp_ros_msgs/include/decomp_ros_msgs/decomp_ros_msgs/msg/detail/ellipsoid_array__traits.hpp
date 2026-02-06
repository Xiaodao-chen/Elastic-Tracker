// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from decomp_ros_msgs:msg/EllipsoidArray.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__TRAITS_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "decomp_ros_msgs/msg/detail/ellipsoid_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'ellipsoids'
#include "decomp_ros_msgs/msg/detail/ellipsoid__traits.hpp"

namespace decomp_ros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EllipsoidArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: ellipsoids
  {
    if (msg.ellipsoids.size() == 0) {
      out << "ellipsoids: []";
    } else {
      out << "ellipsoids: [";
      size_t pending_items = msg.ellipsoids.size();
      for (auto item : msg.ellipsoids) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EllipsoidArray & msg,
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

  // member: ellipsoids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ellipsoids.size() == 0) {
      out << "ellipsoids: []\n";
    } else {
      out << "ellipsoids:\n";
      for (auto item : msg.ellipsoids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EllipsoidArray & msg, bool use_flow_style = false)
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

}  // namespace decomp_ros_msgs

namespace rosidl_generator_traits
{

[[deprecated("use decomp_ros_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const decomp_ros_msgs::msg::EllipsoidArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  decomp_ros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use decomp_ros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const decomp_ros_msgs::msg::EllipsoidArray & msg)
{
  return decomp_ros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<decomp_ros_msgs::msg::EllipsoidArray>()
{
  return "decomp_ros_msgs::msg::EllipsoidArray";
}

template<>
inline const char * name<decomp_ros_msgs::msg::EllipsoidArray>()
{
  return "decomp_ros_msgs/msg/EllipsoidArray";
}

template<>
struct has_fixed_size<decomp_ros_msgs::msg::EllipsoidArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<decomp_ros_msgs::msg::EllipsoidArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<decomp_ros_msgs::msg::EllipsoidArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__TRAITS_HPP_
