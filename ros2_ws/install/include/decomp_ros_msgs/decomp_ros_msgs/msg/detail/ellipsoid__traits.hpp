// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from decomp_ros_msgs:msg/Ellipsoid.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__TRAITS_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "decomp_ros_msgs/msg/detail/ellipsoid__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace decomp_ros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Ellipsoid & msg,
  std::ostream & out)
{
  out << "{";
  // member: d
  {
    if (msg.d.size() == 0) {
      out << "d: []";
    } else {
      out << "d: [";
      size_t pending_items = msg.d.size();
      for (auto item : msg.d) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: e
  {
    if (msg.e.size() == 0) {
      out << "e: []";
    } else {
      out << "e: [";
      size_t pending_items = msg.e.size();
      for (auto item : msg.e) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const Ellipsoid & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.d.size() == 0) {
      out << "d: []\n";
    } else {
      out << "d:\n";
      for (auto item : msg.d) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.e.size() == 0) {
      out << "e: []\n";
    } else {
      out << "e:\n";
      for (auto item : msg.e) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Ellipsoid & msg, bool use_flow_style = false)
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
  const decomp_ros_msgs::msg::Ellipsoid & msg,
  std::ostream & out, size_t indentation = 0)
{
  decomp_ros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use decomp_ros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const decomp_ros_msgs::msg::Ellipsoid & msg)
{
  return decomp_ros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<decomp_ros_msgs::msg::Ellipsoid>()
{
  return "decomp_ros_msgs::msg::Ellipsoid";
}

template<>
inline const char * name<decomp_ros_msgs::msg::Ellipsoid>()
{
  return "decomp_ros_msgs/msg/Ellipsoid";
}

template<>
struct has_fixed_size<decomp_ros_msgs::msg::Ellipsoid>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<decomp_ros_msgs::msg::Ellipsoid>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<decomp_ros_msgs::msg::Ellipsoid>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__TRAITS_HPP_
