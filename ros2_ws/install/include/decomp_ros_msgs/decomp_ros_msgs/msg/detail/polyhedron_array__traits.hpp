// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from decomp_ros_msgs:msg/PolyhedronArray.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__TRAITS_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "decomp_ros_msgs/msg/detail/polyhedron_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'polyhedrons'
#include "decomp_ros_msgs/msg/detail/polyhedron__traits.hpp"

namespace decomp_ros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PolyhedronArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: polyhedrons
  {
    if (msg.polyhedrons.size() == 0) {
      out << "polyhedrons: []";
    } else {
      out << "polyhedrons: [";
      size_t pending_items = msg.polyhedrons.size();
      for (auto item : msg.polyhedrons) {
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
  const PolyhedronArray & msg,
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

  // member: polyhedrons
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.polyhedrons.size() == 0) {
      out << "polyhedrons: []\n";
    } else {
      out << "polyhedrons:\n";
      for (auto item : msg.polyhedrons) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PolyhedronArray & msg, bool use_flow_style = false)
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
  const decomp_ros_msgs::msg::PolyhedronArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  decomp_ros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use decomp_ros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const decomp_ros_msgs::msg::PolyhedronArray & msg)
{
  return decomp_ros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<decomp_ros_msgs::msg::PolyhedronArray>()
{
  return "decomp_ros_msgs::msg::PolyhedronArray";
}

template<>
inline const char * name<decomp_ros_msgs::msg::PolyhedronArray>()
{
  return "decomp_ros_msgs/msg/PolyhedronArray";
}

template<>
struct has_fixed_size<decomp_ros_msgs::msg::PolyhedronArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<decomp_ros_msgs::msg::PolyhedronArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<decomp_ros_msgs::msg::PolyhedronArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__TRAITS_HPP_
