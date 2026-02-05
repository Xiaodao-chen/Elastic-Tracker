// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from decomp_ros_msgs:msg/Polyhedron.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__TRAITS_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "decomp_ros_msgs/msg/detail/polyhedron__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'points'
// Member 'normals'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace decomp_ros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Polyhedron & msg,
  std::ostream & out)
{
  out << "{";
  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: normals
  {
    if (msg.normals.size() == 0) {
      out << "normals: []";
    } else {
      out << "normals: [";
      size_t pending_items = msg.normals.size();
      for (auto item : msg.normals) {
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
  const Polyhedron & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: normals
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.normals.size() == 0) {
      out << "normals: []\n";
    } else {
      out << "normals:\n";
      for (auto item : msg.normals) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Polyhedron & msg, bool use_flow_style = false)
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
  const decomp_ros_msgs::msg::Polyhedron & msg,
  std::ostream & out, size_t indentation = 0)
{
  decomp_ros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use decomp_ros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const decomp_ros_msgs::msg::Polyhedron & msg)
{
  return decomp_ros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<decomp_ros_msgs::msg::Polyhedron>()
{
  return "decomp_ros_msgs::msg::Polyhedron";
}

template<>
inline const char * name<decomp_ros_msgs::msg::Polyhedron>()
{
  return "decomp_ros_msgs/msg/Polyhedron";
}

template<>
struct has_fixed_size<decomp_ros_msgs::msg::Polyhedron>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<decomp_ros_msgs::msg::Polyhedron>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<decomp_ros_msgs::msg::Polyhedron>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__TRAITS_HPP_
