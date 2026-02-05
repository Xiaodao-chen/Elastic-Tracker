// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/OccMap3d.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadrotor_msgs/msg/occ_map3d.hpp"


#ifndef QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/occ_map3d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OccMap3d & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: resolution
  {
    out << "resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution, out);
    out << ", ";
  }

  // member: size_x
  {
    out << "size_x: ";
    rosidl_generator_traits::value_to_yaml(msg.size_x, out);
    out << ", ";
  }

  // member: size_y
  {
    out << "size_y: ";
    rosidl_generator_traits::value_to_yaml(msg.size_y, out);
    out << ", ";
  }

  // member: size_z
  {
    out << "size_z: ";
    rosidl_generator_traits::value_to_yaml(msg.size_z, out);
    out << ", ";
  }

  // member: offset_x
  {
    out << "offset_x: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_x, out);
    out << ", ";
  }

  // member: offset_y
  {
    out << "offset_y: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_y, out);
    out << ", ";
  }

  // member: offset_z
  {
    out << "offset_z: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_z, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
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
  const OccMap3d & msg,
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

  // member: resolution
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution, out);
    out << "\n";
  }

  // member: size_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size_x: ";
    rosidl_generator_traits::value_to_yaml(msg.size_x, out);
    out << "\n";
  }

  // member: size_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size_y: ";
    rosidl_generator_traits::value_to_yaml(msg.size_y, out);
    out << "\n";
  }

  // member: size_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size_z: ";
    rosidl_generator_traits::value_to_yaml(msg.size_z, out);
    out << "\n";
  }

  // member: offset_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offset_x: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_x, out);
    out << "\n";
  }

  // member: offset_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offset_y: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_y, out);
    out << "\n";
  }

  // member: offset_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offset_z: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_z, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
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

inline std::string to_yaml(const OccMap3d & msg, bool use_flow_style = false)
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

}  // namespace quadrotor_msgs

namespace rosidl_generator_traits
{

[[deprecated("use quadrotor_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const quadrotor_msgs::msg::OccMap3d & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::OccMap3d & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::OccMap3d>()
{
  return "quadrotor_msgs::msg::OccMap3d";
}

template<>
inline const char * name<quadrotor_msgs::msg::OccMap3d>()
{
  return "quadrotor_msgs/msg/OccMap3d";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::OccMap3d>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::OccMap3d>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quadrotor_msgs::msg::OccMap3d>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__TRAITS_HPP_
