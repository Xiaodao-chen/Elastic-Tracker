// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/ReplanState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadrotor_msgs/msg/replan_state.hpp"


#ifndef QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/replan_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'target'
#include "nav_msgs/msg/detail/odometry__traits.hpp"
// Member 'occmap'
#include "quadrotor_msgs/msg/detail/occ_map3d__traits.hpp"
// Member 'replan_stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ReplanState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: ini_state
  {
    if (msg.ini_state.size() == 0) {
      out << "ini_state: []";
    } else {
      out << "ini_state: [";
      size_t pending_items = msg.ini_state.size();
      for (auto item : msg.ini_state) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: target
  {
    out << "target: ";
    to_flow_style_yaml(msg.target, out);
    out << ", ";
  }

  // member: occmap
  {
    out << "occmap: ";
    to_flow_style_yaml(msg.occmap, out);
    out << ", ";
  }

  // member: path
  {
    if (msg.path.size() == 0) {
      out << "path: []";
    } else {
      out << "path: [";
      size_t pending_items = msg.path.size();
      for (auto item : msg.path) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: replan_stamp
  {
    out << "replan_stamp: ";
    to_flow_style_yaml(msg.replan_stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ReplanState & msg,
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: ini_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ini_state.size() == 0) {
      out << "ini_state: []\n";
    } else {
      out << "ini_state:\n";
      for (auto item : msg.ini_state) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target:\n";
    to_block_style_yaml(msg.target, out, indentation + 2);
  }

  // member: occmap
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "occmap:\n";
    to_block_style_yaml(msg.occmap, out, indentation + 2);
  }

  // member: path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.path.size() == 0) {
      out << "path: []\n";
    } else {
      out << "path:\n";
      for (auto item : msg.path) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: replan_stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "replan_stamp:\n";
    to_block_style_yaml(msg.replan_stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ReplanState & msg, bool use_flow_style = false)
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
  const quadrotor_msgs::msg::ReplanState & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::ReplanState & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::ReplanState>()
{
  return "quadrotor_msgs::msg::ReplanState";
}

template<>
inline const char * name<quadrotor_msgs::msg::ReplanState>()
{
  return "quadrotor_msgs/msg/ReplanState";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::ReplanState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::ReplanState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quadrotor_msgs::msg::ReplanState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__TRAITS_HPP_
