// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/TakeoffLand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/takeoff_land__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TakeoffLand & msg,
  std::ostream & out)
{
  out << "{";
  // member: takeoff_land_cmd
  {
    out << "takeoff_land_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.takeoff_land_cmd, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TakeoffLand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: takeoff_land_cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "takeoff_land_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.takeoff_land_cmd, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TakeoffLand & msg, bool use_flow_style = false)
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
  const quadrotor_msgs::msg::TakeoffLand & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::TakeoffLand & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::TakeoffLand>()
{
  return "quadrotor_msgs::msg::TakeoffLand";
}

template<>
inline const char * name<quadrotor_msgs::msg::TakeoffLand>()
{
  return "quadrotor_msgs/msg/TakeoffLand";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::TakeoffLand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::TakeoffLand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<quadrotor_msgs::msg::TakeoffLand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__TRAITS_HPP_
