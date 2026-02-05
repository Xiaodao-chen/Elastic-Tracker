// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/Px4ctrlDebug.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/px4ctrl_debug__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Px4ctrlDebug & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: des_v_x
  {
    out << "des_v_x: ";
    rosidl_generator_traits::value_to_yaml(msg.des_v_x, out);
    out << ", ";
  }

  // member: des_v_y
  {
    out << "des_v_y: ";
    rosidl_generator_traits::value_to_yaml(msg.des_v_y, out);
    out << ", ";
  }

  // member: des_v_z
  {
    out << "des_v_z: ";
    rosidl_generator_traits::value_to_yaml(msg.des_v_z, out);
    out << ", ";
  }

  // member: fb_a_x
  {
    out << "fb_a_x: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_a_x, out);
    out << ", ";
  }

  // member: fb_a_y
  {
    out << "fb_a_y: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_a_y, out);
    out << ", ";
  }

  // member: fb_a_z
  {
    out << "fb_a_z: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_a_z, out);
    out << ", ";
  }

  // member: des_a_x
  {
    out << "des_a_x: ";
    rosidl_generator_traits::value_to_yaml(msg.des_a_x, out);
    out << ", ";
  }

  // member: des_a_y
  {
    out << "des_a_y: ";
    rosidl_generator_traits::value_to_yaml(msg.des_a_y, out);
    out << ", ";
  }

  // member: des_a_z
  {
    out << "des_a_z: ";
    rosidl_generator_traits::value_to_yaml(msg.des_a_z, out);
    out << ", ";
  }

  // member: des_q_x
  {
    out << "des_q_x: ";
    rosidl_generator_traits::value_to_yaml(msg.des_q_x, out);
    out << ", ";
  }

  // member: des_q_y
  {
    out << "des_q_y: ";
    rosidl_generator_traits::value_to_yaml(msg.des_q_y, out);
    out << ", ";
  }

  // member: des_q_z
  {
    out << "des_q_z: ";
    rosidl_generator_traits::value_to_yaml(msg.des_q_z, out);
    out << ", ";
  }

  // member: des_q_w
  {
    out << "des_q_w: ";
    rosidl_generator_traits::value_to_yaml(msg.des_q_w, out);
    out << ", ";
  }

  // member: des_thr
  {
    out << "des_thr: ";
    rosidl_generator_traits::value_to_yaml(msg.des_thr, out);
    out << ", ";
  }

  // member: thr2acc
  {
    out << "thr2acc: ";
    rosidl_generator_traits::value_to_yaml(msg.thr2acc, out);
    out << ", ";
  }

  // member: thr_scale_compensate
  {
    out << "thr_scale_compensate: ";
    rosidl_generator_traits::value_to_yaml(msg.thr_scale_compensate, out);
    out << ", ";
  }

  // member: voltage
  {
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << ", ";
  }

  // member: err_axisang_x
  {
    out << "err_axisang_x: ";
    rosidl_generator_traits::value_to_yaml(msg.err_axisang_x, out);
    out << ", ";
  }

  // member: err_axisang_y
  {
    out << "err_axisang_y: ";
    rosidl_generator_traits::value_to_yaml(msg.err_axisang_y, out);
    out << ", ";
  }

  // member: err_axisang_z
  {
    out << "err_axisang_z: ";
    rosidl_generator_traits::value_to_yaml(msg.err_axisang_z, out);
    out << ", ";
  }

  // member: err_axisang_ang
  {
    out << "err_axisang_ang: ";
    rosidl_generator_traits::value_to_yaml(msg.err_axisang_ang, out);
    out << ", ";
  }

  // member: fb_rate_x
  {
    out << "fb_rate_x: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_rate_x, out);
    out << ", ";
  }

  // member: fb_rate_y
  {
    out << "fb_rate_y: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_rate_y, out);
    out << ", ";
  }

  // member: fb_rate_z
  {
    out << "fb_rate_z: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_rate_z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Px4ctrlDebug & msg,
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

  // member: des_v_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_v_x: ";
    rosidl_generator_traits::value_to_yaml(msg.des_v_x, out);
    out << "\n";
  }

  // member: des_v_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_v_y: ";
    rosidl_generator_traits::value_to_yaml(msg.des_v_y, out);
    out << "\n";
  }

  // member: des_v_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_v_z: ";
    rosidl_generator_traits::value_to_yaml(msg.des_v_z, out);
    out << "\n";
  }

  // member: fb_a_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fb_a_x: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_a_x, out);
    out << "\n";
  }

  // member: fb_a_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fb_a_y: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_a_y, out);
    out << "\n";
  }

  // member: fb_a_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fb_a_z: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_a_z, out);
    out << "\n";
  }

  // member: des_a_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_a_x: ";
    rosidl_generator_traits::value_to_yaml(msg.des_a_x, out);
    out << "\n";
  }

  // member: des_a_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_a_y: ";
    rosidl_generator_traits::value_to_yaml(msg.des_a_y, out);
    out << "\n";
  }

  // member: des_a_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_a_z: ";
    rosidl_generator_traits::value_to_yaml(msg.des_a_z, out);
    out << "\n";
  }

  // member: des_q_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_q_x: ";
    rosidl_generator_traits::value_to_yaml(msg.des_q_x, out);
    out << "\n";
  }

  // member: des_q_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_q_y: ";
    rosidl_generator_traits::value_to_yaml(msg.des_q_y, out);
    out << "\n";
  }

  // member: des_q_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_q_z: ";
    rosidl_generator_traits::value_to_yaml(msg.des_q_z, out);
    out << "\n";
  }

  // member: des_q_w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_q_w: ";
    rosidl_generator_traits::value_to_yaml(msg.des_q_w, out);
    out << "\n";
  }

  // member: des_thr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_thr: ";
    rosidl_generator_traits::value_to_yaml(msg.des_thr, out);
    out << "\n";
  }

  // member: thr2acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thr2acc: ";
    rosidl_generator_traits::value_to_yaml(msg.thr2acc, out);
    out << "\n";
  }

  // member: thr_scale_compensate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thr_scale_compensate: ";
    rosidl_generator_traits::value_to_yaml(msg.thr_scale_compensate, out);
    out << "\n";
  }

  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << "\n";
  }

  // member: err_axisang_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "err_axisang_x: ";
    rosidl_generator_traits::value_to_yaml(msg.err_axisang_x, out);
    out << "\n";
  }

  // member: err_axisang_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "err_axisang_y: ";
    rosidl_generator_traits::value_to_yaml(msg.err_axisang_y, out);
    out << "\n";
  }

  // member: err_axisang_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "err_axisang_z: ";
    rosidl_generator_traits::value_to_yaml(msg.err_axisang_z, out);
    out << "\n";
  }

  // member: err_axisang_ang
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "err_axisang_ang: ";
    rosidl_generator_traits::value_to_yaml(msg.err_axisang_ang, out);
    out << "\n";
  }

  // member: fb_rate_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fb_rate_x: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_rate_x, out);
    out << "\n";
  }

  // member: fb_rate_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fb_rate_y: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_rate_y, out);
    out << "\n";
  }

  // member: fb_rate_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fb_rate_z: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_rate_z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Px4ctrlDebug & msg, bool use_flow_style = false)
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
  const quadrotor_msgs::msg::Px4ctrlDebug & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::Px4ctrlDebug & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::Px4ctrlDebug>()
{
  return "quadrotor_msgs::msg::Px4ctrlDebug";
}

template<>
inline const char * name<quadrotor_msgs::msg::Px4ctrlDebug>()
{
  return "quadrotor_msgs/msg/Px4ctrlDebug";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::Px4ctrlDebug>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::Px4ctrlDebug>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<quadrotor_msgs::msg::Px4ctrlDebug>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__TRAITS_HPP_
