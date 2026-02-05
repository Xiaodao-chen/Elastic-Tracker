// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/Px4ctrlDebug.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadrotor_msgs/msg/px4ctrl_debug.hpp"


#ifndef QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/px4ctrl_debug__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_Px4ctrlDebug_fb_rate_z
{
public:
  explicit Init_Px4ctrlDebug_fb_rate_z(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::Px4ctrlDebug fb_rate_z(::quadrotor_msgs::msg::Px4ctrlDebug::_fb_rate_z_type arg)
  {
    msg_.fb_rate_z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_fb_rate_y
{
public:
  explicit Init_Px4ctrlDebug_fb_rate_y(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_fb_rate_z fb_rate_y(::quadrotor_msgs::msg::Px4ctrlDebug::_fb_rate_y_type arg)
  {
    msg_.fb_rate_y = std::move(arg);
    return Init_Px4ctrlDebug_fb_rate_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_fb_rate_x
{
public:
  explicit Init_Px4ctrlDebug_fb_rate_x(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_fb_rate_y fb_rate_x(::quadrotor_msgs::msg::Px4ctrlDebug::_fb_rate_x_type arg)
  {
    msg_.fb_rate_x = std::move(arg);
    return Init_Px4ctrlDebug_fb_rate_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_err_axisang_ang
{
public:
  explicit Init_Px4ctrlDebug_err_axisang_ang(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_fb_rate_x err_axisang_ang(::quadrotor_msgs::msg::Px4ctrlDebug::_err_axisang_ang_type arg)
  {
    msg_.err_axisang_ang = std::move(arg);
    return Init_Px4ctrlDebug_fb_rate_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_err_axisang_z
{
public:
  explicit Init_Px4ctrlDebug_err_axisang_z(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_err_axisang_ang err_axisang_z(::quadrotor_msgs::msg::Px4ctrlDebug::_err_axisang_z_type arg)
  {
    msg_.err_axisang_z = std::move(arg);
    return Init_Px4ctrlDebug_err_axisang_ang(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_err_axisang_y
{
public:
  explicit Init_Px4ctrlDebug_err_axisang_y(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_err_axisang_z err_axisang_y(::quadrotor_msgs::msg::Px4ctrlDebug::_err_axisang_y_type arg)
  {
    msg_.err_axisang_y = std::move(arg);
    return Init_Px4ctrlDebug_err_axisang_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_err_axisang_x
{
public:
  explicit Init_Px4ctrlDebug_err_axisang_x(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_err_axisang_y err_axisang_x(::quadrotor_msgs::msg::Px4ctrlDebug::_err_axisang_x_type arg)
  {
    msg_.err_axisang_x = std::move(arg);
    return Init_Px4ctrlDebug_err_axisang_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_voltage
{
public:
  explicit Init_Px4ctrlDebug_voltage(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_err_axisang_x voltage(::quadrotor_msgs::msg::Px4ctrlDebug::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_Px4ctrlDebug_err_axisang_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_thr_scale_compensate
{
public:
  explicit Init_Px4ctrlDebug_thr_scale_compensate(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_voltage thr_scale_compensate(::quadrotor_msgs::msg::Px4ctrlDebug::_thr_scale_compensate_type arg)
  {
    msg_.thr_scale_compensate = std::move(arg);
    return Init_Px4ctrlDebug_voltage(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_thr2acc
{
public:
  explicit Init_Px4ctrlDebug_thr2acc(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_thr_scale_compensate thr2acc(::quadrotor_msgs::msg::Px4ctrlDebug::_thr2acc_type arg)
  {
    msg_.thr2acc = std::move(arg);
    return Init_Px4ctrlDebug_thr_scale_compensate(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_thr
{
public:
  explicit Init_Px4ctrlDebug_des_thr(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_thr2acc des_thr(::quadrotor_msgs::msg::Px4ctrlDebug::_des_thr_type arg)
  {
    msg_.des_thr = std::move(arg);
    return Init_Px4ctrlDebug_thr2acc(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_q_w
{
public:
  explicit Init_Px4ctrlDebug_des_q_w(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_des_thr des_q_w(::quadrotor_msgs::msg::Px4ctrlDebug::_des_q_w_type arg)
  {
    msg_.des_q_w = std::move(arg);
    return Init_Px4ctrlDebug_des_thr(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_q_z
{
public:
  explicit Init_Px4ctrlDebug_des_q_z(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_des_q_w des_q_z(::quadrotor_msgs::msg::Px4ctrlDebug::_des_q_z_type arg)
  {
    msg_.des_q_z = std::move(arg);
    return Init_Px4ctrlDebug_des_q_w(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_q_y
{
public:
  explicit Init_Px4ctrlDebug_des_q_y(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_des_q_z des_q_y(::quadrotor_msgs::msg::Px4ctrlDebug::_des_q_y_type arg)
  {
    msg_.des_q_y = std::move(arg);
    return Init_Px4ctrlDebug_des_q_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_q_x
{
public:
  explicit Init_Px4ctrlDebug_des_q_x(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_des_q_y des_q_x(::quadrotor_msgs::msg::Px4ctrlDebug::_des_q_x_type arg)
  {
    msg_.des_q_x = std::move(arg);
    return Init_Px4ctrlDebug_des_q_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_a_z
{
public:
  explicit Init_Px4ctrlDebug_des_a_z(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_des_q_x des_a_z(::quadrotor_msgs::msg::Px4ctrlDebug::_des_a_z_type arg)
  {
    msg_.des_a_z = std::move(arg);
    return Init_Px4ctrlDebug_des_q_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_a_y
{
public:
  explicit Init_Px4ctrlDebug_des_a_y(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_des_a_z des_a_y(::quadrotor_msgs::msg::Px4ctrlDebug::_des_a_y_type arg)
  {
    msg_.des_a_y = std::move(arg);
    return Init_Px4ctrlDebug_des_a_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_a_x
{
public:
  explicit Init_Px4ctrlDebug_des_a_x(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_des_a_y des_a_x(::quadrotor_msgs::msg::Px4ctrlDebug::_des_a_x_type arg)
  {
    msg_.des_a_x = std::move(arg);
    return Init_Px4ctrlDebug_des_a_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_fb_a_z
{
public:
  explicit Init_Px4ctrlDebug_fb_a_z(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_des_a_x fb_a_z(::quadrotor_msgs::msg::Px4ctrlDebug::_fb_a_z_type arg)
  {
    msg_.fb_a_z = std::move(arg);
    return Init_Px4ctrlDebug_des_a_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_fb_a_y
{
public:
  explicit Init_Px4ctrlDebug_fb_a_y(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_fb_a_z fb_a_y(::quadrotor_msgs::msg::Px4ctrlDebug::_fb_a_y_type arg)
  {
    msg_.fb_a_y = std::move(arg);
    return Init_Px4ctrlDebug_fb_a_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_fb_a_x
{
public:
  explicit Init_Px4ctrlDebug_fb_a_x(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_fb_a_y fb_a_x(::quadrotor_msgs::msg::Px4ctrlDebug::_fb_a_x_type arg)
  {
    msg_.fb_a_x = std::move(arg);
    return Init_Px4ctrlDebug_fb_a_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_v_z
{
public:
  explicit Init_Px4ctrlDebug_des_v_z(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_fb_a_x des_v_z(::quadrotor_msgs::msg::Px4ctrlDebug::_des_v_z_type arg)
  {
    msg_.des_v_z = std::move(arg);
    return Init_Px4ctrlDebug_fb_a_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_v_y
{
public:
  explicit Init_Px4ctrlDebug_des_v_y(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_des_v_z des_v_y(::quadrotor_msgs::msg::Px4ctrlDebug::_des_v_y_type arg)
  {
    msg_.des_v_y = std::move(arg);
    return Init_Px4ctrlDebug_des_v_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_des_v_x
{
public:
  explicit Init_Px4ctrlDebug_des_v_x(::quadrotor_msgs::msg::Px4ctrlDebug & msg)
  : msg_(msg)
  {}
  Init_Px4ctrlDebug_des_v_y des_v_x(::quadrotor_msgs::msg::Px4ctrlDebug::_des_v_x_type arg)
  {
    msg_.des_v_x = std::move(arg);
    return Init_Px4ctrlDebug_des_v_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

class Init_Px4ctrlDebug_header
{
public:
  Init_Px4ctrlDebug_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Px4ctrlDebug_des_v_x header(::quadrotor_msgs::msg::Px4ctrlDebug::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Px4ctrlDebug_des_v_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::Px4ctrlDebug msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::Px4ctrlDebug>()
{
  return quadrotor_msgs::msg::builder::Init_Px4ctrlDebug_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__BUILDER_HPP_
