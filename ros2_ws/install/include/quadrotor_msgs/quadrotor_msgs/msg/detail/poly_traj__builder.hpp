// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/PolyTraj.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadrotor_msgs/msg/poly_traj.hpp"


#ifndef QUADROTOR_MSGS__MSG__DETAIL__POLY_TRAJ__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__POLY_TRAJ__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/poly_traj__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_PolyTraj_duration
{
public:
  explicit Init_PolyTraj_duration(::quadrotor_msgs::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::PolyTraj duration(::quadrotor_msgs::msg::PolyTraj::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

class Init_PolyTraj_coef_z
{
public:
  explicit Init_PolyTraj_coef_z(::quadrotor_msgs::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_duration coef_z(::quadrotor_msgs::msg::PolyTraj::_coef_z_type arg)
  {
    msg_.coef_z = std::move(arg);
    return Init_PolyTraj_duration(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

class Init_PolyTraj_coef_y
{
public:
  explicit Init_PolyTraj_coef_y(::quadrotor_msgs::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_coef_z coef_y(::quadrotor_msgs::msg::PolyTraj::_coef_y_type arg)
  {
    msg_.coef_y = std::move(arg);
    return Init_PolyTraj_coef_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

class Init_PolyTraj_coef_x
{
public:
  explicit Init_PolyTraj_coef_x(::quadrotor_msgs::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_coef_y coef_x(::quadrotor_msgs::msg::PolyTraj::_coef_x_type arg)
  {
    msg_.coef_x = std::move(arg);
    return Init_PolyTraj_coef_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

class Init_PolyTraj_order
{
public:
  explicit Init_PolyTraj_order(::quadrotor_msgs::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_coef_x order(::quadrotor_msgs::msg::PolyTraj::_order_type arg)
  {
    msg_.order = std::move(arg);
    return Init_PolyTraj_coef_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

class Init_PolyTraj_hover_p
{
public:
  explicit Init_PolyTraj_hover_p(::quadrotor_msgs::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_order hover_p(::quadrotor_msgs::msg::PolyTraj::_hover_p_type arg)
  {
    msg_.hover_p = std::move(arg);
    return Init_PolyTraj_order(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

class Init_PolyTraj_yaw
{
public:
  explicit Init_PolyTraj_yaw(::quadrotor_msgs::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_hover_p yaw(::quadrotor_msgs::msg::PolyTraj::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PolyTraj_hover_p(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

class Init_PolyTraj_hover
{
public:
  explicit Init_PolyTraj_hover(::quadrotor_msgs::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_yaw hover(::quadrotor_msgs::msg::PolyTraj::_hover_type arg)
  {
    msg_.hover = std::move(arg);
    return Init_PolyTraj_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

class Init_PolyTraj_start_time
{
public:
  explicit Init_PolyTraj_start_time(::quadrotor_msgs::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_hover start_time(::quadrotor_msgs::msg::PolyTraj::_start_time_type arg)
  {
    msg_.start_time = std::move(arg);
    return Init_PolyTraj_hover(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

class Init_PolyTraj_traj_id
{
public:
  explicit Init_PolyTraj_traj_id(::quadrotor_msgs::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_start_time traj_id(::quadrotor_msgs::msg::PolyTraj::_traj_id_type arg)
  {
    msg_.traj_id = std::move(arg);
    return Init_PolyTraj_start_time(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

class Init_PolyTraj_drone_id
{
public:
  Init_PolyTraj_drone_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolyTraj_traj_id drone_id(::quadrotor_msgs::msg::PolyTraj::_drone_id_type arg)
  {
    msg_.drone_id = std::move(arg);
    return Init_PolyTraj_traj_id(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolyTraj msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::PolyTraj>()
{
  return quadrotor_msgs::msg::builder::Init_PolyTraj_drone_id();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__POLY_TRAJ__BUILDER_HPP_
