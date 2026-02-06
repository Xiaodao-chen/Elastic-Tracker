// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/ReplanState.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/replan_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_ReplanState_replan_stamp
{
public:
  explicit Init_ReplanState_replan_stamp(::quadrotor_msgs::msg::ReplanState & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::ReplanState replan_stamp(::quadrotor_msgs::msg::ReplanState::_replan_stamp_type arg)
  {
    msg_.replan_stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::ReplanState msg_;
};

class Init_ReplanState_path
{
public:
  explicit Init_ReplanState_path(::quadrotor_msgs::msg::ReplanState & msg)
  : msg_(msg)
  {}
  Init_ReplanState_replan_stamp path(::quadrotor_msgs::msg::ReplanState::_path_type arg)
  {
    msg_.path = std::move(arg);
    return Init_ReplanState_replan_stamp(msg_);
  }

private:
  ::quadrotor_msgs::msg::ReplanState msg_;
};

class Init_ReplanState_occmap
{
public:
  explicit Init_ReplanState_occmap(::quadrotor_msgs::msg::ReplanState & msg)
  : msg_(msg)
  {}
  Init_ReplanState_path occmap(::quadrotor_msgs::msg::ReplanState::_occmap_type arg)
  {
    msg_.occmap = std::move(arg);
    return Init_ReplanState_path(msg_);
  }

private:
  ::quadrotor_msgs::msg::ReplanState msg_;
};

class Init_ReplanState_target
{
public:
  explicit Init_ReplanState_target(::quadrotor_msgs::msg::ReplanState & msg)
  : msg_(msg)
  {}
  Init_ReplanState_occmap target(::quadrotor_msgs::msg::ReplanState::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_ReplanState_occmap(msg_);
  }

private:
  ::quadrotor_msgs::msg::ReplanState msg_;
};

class Init_ReplanState_ini_state
{
public:
  explicit Init_ReplanState_ini_state(::quadrotor_msgs::msg::ReplanState & msg)
  : msg_(msg)
  {}
  Init_ReplanState_target ini_state(::quadrotor_msgs::msg::ReplanState::_ini_state_type arg)
  {
    msg_.ini_state = std::move(arg);
    return Init_ReplanState_target(msg_);
  }

private:
  ::quadrotor_msgs::msg::ReplanState msg_;
};

class Init_ReplanState_state
{
public:
  explicit Init_ReplanState_state(::quadrotor_msgs::msg::ReplanState & msg)
  : msg_(msg)
  {}
  Init_ReplanState_ini_state state(::quadrotor_msgs::msg::ReplanState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_ReplanState_ini_state(msg_);
  }

private:
  ::quadrotor_msgs::msg::ReplanState msg_;
};

class Init_ReplanState_header
{
public:
  Init_ReplanState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReplanState_state header(::quadrotor_msgs::msg::ReplanState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ReplanState_state(msg_);
  }

private:
  ::quadrotor_msgs::msg::ReplanState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::ReplanState>()
{
  return quadrotor_msgs::msg::builder::Init_ReplanState_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__BUILDER_HPP_
