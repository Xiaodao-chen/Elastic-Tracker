// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/TakeoffLand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/takeoff_land__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_TakeoffLand_takeoff_land_cmd
{
public:
  Init_TakeoffLand_takeoff_land_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::quadrotor_msgs::msg::TakeoffLand takeoff_land_cmd(::quadrotor_msgs::msg::TakeoffLand::_takeoff_land_cmd_type arg)
  {
    msg_.takeoff_land_cmd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::TakeoffLand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::TakeoffLand>()
{
  return quadrotor_msgs::msg::builder::Init_TakeoffLand_takeoff_land_cmd();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__BUILDER_HPP_
