// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from decomp_ros_msgs:msg/Ellipsoid.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "decomp_ros_msgs/msg/ellipsoid.hpp"


#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__BUILDER_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "decomp_ros_msgs/msg/detail/ellipsoid__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace decomp_ros_msgs
{

namespace msg
{

namespace builder
{

class Init_Ellipsoid_e
{
public:
  explicit Init_Ellipsoid_e(::decomp_ros_msgs::msg::Ellipsoid & msg)
  : msg_(msg)
  {}
  ::decomp_ros_msgs::msg::Ellipsoid e(::decomp_ros_msgs::msg::Ellipsoid::_e_type arg)
  {
    msg_.e = std::move(arg);
    return std::move(msg_);
  }

private:
  ::decomp_ros_msgs::msg::Ellipsoid msg_;
};

class Init_Ellipsoid_d
{
public:
  Init_Ellipsoid_d()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Ellipsoid_e d(::decomp_ros_msgs::msg::Ellipsoid::_d_type arg)
  {
    msg_.d = std::move(arg);
    return Init_Ellipsoid_e(msg_);
  }

private:
  ::decomp_ros_msgs::msg::Ellipsoid msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::decomp_ros_msgs::msg::Ellipsoid>()
{
  return decomp_ros_msgs::msg::builder::Init_Ellipsoid_d();
}

}  // namespace decomp_ros_msgs

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__BUILDER_HPP_
