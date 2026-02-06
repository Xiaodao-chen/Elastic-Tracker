// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from object_detection_msgs:msg/MotorAngle.idl
// generated code does not contain a copyright notice

#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__MOTOR_ANGLE__BUILDER_HPP_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__MOTOR_ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "object_detection_msgs/msg/detail/motor_angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace object_detection_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorAngle_angle
{
public:
  explicit Init_MotorAngle_angle(::object_detection_msgs::msg::MotorAngle & msg)
  : msg_(msg)
  {}
  ::object_detection_msgs::msg::MotorAngle angle(::object_detection_msgs::msg::MotorAngle::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::object_detection_msgs::msg::MotorAngle msg_;
};

class Init_MotorAngle_header
{
public:
  Init_MotorAngle_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorAngle_angle header(::object_detection_msgs::msg::MotorAngle::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorAngle_angle(msg_);
  }

private:
  ::object_detection_msgs::msg::MotorAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::object_detection_msgs::msg::MotorAngle>()
{
  return object_detection_msgs::msg::builder::Init_MotorAngle_header();
}

}  // namespace object_detection_msgs

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__MOTOR_ANGLE__BUILDER_HPP_
