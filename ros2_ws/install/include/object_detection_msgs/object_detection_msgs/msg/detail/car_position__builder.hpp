// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from object_detection_msgs:msg/CarPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "object_detection_msgs/msg/car_position.hpp"


#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__CAR_POSITION__BUILDER_HPP_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__CAR_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "object_detection_msgs/msg/detail/car_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace object_detection_msgs
{

namespace msg
{

namespace builder
{

class Init_CarPosition_zmean
{
public:
  explicit Init_CarPosition_zmean(::object_detection_msgs::msg::CarPosition & msg)
  : msg_(msg)
  {}
  ::object_detection_msgs::msg::CarPosition zmean(::object_detection_msgs::msg::CarPosition::_zmean_type arg)
  {
    msg_.zmean = std::move(arg);
    return std::move(msg_);
  }

private:
  ::object_detection_msgs::msg::CarPosition msg_;
};

class Init_CarPosition_ymean
{
public:
  explicit Init_CarPosition_ymean(::object_detection_msgs::msg::CarPosition & msg)
  : msg_(msg)
  {}
  Init_CarPosition_zmean ymean(::object_detection_msgs::msg::CarPosition::_ymean_type arg)
  {
    msg_.ymean = std::move(arg);
    return Init_CarPosition_zmean(msg_);
  }

private:
  ::object_detection_msgs::msg::CarPosition msg_;
};

class Init_CarPosition_xmean
{
public:
  explicit Init_CarPosition_xmean(::object_detection_msgs::msg::CarPosition & msg)
  : msg_(msg)
  {}
  Init_CarPosition_ymean xmean(::object_detection_msgs::msg::CarPosition::_xmean_type arg)
  {
    msg_.xmean = std::move(arg);
    return Init_CarPosition_ymean(msg_);
  }

private:
  ::object_detection_msgs::msg::CarPosition msg_;
};

class Init_CarPosition_header
{
public:
  Init_CarPosition_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarPosition_xmean header(::object_detection_msgs::msg::CarPosition::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CarPosition_xmean(msg_);
  }

private:
  ::object_detection_msgs::msg::CarPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::object_detection_msgs::msg::CarPosition>()
{
  return object_detection_msgs::msg::builder::Init_CarPosition_header();
}

}  // namespace object_detection_msgs

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__CAR_POSITION__BUILDER_HPP_
