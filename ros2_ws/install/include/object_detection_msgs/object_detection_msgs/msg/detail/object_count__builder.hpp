// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from object_detection_msgs:msg/ObjectCount.idl
// generated code does not contain a copyright notice

#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT_COUNT__BUILDER_HPP_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT_COUNT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "object_detection_msgs/msg/detail/object_count__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace object_detection_msgs
{

namespace msg
{

namespace builder
{

class Init_ObjectCount_count
{
public:
  explicit Init_ObjectCount_count(::object_detection_msgs::msg::ObjectCount & msg)
  : msg_(msg)
  {}
  ::object_detection_msgs::msg::ObjectCount count(::object_detection_msgs::msg::ObjectCount::_count_type arg)
  {
    msg_.count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::object_detection_msgs::msg::ObjectCount msg_;
};

class Init_ObjectCount_header
{
public:
  Init_ObjectCount_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectCount_count header(::object_detection_msgs::msg::ObjectCount::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectCount_count(msg_);
  }

private:
  ::object_detection_msgs::msg::ObjectCount msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::object_detection_msgs::msg::ObjectCount>()
{
  return object_detection_msgs::msg::builder::Init_ObjectCount_header();
}

}  // namespace object_detection_msgs

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT_COUNT__BUILDER_HPP_
