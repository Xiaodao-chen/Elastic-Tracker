// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from decomp_ros_msgs:msg/EllipsoidArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "decomp_ros_msgs/msg/ellipsoid_array.hpp"


#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__BUILDER_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "decomp_ros_msgs/msg/detail/ellipsoid_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace decomp_ros_msgs
{

namespace msg
{

namespace builder
{

class Init_EllipsoidArray_ellipsoids
{
public:
  explicit Init_EllipsoidArray_ellipsoids(::decomp_ros_msgs::msg::EllipsoidArray & msg)
  : msg_(msg)
  {}
  ::decomp_ros_msgs::msg::EllipsoidArray ellipsoids(::decomp_ros_msgs::msg::EllipsoidArray::_ellipsoids_type arg)
  {
    msg_.ellipsoids = std::move(arg);
    return std::move(msg_);
  }

private:
  ::decomp_ros_msgs::msg::EllipsoidArray msg_;
};

class Init_EllipsoidArray_header
{
public:
  Init_EllipsoidArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EllipsoidArray_ellipsoids header(::decomp_ros_msgs::msg::EllipsoidArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EllipsoidArray_ellipsoids(msg_);
  }

private:
  ::decomp_ros_msgs::msg::EllipsoidArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::decomp_ros_msgs::msg::EllipsoidArray>()
{
  return decomp_ros_msgs::msg::builder::Init_EllipsoidArray_header();
}

}  // namespace decomp_ros_msgs

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__BUILDER_HPP_
