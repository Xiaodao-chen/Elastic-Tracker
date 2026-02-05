// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from decomp_ros_msgs:msg/PolyhedronArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "decomp_ros_msgs/msg/polyhedron_array.hpp"


#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__BUILDER_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "decomp_ros_msgs/msg/detail/polyhedron_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace decomp_ros_msgs
{

namespace msg
{

namespace builder
{

class Init_PolyhedronArray_polyhedrons
{
public:
  explicit Init_PolyhedronArray_polyhedrons(::decomp_ros_msgs::msg::PolyhedronArray & msg)
  : msg_(msg)
  {}
  ::decomp_ros_msgs::msg::PolyhedronArray polyhedrons(::decomp_ros_msgs::msg::PolyhedronArray::_polyhedrons_type arg)
  {
    msg_.polyhedrons = std::move(arg);
    return std::move(msg_);
  }

private:
  ::decomp_ros_msgs::msg::PolyhedronArray msg_;
};

class Init_PolyhedronArray_header
{
public:
  Init_PolyhedronArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolyhedronArray_polyhedrons header(::decomp_ros_msgs::msg::PolyhedronArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PolyhedronArray_polyhedrons(msg_);
  }

private:
  ::decomp_ros_msgs::msg::PolyhedronArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::decomp_ros_msgs::msg::PolyhedronArray>()
{
  return decomp_ros_msgs::msg::builder::Init_PolyhedronArray_header();
}

}  // namespace decomp_ros_msgs

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__BUILDER_HPP_
