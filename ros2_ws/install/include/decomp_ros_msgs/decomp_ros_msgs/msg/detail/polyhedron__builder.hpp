// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from decomp_ros_msgs:msg/Polyhedron.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "decomp_ros_msgs/msg/polyhedron.hpp"


#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__BUILDER_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "decomp_ros_msgs/msg/detail/polyhedron__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace decomp_ros_msgs
{

namespace msg
{

namespace builder
{

class Init_Polyhedron_normals
{
public:
  explicit Init_Polyhedron_normals(::decomp_ros_msgs::msg::Polyhedron & msg)
  : msg_(msg)
  {}
  ::decomp_ros_msgs::msg::Polyhedron normals(::decomp_ros_msgs::msg::Polyhedron::_normals_type arg)
  {
    msg_.normals = std::move(arg);
    return std::move(msg_);
  }

private:
  ::decomp_ros_msgs::msg::Polyhedron msg_;
};

class Init_Polyhedron_points
{
public:
  Init_Polyhedron_points()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Polyhedron_normals points(::decomp_ros_msgs::msg::Polyhedron::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_Polyhedron_normals(msg_);
  }

private:
  ::decomp_ros_msgs::msg::Polyhedron msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::decomp_ros_msgs::msg::Polyhedron>()
{
  return decomp_ros_msgs::msg::builder::Init_Polyhedron_points();
}

}  // namespace decomp_ros_msgs

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__BUILDER_HPP_
