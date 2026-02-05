// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/OccMap3d.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadrotor_msgs/msg/occ_map3d.hpp"


#ifndef QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/occ_map3d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_OccMap3d_data
{
public:
  explicit Init_OccMap3d_data(::quadrotor_msgs::msg::OccMap3d & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::OccMap3d data(::quadrotor_msgs::msg::OccMap3d::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::OccMap3d msg_;
};

class Init_OccMap3d_offset_z
{
public:
  explicit Init_OccMap3d_offset_z(::quadrotor_msgs::msg::OccMap3d & msg)
  : msg_(msg)
  {}
  Init_OccMap3d_data offset_z(::quadrotor_msgs::msg::OccMap3d::_offset_z_type arg)
  {
    msg_.offset_z = std::move(arg);
    return Init_OccMap3d_data(msg_);
  }

private:
  ::quadrotor_msgs::msg::OccMap3d msg_;
};

class Init_OccMap3d_offset_y
{
public:
  explicit Init_OccMap3d_offset_y(::quadrotor_msgs::msg::OccMap3d & msg)
  : msg_(msg)
  {}
  Init_OccMap3d_offset_z offset_y(::quadrotor_msgs::msg::OccMap3d::_offset_y_type arg)
  {
    msg_.offset_y = std::move(arg);
    return Init_OccMap3d_offset_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::OccMap3d msg_;
};

class Init_OccMap3d_offset_x
{
public:
  explicit Init_OccMap3d_offset_x(::quadrotor_msgs::msg::OccMap3d & msg)
  : msg_(msg)
  {}
  Init_OccMap3d_offset_y offset_x(::quadrotor_msgs::msg::OccMap3d::_offset_x_type arg)
  {
    msg_.offset_x = std::move(arg);
    return Init_OccMap3d_offset_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::OccMap3d msg_;
};

class Init_OccMap3d_size_z
{
public:
  explicit Init_OccMap3d_size_z(::quadrotor_msgs::msg::OccMap3d & msg)
  : msg_(msg)
  {}
  Init_OccMap3d_offset_x size_z(::quadrotor_msgs::msg::OccMap3d::_size_z_type arg)
  {
    msg_.size_z = std::move(arg);
    return Init_OccMap3d_offset_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::OccMap3d msg_;
};

class Init_OccMap3d_size_y
{
public:
  explicit Init_OccMap3d_size_y(::quadrotor_msgs::msg::OccMap3d & msg)
  : msg_(msg)
  {}
  Init_OccMap3d_size_z size_y(::quadrotor_msgs::msg::OccMap3d::_size_y_type arg)
  {
    msg_.size_y = std::move(arg);
    return Init_OccMap3d_size_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::OccMap3d msg_;
};

class Init_OccMap3d_size_x
{
public:
  explicit Init_OccMap3d_size_x(::quadrotor_msgs::msg::OccMap3d & msg)
  : msg_(msg)
  {}
  Init_OccMap3d_size_y size_x(::quadrotor_msgs::msg::OccMap3d::_size_x_type arg)
  {
    msg_.size_x = std::move(arg);
    return Init_OccMap3d_size_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::OccMap3d msg_;
};

class Init_OccMap3d_resolution
{
public:
  explicit Init_OccMap3d_resolution(::quadrotor_msgs::msg::OccMap3d & msg)
  : msg_(msg)
  {}
  Init_OccMap3d_size_x resolution(::quadrotor_msgs::msg::OccMap3d::_resolution_type arg)
  {
    msg_.resolution = std::move(arg);
    return Init_OccMap3d_size_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::OccMap3d msg_;
};

class Init_OccMap3d_header
{
public:
  Init_OccMap3d_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OccMap3d_resolution header(::quadrotor_msgs::msg::OccMap3d::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OccMap3d_resolution(msg_);
  }

private:
  ::quadrotor_msgs::msg::OccMap3d msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::OccMap3d>()
{
  return quadrotor_msgs::msg::builder::Init_OccMap3d_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__BUILDER_HPP_
