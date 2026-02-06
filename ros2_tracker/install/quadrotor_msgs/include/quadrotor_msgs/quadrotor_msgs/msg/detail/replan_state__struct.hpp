// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/ReplanState.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'target'
#include "nav_msgs/msg/detail/odometry__struct.hpp"
// Member 'occmap'
#include "quadrotor_msgs/msg/detail/occ_map3d__struct.hpp"
// Member 'replan_stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quadrotor_msgs__msg__ReplanState __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__ReplanState __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ReplanState_
{
  using Type = ReplanState_<ContainerAllocator>;

  explicit ReplanState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    target(_init),
    occmap(_init),
    replan_stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  explicit ReplanState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    target(_alloc, _init),
    occmap(_alloc, _init),
    replan_stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    int16_t;
  _state_type state;
  using _ini_state_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _ini_state_type ini_state;
  using _target_type =
    nav_msgs::msg::Odometry_<ContainerAllocator>;
  _target_type target;
  using _occmap_type =
    quadrotor_msgs::msg::OccMap3d_<ContainerAllocator>;
  _occmap_type occmap;
  using _path_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _path_type path;
  using _replan_stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _replan_stamp_type replan_stamp;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const int16_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__ini_state(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->ini_state = _arg;
    return *this;
  }
  Type & set__target(
    const nav_msgs::msg::Odometry_<ContainerAllocator> & _arg)
  {
    this->target = _arg;
    return *this;
  }
  Type & set__occmap(
    const quadrotor_msgs::msg::OccMap3d_<ContainerAllocator> & _arg)
  {
    this->occmap = _arg;
    return *this;
  }
  Type & set__path(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->path = _arg;
    return *this;
  }
  Type & set__replan_stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->replan_stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::ReplanState_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::ReplanState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::ReplanState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::ReplanState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::ReplanState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::ReplanState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::ReplanState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::ReplanState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::ReplanState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::ReplanState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__ReplanState
    std::shared_ptr<quadrotor_msgs::msg::ReplanState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__ReplanState
    std::shared_ptr<quadrotor_msgs::msg::ReplanState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReplanState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->ini_state != other.ini_state) {
      return false;
    }
    if (this->target != other.target) {
      return false;
    }
    if (this->occmap != other.occmap) {
      return false;
    }
    if (this->path != other.path) {
      return false;
    }
    if (this->replan_stamp != other.replan_stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReplanState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReplanState_

// alias to use template instance with default allocator
using ReplanState =
  quadrotor_msgs::msg::ReplanState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__REPLAN_STATE__STRUCT_HPP_
