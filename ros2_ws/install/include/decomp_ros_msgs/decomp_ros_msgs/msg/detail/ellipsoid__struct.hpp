// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from decomp_ros_msgs:msg/Ellipsoid.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "decomp_ros_msgs/msg/ellipsoid.hpp"


#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__STRUCT_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__decomp_ros_msgs__msg__Ellipsoid __attribute__((deprecated))
#else
# define DEPRECATED__decomp_ros_msgs__msg__Ellipsoid __declspec(deprecated)
#endif

namespace decomp_ros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Ellipsoid_
{
  using Type = Ellipsoid_<ContainerAllocator>;

  explicit Ellipsoid_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->d.begin(), this->d.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->e.begin(), this->e.end(), 0.0);
    }
  }

  explicit Ellipsoid_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : d(_alloc),
    e(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->d.begin(), this->d.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->e.begin(), this->e.end(), 0.0);
    }
  }

  // field types and members
  using _d_type =
    std::array<double, 3>;
  _d_type d;
  using _e_type =
    std::array<double, 9>;
  _e_type e;

  // setters for named parameter idiom
  Type & set__d(
    const std::array<double, 3> & _arg)
  {
    this->d = _arg;
    return *this;
  }
  Type & set__e(
    const std::array<double, 9> & _arg)
  {
    this->e = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator> *;
  using ConstRawPtr =
    const decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__decomp_ros_msgs__msg__Ellipsoid
    std::shared_ptr<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__decomp_ros_msgs__msg__Ellipsoid
    std::shared_ptr<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Ellipsoid_ & other) const
  {
    if (this->d != other.d) {
      return false;
    }
    if (this->e != other.e) {
      return false;
    }
    return true;
  }
  bool operator!=(const Ellipsoid_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Ellipsoid_

// alias to use template instance with default allocator
using Ellipsoid =
  decomp_ros_msgs::msg::Ellipsoid_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace decomp_ros_msgs

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID__STRUCT_HPP_
