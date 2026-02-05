// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/TakeoffLand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__quadrotor_msgs__msg__TakeoffLand __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__TakeoffLand __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TakeoffLand_
{
  using Type = TakeoffLand_<ContainerAllocator>;

  explicit TakeoffLand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->takeoff_land_cmd = 0;
    }
  }

  explicit TakeoffLand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->takeoff_land_cmd = 0;
    }
  }

  // field types and members
  using _takeoff_land_cmd_type =
    uint8_t;
  _takeoff_land_cmd_type takeoff_land_cmd;

  // setters for named parameter idiom
  Type & set__takeoff_land_cmd(
    const uint8_t & _arg)
  {
    this->takeoff_land_cmd = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t TAKEOFF =
    1u;
  static constexpr uint8_t LAND =
    2u;

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__TakeoffLand
    std::shared_ptr<quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__TakeoffLand
    std::shared_ptr<quadrotor_msgs::msg::TakeoffLand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TakeoffLand_ & other) const
  {
    if (this->takeoff_land_cmd != other.takeoff_land_cmd) {
      return false;
    }
    return true;
  }
  bool operator!=(const TakeoffLand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TakeoffLand_

// alias to use template instance with default allocator
using TakeoffLand =
  quadrotor_msgs::msg::TakeoffLand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TakeoffLand_<ContainerAllocator>::TAKEOFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TakeoffLand_<ContainerAllocator>::LAND;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__TAKEOFF_LAND__STRUCT_HPP_
