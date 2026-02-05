// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from object_detection_msgs:msg/CarPosition.idl
// generated code does not contain a copyright notice

#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__CAR_POSITION__STRUCT_HPP_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__CAR_POSITION__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__object_detection_msgs__msg__CarPosition __attribute__((deprecated))
#else
# define DEPRECATED__object_detection_msgs__msg__CarPosition __declspec(deprecated)
#endif

namespace object_detection_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CarPosition_
{
  using Type = CarPosition_<ContainerAllocator>;

  explicit CarPosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->xmean = 0.0;
      this->ymean = 0.0;
      this->zmean = 0.0;
    }
  }

  explicit CarPosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->xmean = 0.0;
      this->ymean = 0.0;
      this->zmean = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _xmean_type =
    double;
  _xmean_type xmean;
  using _ymean_type =
    double;
  _ymean_type ymean;
  using _zmean_type =
    double;
  _zmean_type zmean;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__xmean(
    const double & _arg)
  {
    this->xmean = _arg;
    return *this;
  }
  Type & set__ymean(
    const double & _arg)
  {
    this->ymean = _arg;
    return *this;
  }
  Type & set__zmean(
    const double & _arg)
  {
    this->zmean = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    object_detection_msgs::msg::CarPosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const object_detection_msgs::msg::CarPosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<object_detection_msgs::msg::CarPosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<object_detection_msgs::msg::CarPosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      object_detection_msgs::msg::CarPosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<object_detection_msgs::msg::CarPosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      object_detection_msgs::msg::CarPosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<object_detection_msgs::msg::CarPosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<object_detection_msgs::msg::CarPosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<object_detection_msgs::msg::CarPosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__object_detection_msgs__msg__CarPosition
    std::shared_ptr<object_detection_msgs::msg::CarPosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__object_detection_msgs__msg__CarPosition
    std::shared_ptr<object_detection_msgs::msg::CarPosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarPosition_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->xmean != other.xmean) {
      return false;
    }
    if (this->ymean != other.ymean) {
      return false;
    }
    if (this->zmean != other.zmean) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarPosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarPosition_

// alias to use template instance with default allocator
using CarPosition =
  object_detection_msgs::msg::CarPosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace object_detection_msgs

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__CAR_POSITION__STRUCT_HPP_
