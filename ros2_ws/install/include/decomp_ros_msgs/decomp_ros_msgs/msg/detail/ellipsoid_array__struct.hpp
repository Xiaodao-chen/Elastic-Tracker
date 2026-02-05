// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from decomp_ros_msgs:msg/EllipsoidArray.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__STRUCT_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__STRUCT_HPP_

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
// Member 'ellipsoids'
#include "decomp_ros_msgs/msg/detail/ellipsoid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__decomp_ros_msgs__msg__EllipsoidArray __attribute__((deprecated))
#else
# define DEPRECATED__decomp_ros_msgs__msg__EllipsoidArray __declspec(deprecated)
#endif

namespace decomp_ros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EllipsoidArray_
{
  using Type = EllipsoidArray_<ContainerAllocator>;

  explicit EllipsoidArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit EllipsoidArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _ellipsoids_type =
    std::vector<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator>>>;
  _ellipsoids_type ellipsoids;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__ellipsoids(
    const std::vector<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<decomp_ros_msgs::msg::Ellipsoid_<ContainerAllocator>>> & _arg)
  {
    this->ellipsoids = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__decomp_ros_msgs__msg__EllipsoidArray
    std::shared_ptr<decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__decomp_ros_msgs__msg__EllipsoidArray
    std::shared_ptr<decomp_ros_msgs::msg::EllipsoidArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EllipsoidArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->ellipsoids != other.ellipsoids) {
      return false;
    }
    return true;
  }
  bool operator!=(const EllipsoidArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EllipsoidArray_

// alias to use template instance with default allocator
using EllipsoidArray =
  decomp_ros_msgs::msg::EllipsoidArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace decomp_ros_msgs

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__ELLIPSOID_ARRAY__STRUCT_HPP_
