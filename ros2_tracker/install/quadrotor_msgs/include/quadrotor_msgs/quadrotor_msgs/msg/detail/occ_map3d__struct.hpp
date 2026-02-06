// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/OccMap3d.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__STRUCT_HPP_

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
# define DEPRECATED__quadrotor_msgs__msg__OccMap3d __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__OccMap3d __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OccMap3d_
{
  using Type = OccMap3d_<ContainerAllocator>;

  explicit OccMap3d_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->resolution = 0.0f;
      this->size_x = 0;
      this->size_y = 0;
      this->size_z = 0;
      this->offset_x = 0;
      this->offset_y = 0;
      this->offset_z = 0;
    }
  }

  explicit OccMap3d_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->resolution = 0.0f;
      this->size_x = 0;
      this->size_y = 0;
      this->size_z = 0;
      this->offset_x = 0;
      this->offset_y = 0;
      this->offset_z = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _resolution_type =
    float;
  _resolution_type resolution;
  using _size_x_type =
    int16_t;
  _size_x_type size_x;
  using _size_y_type =
    int16_t;
  _size_y_type size_y;
  using _size_z_type =
    int16_t;
  _size_z_type size_z;
  using _offset_x_type =
    int16_t;
  _offset_x_type offset_x;
  using _offset_y_type =
    int16_t;
  _offset_y_type offset_y;
  using _offset_z_type =
    int16_t;
  _offset_z_type offset_z;
  using _data_type =
    std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__resolution(
    const float & _arg)
  {
    this->resolution = _arg;
    return *this;
  }
  Type & set__size_x(
    const int16_t & _arg)
  {
    this->size_x = _arg;
    return *this;
  }
  Type & set__size_y(
    const int16_t & _arg)
  {
    this->size_y = _arg;
    return *this;
  }
  Type & set__size_z(
    const int16_t & _arg)
  {
    this->size_z = _arg;
    return *this;
  }
  Type & set__offset_x(
    const int16_t & _arg)
  {
    this->offset_x = _arg;
    return *this;
  }
  Type & set__offset_y(
    const int16_t & _arg)
  {
    this->offset_y = _arg;
    return *this;
  }
  Type & set__offset_z(
    const int16_t & _arg)
  {
    this->offset_z = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::OccMap3d_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::OccMap3d_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::OccMap3d_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::OccMap3d_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::OccMap3d_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::OccMap3d_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::OccMap3d_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::OccMap3d_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::OccMap3d_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::OccMap3d_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__OccMap3d
    std::shared_ptr<quadrotor_msgs::msg::OccMap3d_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__OccMap3d
    std::shared_ptr<quadrotor_msgs::msg::OccMap3d_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OccMap3d_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->resolution != other.resolution) {
      return false;
    }
    if (this->size_x != other.size_x) {
      return false;
    }
    if (this->size_y != other.size_y) {
      return false;
    }
    if (this->size_z != other.size_z) {
      return false;
    }
    if (this->offset_x != other.offset_x) {
      return false;
    }
    if (this->offset_y != other.offset_y) {
      return false;
    }
    if (this->offset_z != other.offset_z) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const OccMap3d_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OccMap3d_

// alias to use template instance with default allocator
using OccMap3d =
  quadrotor_msgs::msg::OccMap3d_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__OCC_MAP3D__STRUCT_HPP_
