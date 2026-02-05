// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from decomp_ros_msgs:msg/Polyhedron.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__STRUCT_HPP_
#define DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'points'
// Member 'normals'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__decomp_ros_msgs__msg__Polyhedron __attribute__((deprecated))
#else
# define DEPRECATED__decomp_ros_msgs__msg__Polyhedron __declspec(deprecated)
#endif

namespace decomp_ros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Polyhedron_
{
  using Type = Polyhedron_<ContainerAllocator>;

  explicit Polyhedron_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Polyhedron_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _points_type points;
  using _normals_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _normals_type normals;

  // setters for named parameter idiom
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__normals(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->normals = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator> *;
  using ConstRawPtr =
    const decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__decomp_ros_msgs__msg__Polyhedron
    std::shared_ptr<decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__decomp_ros_msgs__msg__Polyhedron
    std::shared_ptr<decomp_ros_msgs::msg::Polyhedron_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Polyhedron_ & other) const
  {
    if (this->points != other.points) {
      return false;
    }
    if (this->normals != other.normals) {
      return false;
    }
    return true;
  }
  bool operator!=(const Polyhedron_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Polyhedron_

// alias to use template instance with default allocator
using Polyhedron =
  decomp_ros_msgs::msg::Polyhedron_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace decomp_ros_msgs

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__STRUCT_HPP_
