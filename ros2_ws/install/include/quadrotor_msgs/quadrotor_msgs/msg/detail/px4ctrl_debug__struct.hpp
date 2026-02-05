// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/Px4ctrlDebug.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadrotor_msgs/msg/px4ctrl_debug.hpp"


#ifndef QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__STRUCT_HPP_

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
# define DEPRECATED__quadrotor_msgs__msg__Px4ctrlDebug __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__Px4ctrlDebug __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Px4ctrlDebug_
{
  using Type = Px4ctrlDebug_<ContainerAllocator>;

  explicit Px4ctrlDebug_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->des_v_x = 0.0;
      this->des_v_y = 0.0;
      this->des_v_z = 0.0;
      this->fb_a_x = 0.0;
      this->fb_a_y = 0.0;
      this->fb_a_z = 0.0;
      this->des_a_x = 0.0;
      this->des_a_y = 0.0;
      this->des_a_z = 0.0;
      this->des_q_x = 0.0;
      this->des_q_y = 0.0;
      this->des_q_z = 0.0;
      this->des_q_w = 0.0;
      this->des_thr = 0.0;
      this->thr2acc = 0.0;
      this->thr_scale_compensate = 0.0;
      this->voltage = 0.0;
      this->err_axisang_x = 0.0;
      this->err_axisang_y = 0.0;
      this->err_axisang_z = 0.0;
      this->err_axisang_ang = 0.0;
      this->fb_rate_x = 0.0;
      this->fb_rate_y = 0.0;
      this->fb_rate_z = 0.0;
    }
  }

  explicit Px4ctrlDebug_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->des_v_x = 0.0;
      this->des_v_y = 0.0;
      this->des_v_z = 0.0;
      this->fb_a_x = 0.0;
      this->fb_a_y = 0.0;
      this->fb_a_z = 0.0;
      this->des_a_x = 0.0;
      this->des_a_y = 0.0;
      this->des_a_z = 0.0;
      this->des_q_x = 0.0;
      this->des_q_y = 0.0;
      this->des_q_z = 0.0;
      this->des_q_w = 0.0;
      this->des_thr = 0.0;
      this->thr2acc = 0.0;
      this->thr_scale_compensate = 0.0;
      this->voltage = 0.0;
      this->err_axisang_x = 0.0;
      this->err_axisang_y = 0.0;
      this->err_axisang_z = 0.0;
      this->err_axisang_ang = 0.0;
      this->fb_rate_x = 0.0;
      this->fb_rate_y = 0.0;
      this->fb_rate_z = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _des_v_x_type =
    double;
  _des_v_x_type des_v_x;
  using _des_v_y_type =
    double;
  _des_v_y_type des_v_y;
  using _des_v_z_type =
    double;
  _des_v_z_type des_v_z;
  using _fb_a_x_type =
    double;
  _fb_a_x_type fb_a_x;
  using _fb_a_y_type =
    double;
  _fb_a_y_type fb_a_y;
  using _fb_a_z_type =
    double;
  _fb_a_z_type fb_a_z;
  using _des_a_x_type =
    double;
  _des_a_x_type des_a_x;
  using _des_a_y_type =
    double;
  _des_a_y_type des_a_y;
  using _des_a_z_type =
    double;
  _des_a_z_type des_a_z;
  using _des_q_x_type =
    double;
  _des_q_x_type des_q_x;
  using _des_q_y_type =
    double;
  _des_q_y_type des_q_y;
  using _des_q_z_type =
    double;
  _des_q_z_type des_q_z;
  using _des_q_w_type =
    double;
  _des_q_w_type des_q_w;
  using _des_thr_type =
    double;
  _des_thr_type des_thr;
  using _thr2acc_type =
    double;
  _thr2acc_type thr2acc;
  using _thr_scale_compensate_type =
    double;
  _thr_scale_compensate_type thr_scale_compensate;
  using _voltage_type =
    double;
  _voltage_type voltage;
  using _err_axisang_x_type =
    double;
  _err_axisang_x_type err_axisang_x;
  using _err_axisang_y_type =
    double;
  _err_axisang_y_type err_axisang_y;
  using _err_axisang_z_type =
    double;
  _err_axisang_z_type err_axisang_z;
  using _err_axisang_ang_type =
    double;
  _err_axisang_ang_type err_axisang_ang;
  using _fb_rate_x_type =
    double;
  _fb_rate_x_type fb_rate_x;
  using _fb_rate_y_type =
    double;
  _fb_rate_y_type fb_rate_y;
  using _fb_rate_z_type =
    double;
  _fb_rate_z_type fb_rate_z;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__des_v_x(
    const double & _arg)
  {
    this->des_v_x = _arg;
    return *this;
  }
  Type & set__des_v_y(
    const double & _arg)
  {
    this->des_v_y = _arg;
    return *this;
  }
  Type & set__des_v_z(
    const double & _arg)
  {
    this->des_v_z = _arg;
    return *this;
  }
  Type & set__fb_a_x(
    const double & _arg)
  {
    this->fb_a_x = _arg;
    return *this;
  }
  Type & set__fb_a_y(
    const double & _arg)
  {
    this->fb_a_y = _arg;
    return *this;
  }
  Type & set__fb_a_z(
    const double & _arg)
  {
    this->fb_a_z = _arg;
    return *this;
  }
  Type & set__des_a_x(
    const double & _arg)
  {
    this->des_a_x = _arg;
    return *this;
  }
  Type & set__des_a_y(
    const double & _arg)
  {
    this->des_a_y = _arg;
    return *this;
  }
  Type & set__des_a_z(
    const double & _arg)
  {
    this->des_a_z = _arg;
    return *this;
  }
  Type & set__des_q_x(
    const double & _arg)
  {
    this->des_q_x = _arg;
    return *this;
  }
  Type & set__des_q_y(
    const double & _arg)
  {
    this->des_q_y = _arg;
    return *this;
  }
  Type & set__des_q_z(
    const double & _arg)
  {
    this->des_q_z = _arg;
    return *this;
  }
  Type & set__des_q_w(
    const double & _arg)
  {
    this->des_q_w = _arg;
    return *this;
  }
  Type & set__des_thr(
    const double & _arg)
  {
    this->des_thr = _arg;
    return *this;
  }
  Type & set__thr2acc(
    const double & _arg)
  {
    this->thr2acc = _arg;
    return *this;
  }
  Type & set__thr_scale_compensate(
    const double & _arg)
  {
    this->thr_scale_compensate = _arg;
    return *this;
  }
  Type & set__voltage(
    const double & _arg)
  {
    this->voltage = _arg;
    return *this;
  }
  Type & set__err_axisang_x(
    const double & _arg)
  {
    this->err_axisang_x = _arg;
    return *this;
  }
  Type & set__err_axisang_y(
    const double & _arg)
  {
    this->err_axisang_y = _arg;
    return *this;
  }
  Type & set__err_axisang_z(
    const double & _arg)
  {
    this->err_axisang_z = _arg;
    return *this;
  }
  Type & set__err_axisang_ang(
    const double & _arg)
  {
    this->err_axisang_ang = _arg;
    return *this;
  }
  Type & set__fb_rate_x(
    const double & _arg)
  {
    this->fb_rate_x = _arg;
    return *this;
  }
  Type & set__fb_rate_y(
    const double & _arg)
  {
    this->fb_rate_y = _arg;
    return *this;
  }
  Type & set__fb_rate_z(
    const double & _arg)
  {
    this->fb_rate_z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__Px4ctrlDebug
    std::shared_ptr<quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__Px4ctrlDebug
    std::shared_ptr<quadrotor_msgs::msg::Px4ctrlDebug_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Px4ctrlDebug_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->des_v_x != other.des_v_x) {
      return false;
    }
    if (this->des_v_y != other.des_v_y) {
      return false;
    }
    if (this->des_v_z != other.des_v_z) {
      return false;
    }
    if (this->fb_a_x != other.fb_a_x) {
      return false;
    }
    if (this->fb_a_y != other.fb_a_y) {
      return false;
    }
    if (this->fb_a_z != other.fb_a_z) {
      return false;
    }
    if (this->des_a_x != other.des_a_x) {
      return false;
    }
    if (this->des_a_y != other.des_a_y) {
      return false;
    }
    if (this->des_a_z != other.des_a_z) {
      return false;
    }
    if (this->des_q_x != other.des_q_x) {
      return false;
    }
    if (this->des_q_y != other.des_q_y) {
      return false;
    }
    if (this->des_q_z != other.des_q_z) {
      return false;
    }
    if (this->des_q_w != other.des_q_w) {
      return false;
    }
    if (this->des_thr != other.des_thr) {
      return false;
    }
    if (this->thr2acc != other.thr2acc) {
      return false;
    }
    if (this->thr_scale_compensate != other.thr_scale_compensate) {
      return false;
    }
    if (this->voltage != other.voltage) {
      return false;
    }
    if (this->err_axisang_x != other.err_axisang_x) {
      return false;
    }
    if (this->err_axisang_y != other.err_axisang_y) {
      return false;
    }
    if (this->err_axisang_z != other.err_axisang_z) {
      return false;
    }
    if (this->err_axisang_ang != other.err_axisang_ang) {
      return false;
    }
    if (this->fb_rate_x != other.fb_rate_x) {
      return false;
    }
    if (this->fb_rate_y != other.fb_rate_y) {
      return false;
    }
    if (this->fb_rate_z != other.fb_rate_z) {
      return false;
    }
    return true;
  }
  bool operator!=(const Px4ctrlDebug_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Px4ctrlDebug_

// alias to use template instance with default allocator
using Px4ctrlDebug =
  quadrotor_msgs::msg::Px4ctrlDebug_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__STRUCT_HPP_
