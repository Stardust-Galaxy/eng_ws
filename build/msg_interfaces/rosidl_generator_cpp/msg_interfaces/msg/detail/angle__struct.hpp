// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/Angle.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ANGLE__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ANGLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__Angle __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__Angle __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Angle_
{
  using Type = Angle_<ContainerAllocator>;

  explicit Angle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->found = 0;
      this->quaternion1 = 0.0;
      this->quaternion2 = 0.0;
      this->quaternion3 = 0.0;
      this->quaternion4 = 0.0;
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
    }
  }

  explicit Angle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->found = 0;
      this->quaternion1 = 0.0;
      this->quaternion2 = 0.0;
      this->quaternion3 = 0.0;
      this->quaternion4 = 0.0;
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
    }
  }

  // field types and members
  using _found_type =
    int8_t;
  _found_type found;
  using _quaternion1_type =
    double;
  _quaternion1_type quaternion1;
  using _quaternion2_type =
    double;
  _quaternion2_type quaternion2;
  using _quaternion3_type =
    double;
  _quaternion3_type quaternion3;
  using _quaternion4_type =
    double;
  _quaternion4_type quaternion4;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;

  // setters for named parameter idiom
  Type & set__found(
    const int8_t & _arg)
  {
    this->found = _arg;
    return *this;
  }
  Type & set__quaternion1(
    const double & _arg)
  {
    this->quaternion1 = _arg;
    return *this;
  }
  Type & set__quaternion2(
    const double & _arg)
  {
    this->quaternion2 = _arg;
    return *this;
  }
  Type & set__quaternion3(
    const double & _arg)
  {
    this->quaternion3 = _arg;
    return *this;
  }
  Type & set__quaternion4(
    const double & _arg)
  {
    this->quaternion4 = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::Angle_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::Angle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::Angle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::Angle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::Angle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::Angle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::Angle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::Angle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::Angle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::Angle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__Angle
    std::shared_ptr<msg_interfaces::msg::Angle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__Angle
    std::shared_ptr<msg_interfaces::msg::Angle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Angle_ & other) const
  {
    if (this->found != other.found) {
      return false;
    }
    if (this->quaternion1 != other.quaternion1) {
      return false;
    }
    if (this->quaternion2 != other.quaternion2) {
      return false;
    }
    if (this->quaternion3 != other.quaternion3) {
      return false;
    }
    if (this->quaternion4 != other.quaternion4) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    return true;
  }
  bool operator!=(const Angle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Angle_

// alias to use template instance with default allocator
using Angle =
  msg_interfaces::msg::Angle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__ANGLE__STRUCT_HPP_
