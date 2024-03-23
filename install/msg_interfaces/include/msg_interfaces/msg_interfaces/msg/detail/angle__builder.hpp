// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/Angle.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ANGLE__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_Angle_z
{
public:
  explicit Init_Angle_z(::msg_interfaces::msg::Angle & msg)
  : msg_(msg)
  {}
  ::msg_interfaces::msg::Angle z(::msg_interfaces::msg::Angle::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::Angle msg_;
};

class Init_Angle_y
{
public:
  explicit Init_Angle_y(::msg_interfaces::msg::Angle & msg)
  : msg_(msg)
  {}
  Init_Angle_z y(::msg_interfaces::msg::Angle::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Angle_z(msg_);
  }

private:
  ::msg_interfaces::msg::Angle msg_;
};

class Init_Angle_x
{
public:
  explicit Init_Angle_x(::msg_interfaces::msg::Angle & msg)
  : msg_(msg)
  {}
  Init_Angle_y x(::msg_interfaces::msg::Angle::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Angle_y(msg_);
  }

private:
  ::msg_interfaces::msg::Angle msg_;
};

class Init_Angle_quaternion4
{
public:
  explicit Init_Angle_quaternion4(::msg_interfaces::msg::Angle & msg)
  : msg_(msg)
  {}
  Init_Angle_x quaternion4(::msg_interfaces::msg::Angle::_quaternion4_type arg)
  {
    msg_.quaternion4 = std::move(arg);
    return Init_Angle_x(msg_);
  }

private:
  ::msg_interfaces::msg::Angle msg_;
};

class Init_Angle_quaternion3
{
public:
  explicit Init_Angle_quaternion3(::msg_interfaces::msg::Angle & msg)
  : msg_(msg)
  {}
  Init_Angle_quaternion4 quaternion3(::msg_interfaces::msg::Angle::_quaternion3_type arg)
  {
    msg_.quaternion3 = std::move(arg);
    return Init_Angle_quaternion4(msg_);
  }

private:
  ::msg_interfaces::msg::Angle msg_;
};

class Init_Angle_quaternion2
{
public:
  explicit Init_Angle_quaternion2(::msg_interfaces::msg::Angle & msg)
  : msg_(msg)
  {}
  Init_Angle_quaternion3 quaternion2(::msg_interfaces::msg::Angle::_quaternion2_type arg)
  {
    msg_.quaternion2 = std::move(arg);
    return Init_Angle_quaternion3(msg_);
  }

private:
  ::msg_interfaces::msg::Angle msg_;
};

class Init_Angle_quaternion1
{
public:
  explicit Init_Angle_quaternion1(::msg_interfaces::msg::Angle & msg)
  : msg_(msg)
  {}
  Init_Angle_quaternion2 quaternion1(::msg_interfaces::msg::Angle::_quaternion1_type arg)
  {
    msg_.quaternion1 = std::move(arg);
    return Init_Angle_quaternion2(msg_);
  }

private:
  ::msg_interfaces::msg::Angle msg_;
};

class Init_Angle_found
{
public:
  Init_Angle_found()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Angle_quaternion1 found(::msg_interfaces::msg::Angle::_found_type arg)
  {
    msg_.found = std::move(arg);
    return Init_Angle_quaternion1(msg_);
  }

private:
  ::msg_interfaces::msg::Angle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::Angle>()
{
  return msg_interfaces::msg::builder::Init_Angle_found();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__ANGLE__BUILDER_HPP_
