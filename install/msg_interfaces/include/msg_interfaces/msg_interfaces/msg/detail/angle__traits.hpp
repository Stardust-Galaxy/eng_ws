// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/Angle.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ANGLE__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ANGLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/angle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Angle & msg,
  std::ostream & out)
{
  out << "{";
  // member: found
  {
    out << "found: ";
    rosidl_generator_traits::value_to_yaml(msg.found, out);
    out << ", ";
  }

  // member: quaternion1
  {
    out << "quaternion1: ";
    rosidl_generator_traits::value_to_yaml(msg.quaternion1, out);
    out << ", ";
  }

  // member: quaternion2
  {
    out << "quaternion2: ";
    rosidl_generator_traits::value_to_yaml(msg.quaternion2, out);
    out << ", ";
  }

  // member: quaternion3
  {
    out << "quaternion3: ";
    rosidl_generator_traits::value_to_yaml(msg.quaternion3, out);
    out << ", ";
  }

  // member: quaternion4
  {
    out << "quaternion4: ";
    rosidl_generator_traits::value_to_yaml(msg.quaternion4, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Angle & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: found
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "found: ";
    rosidl_generator_traits::value_to_yaml(msg.found, out);
    out << "\n";
  }

  // member: quaternion1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quaternion1: ";
    rosidl_generator_traits::value_to_yaml(msg.quaternion1, out);
    out << "\n";
  }

  // member: quaternion2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quaternion2: ";
    rosidl_generator_traits::value_to_yaml(msg.quaternion2, out);
    out << "\n";
  }

  // member: quaternion3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quaternion3: ";
    rosidl_generator_traits::value_to_yaml(msg.quaternion3, out);
    out << "\n";
  }

  // member: quaternion4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quaternion4: ";
    rosidl_generator_traits::value_to_yaml(msg.quaternion4, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Angle & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace msg_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use msg_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const msg_interfaces::msg::Angle & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::Angle & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::Angle>()
{
  return "msg_interfaces::msg::Angle";
}

template<>
inline const char * name<msg_interfaces::msg::Angle>()
{
  return "msg_interfaces/msg/Angle";
}

template<>
struct has_fixed_size<msg_interfaces::msg::Angle>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_interfaces::msg::Angle>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_interfaces::msg::Angle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__ANGLE__TRAITS_HPP_
