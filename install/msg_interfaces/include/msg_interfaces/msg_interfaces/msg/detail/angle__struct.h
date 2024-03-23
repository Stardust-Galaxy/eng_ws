// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_interfaces:msg/Angle.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ANGLE__STRUCT_H_
#define MSG_INTERFACES__MSG__DETAIL__ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Angle in the package msg_interfaces.
typedef struct msg_interfaces__msg__Angle
{
  int8_t found;
  double quaternion1;
  double quaternion2;
  double quaternion3;
  double quaternion4;
  double x;
  double y;
  double z;
} msg_interfaces__msg__Angle;

// Struct for a sequence of msg_interfaces__msg__Angle.
typedef struct msg_interfaces__msg__Angle__Sequence
{
  msg_interfaces__msg__Angle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_interfaces__msg__Angle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__DETAIL__ANGLE__STRUCT_H_
