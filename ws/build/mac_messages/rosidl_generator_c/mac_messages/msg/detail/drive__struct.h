// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mac_messages:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef MAC_MESSAGES__MSG__DETAIL__DRIVE__STRUCT_H_
#define MAC_MESSAGES__MSG__DETAIL__DRIVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Drive in the package mac_messages.
/**
  * Command values: F --> Drive forward
  *                 B --> Drive backward
  *                 S --> Control steering
  *                 E --> Enable drive mode 
  *                 D --> Disable drive mode
 */
typedef struct mac_messages__msg__Drive
{
  rosidl_runtime_c__String command;
  /// Associated with speed or angle
  int16_t control_input;
} mac_messages__msg__Drive;

// Struct for a sequence of mac_messages__msg__Drive.
typedef struct mac_messages__msg__Drive__Sequence
{
  mac_messages__msg__Drive * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mac_messages__msg__Drive__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAC_MESSAGES__MSG__DETAIL__DRIVE__STRUCT_H_
