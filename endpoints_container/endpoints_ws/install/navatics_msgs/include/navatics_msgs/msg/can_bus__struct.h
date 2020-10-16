// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navatics_msgs:msg/CanBus.idl
// generated code does not contain a copyright notice

#ifndef NAVATICS_MSGS__MSG__CAN_BUS__STRUCT_H_
#define NAVATICS_MSGS__MSG__CAN_BUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_generator_c/primitives_sequence.h"

// Struct defined in msg/CanBus in the package navatics_msgs.
typedef struct navatics_msgs__msg__CanBus
{
  uint16_t std_id;
  uint8_t length;
  rosidl_generator_c__uint8__Sequence data;
} navatics_msgs__msg__CanBus;

// Struct for a sequence of navatics_msgs__msg__CanBus.
typedef struct navatics_msgs__msg__CanBus__Sequence
{
  navatics_msgs__msg__CanBus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navatics_msgs__msg__CanBus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAVATICS_MSGS__MSG__CAN_BUS__STRUCT_H_
