// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navatics_msgs:msg/CanBus.idl
// generated code does not contain a copyright notice

#ifndef NAVATICS_MSGS__MSG__CAN_BUS__FUNCTIONS_H_
#define NAVATICS_MSGS__MSG__CAN_BUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_generator_c/visibility_control.h"
#include "navatics_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "navatics_msgs/msg/can_bus__struct.h"

/// Initialize msg/CanBus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * navatics_msgs__msg__CanBus
 * )) before or use
 * navatics_msgs__msg__CanBus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_navatics_msgs
bool
navatics_msgs__msg__CanBus__init(navatics_msgs__msg__CanBus * msg);

/// Finalize msg/CanBus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_navatics_msgs
void
navatics_msgs__msg__CanBus__fini(navatics_msgs__msg__CanBus * msg);

/// Create msg/CanBus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * navatics_msgs__msg__CanBus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_navatics_msgs
navatics_msgs__msg__CanBus *
navatics_msgs__msg__CanBus__create();

/// Destroy msg/CanBus message.
/**
 * It calls
 * navatics_msgs__msg__CanBus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_navatics_msgs
void
navatics_msgs__msg__CanBus__destroy(navatics_msgs__msg__CanBus * msg);


/// Initialize array of msg/CanBus messages.
/**
 * It allocates the memory for the number of elements and calls
 * navatics_msgs__msg__CanBus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_navatics_msgs
bool
navatics_msgs__msg__CanBus__Sequence__init(navatics_msgs__msg__CanBus__Sequence * array, size_t size);

/// Finalize array of msg/CanBus messages.
/**
 * It calls
 * navatics_msgs__msg__CanBus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_navatics_msgs
void
navatics_msgs__msg__CanBus__Sequence__fini(navatics_msgs__msg__CanBus__Sequence * array);

/// Create array of msg/CanBus messages.
/**
 * It allocates the memory for the array and calls
 * navatics_msgs__msg__CanBus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_navatics_msgs
navatics_msgs__msg__CanBus__Sequence *
navatics_msgs__msg__CanBus__Sequence__create(size_t size);

/// Destroy array of msg/CanBus messages.
/**
 * It calls
 * navatics_msgs__msg__CanBus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_navatics_msgs
void
navatics_msgs__msg__CanBus__Sequence__destroy(navatics_msgs__msg__CanBus__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // NAVATICS_MSGS__MSG__CAN_BUS__FUNCTIONS_H_
