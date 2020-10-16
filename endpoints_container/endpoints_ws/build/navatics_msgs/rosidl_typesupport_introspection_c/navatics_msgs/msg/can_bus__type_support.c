// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from navatics_msgs:msg/CanBus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "navatics_msgs/msg/can_bus__rosidl_typesupport_introspection_c.h"
#include "navatics_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "navatics_msgs/msg/can_bus__functions.h"
#include "navatics_msgs/msg/can_bus__struct.h"


// Include directives for member types
// Member `data`
#include "rosidl_generator_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void CanBus__rosidl_typesupport_introspection_c__CanBus_init_function(
  void * message_memory, enum rosidl_runtime_c_message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  navatics_msgs__msg__CanBus__init(message_memory);
}

void CanBus__rosidl_typesupport_introspection_c__CanBus_fini_function(void * message_memory)
{
  navatics_msgs__msg__CanBus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CanBus__rosidl_typesupport_introspection_c__CanBus_message_member_array[3] = {
  {
    "std_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navatics_msgs__msg__CanBus, std_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "length",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navatics_msgs__msg__CanBus, length),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navatics_msgs__msg__CanBus, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CanBus__rosidl_typesupport_introspection_c__CanBus_message_members = {
  "navatics_msgs__msg",  // message namespace
  "CanBus",  // message name
  3,  // number of fields
  sizeof(navatics_msgs__msg__CanBus),
  CanBus__rosidl_typesupport_introspection_c__CanBus_message_member_array,  // message members
  CanBus__rosidl_typesupport_introspection_c__CanBus_init_function,  // function to initialize message memory (memory has to be allocated)
  CanBus__rosidl_typesupport_introspection_c__CanBus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CanBus__rosidl_typesupport_introspection_c__CanBus_message_type_support_handle = {
  0,
  &CanBus__rosidl_typesupport_introspection_c__CanBus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_navatics_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navatics_msgs, msg, CanBus)() {
  if (!CanBus__rosidl_typesupport_introspection_c__CanBus_message_type_support_handle.typesupport_identifier) {
    CanBus__rosidl_typesupport_introspection_c__CanBus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CanBus__rosidl_typesupport_introspection_c__CanBus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
