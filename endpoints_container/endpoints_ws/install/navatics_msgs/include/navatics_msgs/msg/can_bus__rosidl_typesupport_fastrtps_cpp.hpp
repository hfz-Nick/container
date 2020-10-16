// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from navatics_msgs:msg/CanBus.idl
// generated code does not contain a copyright notice

#ifndef NAVATICS_MSGS__MSG__CAN_BUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define NAVATICS_MSGS__MSG__CAN_BUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "navatics_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "navatics_msgs/msg/can_bus__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace navatics_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navatics_msgs
cdr_serialize(
  const navatics_msgs::msg::CanBus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navatics_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  navatics_msgs::msg::CanBus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navatics_msgs
get_serialized_size(
  const navatics_msgs::msg::CanBus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navatics_msgs
max_serialized_size_CanBus(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace navatics_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navatics_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, navatics_msgs, msg, CanBus)();

#ifdef __cplusplus
}
#endif

#endif  // NAVATICS_MSGS__MSG__CAN_BUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
