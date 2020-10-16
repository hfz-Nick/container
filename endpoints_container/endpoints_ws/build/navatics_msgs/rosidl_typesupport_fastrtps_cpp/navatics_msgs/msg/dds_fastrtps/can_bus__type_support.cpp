// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from navatics_msgs:msg/CanBus.idl
// generated code does not contain a copyright notice
#include "navatics_msgs/msg/can_bus__rosidl_typesupport_fastrtps_cpp.hpp"
#include "navatics_msgs/msg/can_bus__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: std_id
  cdr << ros_message.std_id;
  // Member: length
  cdr << ros_message.length;
  // Member: data
  {
    cdr << ros_message.data;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navatics_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  navatics_msgs::msg::CanBus & ros_message)
{
  // Member: std_id
  cdr >> ros_message.std_id;

  // Member: length
  cdr >> ros_message.length;

  // Member: data
  {
    cdr >> ros_message.data;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navatics_msgs
get_serialized_size(
  const navatics_msgs::msg::CanBus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: std_id
  {
    size_t item_size = sizeof(ros_message.std_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: length
  {
    size_t item_size = sizeof(ros_message.length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: data
  {
    size_t array_size = ros_message.data.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.data[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navatics_msgs
max_serialized_size_CanBus(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: std_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: length
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: data
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _CanBus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const navatics_msgs::msg::CanBus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _CanBus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<navatics_msgs::msg::CanBus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _CanBus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const navatics_msgs::msg::CanBus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _CanBus__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_CanBus(full_bounded, 0);
}

static message_type_support_callbacks_t _CanBus__callbacks = {
  "navatics_msgs::msg",
  "CanBus",
  _CanBus__cdr_serialize,
  _CanBus__cdr_deserialize,
  _CanBus__get_serialized_size,
  _CanBus__max_serialized_size
};

static rosidl_message_type_support_t _CanBus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_CanBus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace navatics_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_navatics_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<navatics_msgs::msg::CanBus>()
{
  return &navatics_msgs::msg::typesupport_fastrtps_cpp::_CanBus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, navatics_msgs, msg, CanBus)() {
  return &navatics_msgs::msg::typesupport_fastrtps_cpp::_CanBus__handle;
}

#ifdef __cplusplus
}
#endif
