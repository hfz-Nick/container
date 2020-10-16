// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from navatics_msgs:msg/CanBus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "navatics_msgs/msg/can_bus__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace navatics_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CanBus_init_function(
  void * message_memory, rosidl_generator_cpp::MessageInitialization _init)
{
  new (message_memory) navatics_msgs::msg::CanBus(_init);
}

void CanBus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<navatics_msgs::msg::CanBus *>(message_memory);
  typed_message->~CanBus();
}

size_t size_function__CanBus__data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CanBus__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__CanBus__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void resize_function__CanBus__data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CanBus_message_member_array[3] = {
  {
    "std_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navatics_msgs::msg::CanBus, std_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "length",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navatics_msgs::msg::CanBus, length),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navatics_msgs::msg::CanBus, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__CanBus__data,  // size() function pointer
    get_const_function__CanBus__data,  // get_const(index) function pointer
    get_function__CanBus__data,  // get(index) function pointer
    resize_function__CanBus__data  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CanBus_message_members = {
  "navatics_msgs::msg",  // message namespace
  "CanBus",  // message name
  3,  // number of fields
  sizeof(navatics_msgs::msg::CanBus),
  CanBus_message_member_array,  // message members
  CanBus_init_function,  // function to initialize message memory (memory has to be allocated)
  CanBus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CanBus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CanBus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace navatics_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<navatics_msgs::msg::CanBus>()
{
  return &::navatics_msgs::msg::rosidl_typesupport_introspection_cpp::CanBus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, navatics_msgs, msg, CanBus)() {
  return &::navatics_msgs::msg::rosidl_typesupport_introspection_cpp::CanBus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
