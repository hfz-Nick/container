// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from navatics_msgs:msg/CanBus.idl
// generated code does not contain a copyright notice

#ifndef NAVATICS_MSGS__MSG__CAN_BUS__TRAITS_HPP_
#define NAVATICS_MSGS__MSG__CAN_BUS__TRAITS_HPP_

#include "navatics_msgs/msg/can_bus__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<navatics_msgs::msg::CanBus>()
{
  return "navatics_msgs::msg::CanBus";
}

template<>
struct has_fixed_size<navatics_msgs::msg::CanBus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<navatics_msgs::msg::CanBus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<navatics_msgs::msg::CanBus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NAVATICS_MSGS__MSG__CAN_BUS__TRAITS_HPP_
