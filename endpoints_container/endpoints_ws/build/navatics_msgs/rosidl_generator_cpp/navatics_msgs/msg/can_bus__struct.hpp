// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from navatics_msgs:msg/CanBus.idl
// generated code does not contain a copyright notice

#ifndef NAVATICS_MSGS__MSG__CAN_BUS__STRUCT_HPP_
#define NAVATICS_MSGS__MSG__CAN_BUS__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__navatics_msgs__msg__CanBus __attribute__((deprecated))
#else
# define DEPRECATED__navatics_msgs__msg__CanBus __declspec(deprecated)
#endif

namespace navatics_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CanBus_
{
  using Type = CanBus_<ContainerAllocator>;

  explicit CanBus_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->std_id = 0;
      this->length = 0;
    }
  }

  explicit CanBus_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->std_id = 0;
      this->length = 0;
    }
  }

  // field types and members
  using _std_id_type =
    uint16_t;
  _std_id_type std_id;
  using _length_type =
    uint8_t;
  _length_type length;
  using _data_type =
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__std_id(
    const uint16_t & _arg)
  {
    this->std_id = _arg;
    return *this;
  }
  Type & set__length(
    const uint8_t & _arg)
  {
    this->length = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    navatics_msgs::msg::CanBus_<ContainerAllocator> *;
  using ConstRawPtr =
    const navatics_msgs::msg::CanBus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<navatics_msgs::msg::CanBus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<navatics_msgs::msg::CanBus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      navatics_msgs::msg::CanBus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<navatics_msgs::msg::CanBus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      navatics_msgs::msg::CanBus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<navatics_msgs::msg::CanBus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<navatics_msgs::msg::CanBus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<navatics_msgs::msg::CanBus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__navatics_msgs__msg__CanBus
    std::shared_ptr<navatics_msgs::msg::CanBus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__navatics_msgs__msg__CanBus
    std::shared_ptr<navatics_msgs::msg::CanBus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CanBus_ & other) const
  {
    if (this->std_id != other.std_id) {
      return false;
    }
    if (this->length != other.length) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const CanBus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CanBus_

// alias to use template instance with default allocator
using CanBus =
  navatics_msgs::msg::CanBus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace navatics_msgs

#endif  // NAVATICS_MSGS__MSG__CAN_BUS__STRUCT_HPP_
