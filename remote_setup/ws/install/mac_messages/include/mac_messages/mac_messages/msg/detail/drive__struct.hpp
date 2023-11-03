// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mac_messages:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef MAC_MESSAGES__MSG__DETAIL__DRIVE__STRUCT_HPP_
#define MAC_MESSAGES__MSG__DETAIL__DRIVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mac_messages__msg__Drive __attribute__((deprecated))
#else
# define DEPRECATED__mac_messages__msg__Drive __declspec(deprecated)
#endif

namespace mac_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Drive_
{
  using Type = Drive_<ContainerAllocator>;

  explicit Drive_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->control_input = 0;
    }
  }

  explicit Drive_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->control_input = 0;
    }
  }

  // field types and members
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;
  using _control_input_type =
    int16_t;
  _control_input_type control_input;

  // setters for named parameter idiom
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__control_input(
    const int16_t & _arg)
  {
    this->control_input = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mac_messages::msg::Drive_<ContainerAllocator> *;
  using ConstRawPtr =
    const mac_messages::msg::Drive_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mac_messages::msg::Drive_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mac_messages::msg::Drive_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mac_messages::msg::Drive_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mac_messages::msg::Drive_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mac_messages::msg::Drive_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mac_messages::msg::Drive_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mac_messages::msg::Drive_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mac_messages::msg::Drive_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mac_messages__msg__Drive
    std::shared_ptr<mac_messages::msg::Drive_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mac_messages__msg__Drive
    std::shared_ptr<mac_messages::msg::Drive_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->control_input != other.control_input) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_

// alias to use template instance with default allocator
using Drive =
  mac_messages::msg::Drive_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mac_messages

#endif  // MAC_MESSAGES__MSG__DETAIL__DRIVE__STRUCT_HPP_
