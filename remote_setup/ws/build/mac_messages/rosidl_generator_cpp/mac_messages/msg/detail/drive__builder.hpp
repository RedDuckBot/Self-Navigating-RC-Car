// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mac_messages:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef MAC_MESSAGES__MSG__DETAIL__DRIVE__BUILDER_HPP_
#define MAC_MESSAGES__MSG__DETAIL__DRIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mac_messages/msg/detail/drive__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mac_messages
{

namespace msg
{

namespace builder
{

class Init_Drive_control_input
{
public:
  explicit Init_Drive_control_input(::mac_messages::msg::Drive & msg)
  : msg_(msg)
  {}
  ::mac_messages::msg::Drive control_input(::mac_messages::msg::Drive::_control_input_type arg)
  {
    msg_.control_input = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mac_messages::msg::Drive msg_;
};

class Init_Drive_command
{
public:
  Init_Drive_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Drive_control_input command(::mac_messages::msg::Drive::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_Drive_control_input(msg_);
  }

private:
  ::mac_messages::msg::Drive msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mac_messages::msg::Drive>()
{
  return mac_messages::msg::builder::Init_Drive_command();
}

}  // namespace mac_messages

#endif  // MAC_MESSAGES__MSG__DETAIL__DRIVE__BUILDER_HPP_
