// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mac_messages:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef MAC_MESSAGES__MSG__DETAIL__DRIVE__TRAITS_HPP_
#define MAC_MESSAGES__MSG__DETAIL__DRIVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mac_messages/msg/detail/drive__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mac_messages
{

namespace msg
{

inline void to_flow_style_yaml(
  const Drive & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: control_input
  {
    out << "control_input: ";
    rosidl_generator_traits::value_to_yaml(msg.control_input, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Drive & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: control_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_input: ";
    rosidl_generator_traits::value_to_yaml(msg.control_input, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Drive & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mac_messages

namespace rosidl_generator_traits
{

[[deprecated("use mac_messages::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mac_messages::msg::Drive & msg,
  std::ostream & out, size_t indentation = 0)
{
  mac_messages::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mac_messages::msg::to_yaml() instead")]]
inline std::string to_yaml(const mac_messages::msg::Drive & msg)
{
  return mac_messages::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mac_messages::msg::Drive>()
{
  return "mac_messages::msg::Drive";
}

template<>
inline const char * name<mac_messages::msg::Drive>()
{
  return "mac_messages/msg/Drive";
}

template<>
struct has_fixed_size<mac_messages::msg::Drive>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mac_messages::msg::Drive>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mac_messages::msg::Drive>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAC_MESSAGES__MSG__DETAIL__DRIVE__TRAITS_HPP_
