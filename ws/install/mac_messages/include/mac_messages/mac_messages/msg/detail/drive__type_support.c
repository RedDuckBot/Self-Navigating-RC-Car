// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mac_messages:msg/Drive.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mac_messages/msg/detail/drive__rosidl_typesupport_introspection_c.h"
#include "mac_messages/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mac_messages/msg/detail/drive__functions.h"
#include "mac_messages/msg/detail/drive__struct.h"


// Include directives for member types
// Member `command`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mac_messages__msg__Drive__init(message_memory);
}

void mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_fini_function(void * message_memory)
{
  mac_messages__msg__Drive__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_message_member_array[2] = {
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mac_messages__msg__Drive, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "control_input",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mac_messages__msg__Drive, control_input),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_message_members = {
  "mac_messages__msg",  // message namespace
  "Drive",  // message name
  2,  // number of fields
  sizeof(mac_messages__msg__Drive),
  mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_message_member_array,  // message members
  mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_init_function,  // function to initialize message memory (memory has to be allocated)
  mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_message_type_support_handle = {
  0,
  &mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_message_members,
  get_message_typesupport_handle_function,
  &mac_messages__msg__Drive__get_type_hash,
  &mac_messages__msg__Drive__get_type_description,
  &mac_messages__msg__Drive__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mac_messages
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mac_messages, msg, Drive)() {
  if (!mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_message_type_support_handle.typesupport_identifier) {
    mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mac_messages__msg__Drive__rosidl_typesupport_introspection_c__Drive_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
