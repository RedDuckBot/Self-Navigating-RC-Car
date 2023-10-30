// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mac_messages:msg/Drive.idl
// generated code does not contain a copyright notice

#include "mac_messages/msg/detail/drive__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mac_messages
const rosidl_type_hash_t *
mac_messages__msg__Drive__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0x7b, 0x9f, 0x5a, 0xdb, 0xaa, 0x5f, 0xad,
      0x65, 0x26, 0x07, 0x3d, 0x5f, 0x2e, 0xec, 0x53,
      0xa9, 0xfc, 0x48, 0x0e, 0xec, 0x3a, 0xc2, 0x21,
      0x3f, 0x66, 0x79, 0x5b, 0x3e, 0xc3, 0xc3, 0xc8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mac_messages__msg__Drive__TYPE_NAME[] = "mac_messages/msg/Drive";

// Define type names, field names, and default values
static char mac_messages__msg__Drive__FIELD_NAME__command[] = "command";
static char mac_messages__msg__Drive__FIELD_NAME__control_input[] = "control_input";

static rosidl_runtime_c__type_description__Field mac_messages__msg__Drive__FIELDS[] = {
  {
    {mac_messages__msg__Drive__FIELD_NAME__command, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mac_messages__msg__Drive__FIELD_NAME__control_input, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mac_messages__msg__Drive__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mac_messages__msg__Drive__TYPE_NAME, 22, 22},
      {mac_messages__msg__Drive__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#Command values: F --> Drive forward\n"
  "#                B --> Drive backward\n"
  "#                S --> Control steering\n"
  "#                E --> Enable drive mode \n"
  "#                D --> Disable drive mode\n"
  "string command \n"
  "\n"
  "#Associated with speed or angle\n"
  "int16 control_input";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mac_messages__msg__Drive__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mac_messages__msg__Drive__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 267, 267},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mac_messages__msg__Drive__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mac_messages__msg__Drive__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
