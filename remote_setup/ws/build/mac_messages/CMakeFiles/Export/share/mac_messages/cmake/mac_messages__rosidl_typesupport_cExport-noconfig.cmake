#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mac_messages::mac_messages__rosidl_typesupport_c" for configuration ""
set_property(TARGET mac_messages::mac_messages__rosidl_typesupport_c APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(mac_messages::mac_messages__rosidl_typesupport_c PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "rosidl_runtime_c::rosidl_runtime_c;rosidl_typesupport_c::rosidl_typesupport_c"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmac_messages__rosidl_typesupport_c.so"
  IMPORTED_SONAME_NOCONFIG "libmac_messages__rosidl_typesupport_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS mac_messages::mac_messages__rosidl_typesupport_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_mac_messages::mac_messages__rosidl_typesupport_c "${_IMPORT_PREFIX}/lib/libmac_messages__rosidl_typesupport_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
