#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "tfm_landmark_based_localization_package::tfm_landmark_based_localization_package__rosidl_typesupport_c" for configuration "Debug"
set_property(TARGET tfm_landmark_based_localization_package::tfm_landmark_based_localization_package__rosidl_typesupport_c APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(tfm_landmark_based_localization_package::tfm_landmark_based_localization_package__rosidl_typesupport_c PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_DEBUG "rosidl_runtime_c::rosidl_runtime_c;rosidl_typesupport_c::rosidl_typesupport_c"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libtfm_landmark_based_localization_package__rosidl_typesupport_c.so"
  IMPORTED_SONAME_DEBUG "libtfm_landmark_based_localization_package__rosidl_typesupport_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS tfm_landmark_based_localization_package::tfm_landmark_based_localization_package__rosidl_typesupport_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_tfm_landmark_based_localization_package::tfm_landmark_based_localization_package__rosidl_typesupport_c "${_IMPORT_PREFIX}/lib/libtfm_landmark_based_localization_package__rosidl_typesupport_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)