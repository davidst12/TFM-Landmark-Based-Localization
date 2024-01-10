// generated from
// rosidl_typesupport_fastrtps_c/resource/rosidl_typesupport_fastrtps_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef TFM_LANDMARK_BASED_LOCALIZATION_PACKAGE__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_
#define TFM_LANDMARK_BASED_LOCALIZATION_PACKAGE__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_tfm_landmark_based_localization_package __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tfm_landmark_based_localization_package __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_tfm_landmark_based_localization_package __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tfm_landmark_based_localization_package __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_FASTRTPS_C_BUILDING_DLL_tfm_landmark_based_localization_package
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tfm_landmark_based_localization_package ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_tfm_landmark_based_localization_package
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tfm_landmark_based_localization_package ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tfm_landmark_based_localization_package
  #endif
#else
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_tfm_landmark_based_localization_package __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tfm_landmark_based_localization_package
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tfm_landmark_based_localization_package __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tfm_landmark_based_localization_package
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // TFM_LANDMARK_BASED_LOCALIZATION_PACKAGE__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_
