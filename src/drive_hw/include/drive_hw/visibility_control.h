#ifndef DRIVE_HW__VISIBILITY_CONTROL_H_
#define DRIVE_HW__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DRIVE_HW_EXPORT __attribute__ ((dllexport))
    #define DRIVE_HW_IMPORT __attribute__ ((dllimport))
  #else
    #define DRIVE_HW_EXPORT __declspec(dllexport)
    #define DRIVE_HW_IMPORT __declspec(dllimport)
  #endif
  #ifdef DRIVE_HW_BUILDING_LIBRARY
    #define DRIVE_HW_PUBLIC DRIVE_HW_EXPORT
  #else
    #define DRIVE_HW_PUBLIC DRIVE_HW_IMPORT
  #endif
  #define DRIVE_HW_PUBLIC_TYPE DRIVE_HW_PUBLIC
  #define DRIVE_HW_LOCAL
#else
  #define DRIVE_HW_EXPORT __attribute__ ((visibility("default")))
  #define DRIVE_HW_IMPORT
  #if __GNUC__ >= 4
    #define DRIVE_HW_PUBLIC __attribute__ ((visibility("default")))
    #define DRIVE_HW_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DRIVE_HW_PUBLIC
    #define DRIVE_HW_LOCAL
  #endif
  #define DRIVE_HW_PUBLIC_TYPE
#endif

#endif  // DRIVE_HW__VISIBILITY_CONTROL_H_
