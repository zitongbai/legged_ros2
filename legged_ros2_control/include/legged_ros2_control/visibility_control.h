#ifndef LEGGED_ROS2_CONTROL__VISIBILITY_CONTROL_H_
#define LEGGED_ROS2_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LEGGED_ROS2_CONTROL_EXPORT __attribute__ ((dllexport))
    #define LEGGED_ROS2_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define LEGGED_ROS2_CONTROL_EXPORT __declspec(dllexport)
    #define LEGGED_ROS2_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef LEGGED_ROS2_CONTROL_BUILDING_LIBRARY
    #define LEGGED_ROS2_CONTROL_PUBLIC LEGGED_ROS2_CONTROL_EXPORT
  #else
    #define LEGGED_ROS2_CONTROL_PUBLIC LEGGED_ROS2_CONTROL_IMPORT
  #endif
  #define LEGGED_ROS2_CONTROL_PUBLIC_TYPE LEGGED_ROS2_CONTROL_PUBLIC
  #define LEGGED_ROS2_CONTROL_LOCAL
#else
  #define LEGGED_ROS2_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define LEGGED_ROS2_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define LEGGED_ROS2_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define LEGGED_ROS2_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LEGGED_ROS2_CONTROL_PUBLIC
    #define LEGGED_ROS2_CONTROL_LOCAL
  #endif
  #define LEGGED_ROS2_CONTROL_PUBLIC_TYPE
#endif

#endif  // LEGGED_ROS2_CONTROL__VISIBILITY_CONTROL_H_
