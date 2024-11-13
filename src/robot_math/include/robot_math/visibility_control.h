#ifndef ROBOT_MATH__VISIBILITY_CONTROL_H_
#define ROBOT_MATH__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_MATH_EXPORT __attribute__ ((dllexport))
    #define ROBOT_MATH_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_MATH_EXPORT __declspec(dllexport)
    #define ROBOT_MATH_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_MATH_BUILDING_LIBRARY
    #define ROBOT_MATH_PUBLIC ROBOT_MATH_EXPORT
  #else
    #define ROBOT_MATH_PUBLIC ROBOT_MATH_IMPORT
  #endif
  #define ROBOT_MATH_PUBLIC_TYPE ROBOT_MATH_PUBLIC
  #define ROBOT_MATH_LOCAL
#else
  #define ROBOT_MATH_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_MATH_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_MATH_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_MATH_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_MATH_PUBLIC
    #define ROBOT_MATH_LOCAL
  #endif
  #define ROBOT_MATH_PUBLIC_TYPE
#endif

#endif  // ROBOT_MATH__VISIBILITY_CONTROL_H_
