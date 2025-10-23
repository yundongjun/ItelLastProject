// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef ROS_TUTORIALS_TOPIC__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define ROS_TUTORIALS_TOPIC__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_ros_tutorials_topic __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_ros_tutorials_topic __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_ros_tutorials_topic __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_ros_tutorials_topic __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_ros_tutorials_topic
    #define ROSIDL_GENERATOR_CPP_PUBLIC_ros_tutorials_topic ROSIDL_GENERATOR_CPP_EXPORT_ros_tutorials_topic
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_ros_tutorials_topic ROSIDL_GENERATOR_CPP_IMPORT_ros_tutorials_topic
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_ros_tutorials_topic __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_ros_tutorials_topic
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_ros_tutorials_topic __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_ros_tutorials_topic
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROS_TUTORIALS_TOPIC__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
