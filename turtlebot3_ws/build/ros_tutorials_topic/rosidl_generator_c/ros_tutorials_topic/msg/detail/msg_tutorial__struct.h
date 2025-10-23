// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_tutorials_topic:msg/MsgTutorial.idl
// generated code does not contain a copyright notice

#ifndef ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__STRUCT_H_
#define ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/MsgTutorial in the package ros_tutorials_topic.
typedef struct ros_tutorials_topic__msg__MsgTutorial
{
  builtin_interfaces__msg__Time stamp;
  int32_t data;
} ros_tutorials_topic__msg__MsgTutorial;

// Struct for a sequence of ros_tutorials_topic__msg__MsgTutorial.
typedef struct ros_tutorials_topic__msg__MsgTutorial__Sequence
{
  ros_tutorials_topic__msg__MsgTutorial * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_tutorials_topic__msg__MsgTutorial__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__STRUCT_H_
