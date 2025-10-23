// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_goal_client:srv/Bot3Gpio.idl
// generated code does not contain a copyright notice

#ifndef ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__STRUCT_H_
#define ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Bot3Gpio in the package ros_goal_client.
typedef struct ros_goal_client__srv__Bot3Gpio_Request
{
  int64_t a;
  int64_t b;
} ros_goal_client__srv__Bot3Gpio_Request;

// Struct for a sequence of ros_goal_client__srv__Bot3Gpio_Request.
typedef struct ros_goal_client__srv__Bot3Gpio_Request__Sequence
{
  ros_goal_client__srv__Bot3Gpio_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_goal_client__srv__Bot3Gpio_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Bot3Gpio in the package ros_goal_client.
typedef struct ros_goal_client__srv__Bot3Gpio_Response
{
  int64_t result;
} ros_goal_client__srv__Bot3Gpio_Response;

// Struct for a sequence of ros_goal_client__srv__Bot3Gpio_Response.
typedef struct ros_goal_client__srv__Bot3Gpio_Response__Sequence
{
  ros_goal_client__srv__Bot3Gpio_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_goal_client__srv__Bot3Gpio_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__STRUCT_H_
