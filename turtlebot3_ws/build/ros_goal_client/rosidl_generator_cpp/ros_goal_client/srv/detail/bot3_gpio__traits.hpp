// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_goal_client:srv/Bot3Gpio.idl
// generated code does not contain a copyright notice

#ifndef ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__TRAITS_HPP_
#define ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_goal_client/srv/detail/bot3_gpio__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros_goal_client
{

namespace srv
{

inline void to_flow_style_yaml(
  const Bot3Gpio_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Bot3Gpio_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Bot3Gpio_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros_goal_client

namespace rosidl_generator_traits
{

[[deprecated("use ros_goal_client::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros_goal_client::srv::Bot3Gpio_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_goal_client::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_goal_client::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros_goal_client::srv::Bot3Gpio_Request & msg)
{
  return ros_goal_client::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros_goal_client::srv::Bot3Gpio_Request>()
{
  return "ros_goal_client::srv::Bot3Gpio_Request";
}

template<>
inline const char * name<ros_goal_client::srv::Bot3Gpio_Request>()
{
  return "ros_goal_client/srv/Bot3Gpio_Request";
}

template<>
struct has_fixed_size<ros_goal_client::srv::Bot3Gpio_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros_goal_client::srv::Bot3Gpio_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros_goal_client::srv::Bot3Gpio_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ros_goal_client
{

namespace srv
{

inline void to_flow_style_yaml(
  const Bot3Gpio_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Bot3Gpio_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Bot3Gpio_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros_goal_client

namespace rosidl_generator_traits
{

[[deprecated("use ros_goal_client::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros_goal_client::srv::Bot3Gpio_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_goal_client::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_goal_client::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros_goal_client::srv::Bot3Gpio_Response & msg)
{
  return ros_goal_client::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros_goal_client::srv::Bot3Gpio_Response>()
{
  return "ros_goal_client::srv::Bot3Gpio_Response";
}

template<>
inline const char * name<ros_goal_client::srv::Bot3Gpio_Response>()
{
  return "ros_goal_client/srv/Bot3Gpio_Response";
}

template<>
struct has_fixed_size<ros_goal_client::srv::Bot3Gpio_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros_goal_client::srv::Bot3Gpio_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros_goal_client::srv::Bot3Gpio_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros_goal_client::srv::Bot3Gpio>()
{
  return "ros_goal_client::srv::Bot3Gpio";
}

template<>
inline const char * name<ros_goal_client::srv::Bot3Gpio>()
{
  return "ros_goal_client/srv/Bot3Gpio";
}

template<>
struct has_fixed_size<ros_goal_client::srv::Bot3Gpio>
  : std::integral_constant<
    bool,
    has_fixed_size<ros_goal_client::srv::Bot3Gpio_Request>::value &&
    has_fixed_size<ros_goal_client::srv::Bot3Gpio_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros_goal_client::srv::Bot3Gpio>
  : std::integral_constant<
    bool,
    has_bounded_size<ros_goal_client::srv::Bot3Gpio_Request>::value &&
    has_bounded_size<ros_goal_client::srv::Bot3Gpio_Response>::value
  >
{
};

template<>
struct is_service<ros_goal_client::srv::Bot3Gpio>
  : std::true_type
{
};

template<>
struct is_service_request<ros_goal_client::srv::Bot3Gpio_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros_goal_client::srv::Bot3Gpio_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__TRAITS_HPP_
