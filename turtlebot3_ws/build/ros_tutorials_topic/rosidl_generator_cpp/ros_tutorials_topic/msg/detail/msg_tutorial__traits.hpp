// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_tutorials_topic:msg/MsgTutorial.idl
// generated code does not contain a copyright notice

#ifndef ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__TRAITS_HPP_
#define ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_tutorials_topic/msg/detail/msg_tutorial__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace ros_tutorials_topic
{

namespace msg
{

inline void to_flow_style_yaml(
  const MsgTutorial & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MsgTutorial & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MsgTutorial & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ros_tutorials_topic

namespace rosidl_generator_traits
{

[[deprecated("use ros_tutorials_topic::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros_tutorials_topic::msg::MsgTutorial & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_tutorials_topic::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_tutorials_topic::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_tutorials_topic::msg::MsgTutorial & msg)
{
  return ros_tutorials_topic::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_tutorials_topic::msg::MsgTutorial>()
{
  return "ros_tutorials_topic::msg::MsgTutorial";
}

template<>
inline const char * name<ros_tutorials_topic::msg::MsgTutorial>()
{
  return "ros_tutorials_topic/msg/MsgTutorial";
}

template<>
struct has_fixed_size<ros_tutorials_topic::msg::MsgTutorial>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<ros_tutorials_topic::msg::MsgTutorial>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<ros_tutorials_topic::msg::MsgTutorial>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__TRAITS_HPP_
