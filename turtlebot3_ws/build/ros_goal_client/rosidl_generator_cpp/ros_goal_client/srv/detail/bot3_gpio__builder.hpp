// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_goal_client:srv/Bot3Gpio.idl
// generated code does not contain a copyright notice

#ifndef ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__BUILDER_HPP_
#define ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_goal_client/srv/detail/bot3_gpio__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_goal_client
{

namespace srv
{

namespace builder
{

class Init_Bot3Gpio_Request_b
{
public:
  explicit Init_Bot3Gpio_Request_b(::ros_goal_client::srv::Bot3Gpio_Request & msg)
  : msg_(msg)
  {}
  ::ros_goal_client::srv::Bot3Gpio_Request b(::ros_goal_client::srv::Bot3Gpio_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_goal_client::srv::Bot3Gpio_Request msg_;
};

class Init_Bot3Gpio_Request_a
{
public:
  Init_Bot3Gpio_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Bot3Gpio_Request_b a(::ros_goal_client::srv::Bot3Gpio_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_Bot3Gpio_Request_b(msg_);
  }

private:
  ::ros_goal_client::srv::Bot3Gpio_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_goal_client::srv::Bot3Gpio_Request>()
{
  return ros_goal_client::srv::builder::Init_Bot3Gpio_Request_a();
}

}  // namespace ros_goal_client


namespace ros_goal_client
{

namespace srv
{

namespace builder
{

class Init_Bot3Gpio_Response_result
{
public:
  Init_Bot3Gpio_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros_goal_client::srv::Bot3Gpio_Response result(::ros_goal_client::srv::Bot3Gpio_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_goal_client::srv::Bot3Gpio_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_goal_client::srv::Bot3Gpio_Response>()
{
  return ros_goal_client::srv::builder::Init_Bot3Gpio_Response_result();
}

}  // namespace ros_goal_client

#endif  // ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__BUILDER_HPP_
