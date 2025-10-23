// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_tutorials_topic:msg/MsgTutorial.idl
// generated code does not contain a copyright notice

#ifndef ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__BUILDER_HPP_
#define ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_tutorials_topic/msg/detail/msg_tutorial__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_tutorials_topic
{

namespace msg
{

namespace builder
{

class Init_MsgTutorial_data
{
public:
  explicit Init_MsgTutorial_data(::ros_tutorials_topic::msg::MsgTutorial & msg)
  : msg_(msg)
  {}
  ::ros_tutorials_topic::msg::MsgTutorial data(::ros_tutorials_topic::msg::MsgTutorial::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_tutorials_topic::msg::MsgTutorial msg_;
};

class Init_MsgTutorial_stamp
{
public:
  Init_MsgTutorial_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MsgTutorial_data stamp(::ros_tutorials_topic::msg::MsgTutorial::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_MsgTutorial_data(msg_);
  }

private:
  ::ros_tutorials_topic::msg::MsgTutorial msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_tutorials_topic::msg::MsgTutorial>()
{
  return ros_tutorials_topic::msg::builder::Init_MsgTutorial_stamp();
}

}  // namespace ros_tutorials_topic

#endif  // ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__BUILDER_HPP_
