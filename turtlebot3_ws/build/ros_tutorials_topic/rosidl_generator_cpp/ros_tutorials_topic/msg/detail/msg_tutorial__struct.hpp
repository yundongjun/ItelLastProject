// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_tutorials_topic:msg/MsgTutorial.idl
// generated code does not contain a copyright notice

#ifndef ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__STRUCT_HPP_
#define ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_tutorials_topic__msg__MsgTutorial __attribute__((deprecated))
#else
# define DEPRECATED__ros_tutorials_topic__msg__MsgTutorial __declspec(deprecated)
#endif

namespace ros_tutorials_topic
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MsgTutorial_
{
  using Type = MsgTutorial_<ContainerAllocator>;

  explicit MsgTutorial_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0l;
    }
  }

  explicit MsgTutorial_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0l;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _data_type =
    int32_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__data(
    const int32_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_tutorials_topic__msg__MsgTutorial
    std::shared_ptr<ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_tutorials_topic__msg__MsgTutorial
    std::shared_ptr<ros_tutorials_topic::msg::MsgTutorial_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MsgTutorial_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const MsgTutorial_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MsgTutorial_

// alias to use template instance with default allocator
using MsgTutorial =
  ros_tutorials_topic::msg::MsgTutorial_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_tutorials_topic

#endif  // ROS_TUTORIALS_TOPIC__MSG__DETAIL__MSG_TUTORIAL__STRUCT_HPP_
