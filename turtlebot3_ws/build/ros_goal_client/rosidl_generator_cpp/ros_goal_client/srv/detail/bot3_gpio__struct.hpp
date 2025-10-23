// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_goal_client:srv/Bot3Gpio.idl
// generated code does not contain a copyright notice

#ifndef ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__STRUCT_HPP_
#define ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros_goal_client__srv__Bot3Gpio_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros_goal_client__srv__Bot3Gpio_Request __declspec(deprecated)
#endif

namespace ros_goal_client
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Bot3Gpio_Request_
{
  using Type = Bot3Gpio_Request_<ContainerAllocator>;

  explicit Bot3Gpio_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0ll;
      this->b = 0ll;
    }
  }

  explicit Bot3Gpio_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0ll;
      this->b = 0ll;
    }
  }

  // field types and members
  using _a_type =
    int64_t;
  _a_type a;
  using _b_type =
    int64_t;
  _b_type b;

  // setters for named parameter idiom
  Type & set__a(
    const int64_t & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__b(
    const int64_t & _arg)
  {
    this->b = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_goal_client__srv__Bot3Gpio_Request
    std::shared_ptr<ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_goal_client__srv__Bot3Gpio_Request
    std::shared_ptr<ros_goal_client::srv::Bot3Gpio_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Bot3Gpio_Request_ & other) const
  {
    if (this->a != other.a) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    return true;
  }
  bool operator!=(const Bot3Gpio_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Bot3Gpio_Request_

// alias to use template instance with default allocator
using Bot3Gpio_Request =
  ros_goal_client::srv::Bot3Gpio_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros_goal_client


#ifndef _WIN32
# define DEPRECATED__ros_goal_client__srv__Bot3Gpio_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros_goal_client__srv__Bot3Gpio_Response __declspec(deprecated)
#endif

namespace ros_goal_client
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Bot3Gpio_Response_
{
  using Type = Bot3Gpio_Response_<ContainerAllocator>;

  explicit Bot3Gpio_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0ll;
    }
  }

  explicit Bot3Gpio_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0ll;
    }
  }

  // field types and members
  using _result_type =
    int64_t;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const int64_t & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_goal_client__srv__Bot3Gpio_Response
    std::shared_ptr<ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_goal_client__srv__Bot3Gpio_Response
    std::shared_ptr<ros_goal_client::srv::Bot3Gpio_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Bot3Gpio_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Bot3Gpio_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Bot3Gpio_Response_

// alias to use template instance with default allocator
using Bot3Gpio_Response =
  ros_goal_client::srv::Bot3Gpio_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros_goal_client

namespace ros_goal_client
{

namespace srv
{

struct Bot3Gpio
{
  using Request = ros_goal_client::srv::Bot3Gpio_Request;
  using Response = ros_goal_client::srv::Bot3Gpio_Response;
};

}  // namespace srv

}  // namespace ros_goal_client

#endif  // ROS_GOAL_CLIENT__SRV__DETAIL__BOT3_GPIO__STRUCT_HPP_
