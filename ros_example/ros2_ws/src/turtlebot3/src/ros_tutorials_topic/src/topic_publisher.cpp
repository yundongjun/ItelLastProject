#include <rclcpp/rclcpp.hpp>                          // ROS2 C++ 헤더
#include "ros_tutorials_topic/msg/msg_tutorial.hpp"   // 생성된 커스텀 메시지 헤더

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("topic_publisher");   // 노드명 동일

  // 퍼블리셔: 토픽명 "ros_tutorial_msg", 큐 사이즈 100
  auto pub = node->create_publisher<ros_tutorials_topic::msg::MsgTutorial>(
      "ros_tutorial_msg", 100);

  rclcpp::WallRate loop_rate(10);   // 10 Hz (0.1초 주기)
  ros_tutorials_topic::msg::MsgTutorial msg;
  int count = 0;

  while (rclcpp::ok()) {
    // ROS2에서는 stamp 타입이 builtin_interfaces/Time 이므로 now()로 채움
    msg.stamp = node->get_clock()->now();
    msg.data  = count;

    pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "pub: stamp=%u.%u, data=%d",
                msg.stamp.sec, msg.stamp.nanosec, msg.data);

    rclcpp::spin_some(node);
    loop_rate.sleep();
    ++count;
  }

  rclcpp::shutdown();
  return 0;
}

