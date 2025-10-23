#include <rclcpp/rclcpp.hpp>
#include "ros_tutorials_topic/msg/msg_tutorial.hpp"   // 커스텀 메시지

class TopicSubscriber : public rclcpp::Node {
public:
  TopicSubscriber() : rclcpp::Node("topic_subscriber") {
    sub_ = this->create_subscription<ros_tutorials_topic::msg::MsgTutorial>(
      "ros_tutorial_msg",            // 토픽명
      100,                           // 큐 사이즈
      [this](const ros_tutorials_topic::msg::MsgTutorial & msg) {
        // stamp(sec, nanosec)와 data 출력
        RCLCPP_INFO(
          this->get_logger(),
          "recv msg: stamp.sec=%d, stamp.nanosec=%u, data=%d",
          static_cast<int>(msg.stamp.sec),
          static_cast<unsigned int>(msg.stamp.nanosec),
          static_cast<int>(msg.data)
        );
      }
    );
  }

private:
  rclcpp::Subscription<ros_tutorials_topic::msg::MsgTutorial>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicSubscriber>());
  rclcpp::shutdown();
  return 0;
}

