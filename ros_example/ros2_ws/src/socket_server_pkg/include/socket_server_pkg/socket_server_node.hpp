#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <string>

class SocketBridgeNode : public rclcpp::Node {
public:
  SocketBridgeNode();
  ~SocketBridgeNode();

private:
  // ROS
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_in_;   // to ROS (/socket_in)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_out_; // from ROS (/socket_out)

  // Params
  int port_;
  std::string topic_in_;
  std::string topic_out_;
  int backlog_;
  bool nodelay_;
  int recv_buf_size_;
  int send_buf_size_;

  // TCP
  int server_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread accept_thread_;
  std::mutex clients_mtx_;
  std::vector<int> clients_; // client sockets

  // Methods
  void start_server();
  void stop_server();
  void accept_loop();
  void client_loop(int client_fd);
  void broadcast_line(const std::string &line);
  static bool set_socket_opts(int fd, bool nodelay, int rcvbuf, int sndbuf);
};

