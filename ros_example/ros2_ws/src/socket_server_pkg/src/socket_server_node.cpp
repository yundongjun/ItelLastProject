#include "socket_server_pkg/socket_server_node.hpp"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cerrno>
#include <iostream>

SocketBridgeNode::SocketBridgeNode()
: rclcpp::Node("socket_bridge_node")
{
  // Declare & get params
  port_          = this->declare_parameter<int>("port", 5000);
  topic_in_      = this->declare_parameter<std::string>("topic_in", "/socket_in");
  topic_out_     = this->declare_parameter<std::string>("topic_out", "/socket_out");
  backlog_       = this->declare_parameter<int>("backlog", 8);
  nodelay_       = this->declare_parameter<bool>("tcp_nodelay", true);
  recv_buf_size_ = this->declare_parameter<int>("recv_buffer", 1<<20); // 1MB
  send_buf_size_ = this->declare_parameter<int>("send_buffer", 1<<20); // 1MB

  pub_in_ = this->create_publisher<std_msgs::msg::String>(topic_in_, 10);

  sub_out_ = this->create_subscription<std_msgs::msg::String>(
      topic_out_, 10,
      [this](const std_msgs::msg::String::SharedPtr msg){
        // Broadcast ROS→TCP
        broadcast_line(msg->data + "\n");
      });

  start_server();
  RCLCPP_INFO(this->get_logger(), "SocketBridgeNode started on port %d | in: %s | out: %s",
              port_, topic_in_.c_str(), topic_out_.c_str());
}

SocketBridgeNode::~SocketBridgeNode() {
  stop_server();
}

bool SocketBridgeNode::set_socket_opts(int fd, bool nodelay, int rcvbuf, int sndbuf) {
  int flag = nodelay ? 1 : 0;
  if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) < 0) return false;
  if (rcvbuf > 0) setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  if (sndbuf > 0) setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
  return true;
}

void SocketBridgeNode::start_server() {
  server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    RCLCPP_FATAL(this->get_logger(), "socket() failed: %s", std::strerror(errno));
    throw std::runtime_error("socket failed");
  }

  int yes = 1;
  setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
  setsockopt(server_fd_, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(static_cast<uint16_t>(port_));

  if (bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
    RCLCPP_FATAL(this->get_logger(), "bind() failed: %s", std::strerror(errno));
    ::close(server_fd_);
    server_fd_ = -1;
    throw std::runtime_error("bind failed");
  }

  if (listen(server_fd_, backlog_) < 0) {
    RCLCPP_FATAL(this->get_logger(), "listen() failed: %s", std::strerror(errno));
    ::close(server_fd_);
    server_fd_ = -1;
    throw std::runtime_error("listen failed");
  }

  running_.store(true);
  accept_thread_ = std::thread(&SocketBridgeNode::accept_loop, this);
}

void SocketBridgeNode::stop_server() {
  running_.store(false);
  if (server_fd_ >= 0) {
    ::shutdown(server_fd_, SHUT_RDWR);
    ::close(server_fd_);
    server_fd_ = -1;
  }
  if (accept_thread_.joinable()) accept_thread_.join();

  std::lock_guard<std::mutex> lk(clients_mtx_);
  for (int fd : clients_) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
  clients_.clear();
}

void SocketBridgeNode::accept_loop() {
  RCLCPP_INFO(this->get_logger(), "Accept loop running...");
  while (running_.load()) {
    sockaddr_in caddr{};
    socklen_t clen = sizeof(caddr);
    int cfd = ::accept(server_fd_, (sockaddr*)&caddr, &clen);
    if (cfd < 0) {
      if (errno == EINTR) continue;
      if (!running_.load()) break;
      // EAGAIN or other transient errors
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    set_socket_opts(cfd, nodelay_, recv_buf_size_, send_buf_size_);

    {
      std::lock_guard<std::mutex> lk(clients_mtx_);
      clients_.push_back(cfd);
    }

    char ip[64]; std::snprintf(ip, sizeof(ip), "%s", inet_ntoa(caddr.sin_addr));
    RCLCPP_INFO(this->get_logger(), "Client connected: %s:%d (fd=%d)", ip, ntohs(caddr.sin_port), cfd);

    // per-client thread (detached)
    std::thread(&SocketBridgeNode::client_loop, this, cfd).detach();
  }
  RCLCPP_INFO(this->get_logger(), "Accept loop stopped.");
}

void SocketBridgeNode::client_loop(int client_fd) {
  std::string buffer;
  buffer.reserve(4096);
  constexpr size_t TMP_SZ = 2048;
  char tmp[TMP_SZ];

  while (running_.load()) {
    ssize_t n = ::recv(client_fd, tmp, TMP_SZ, 0);
    if (n == 0) break;              // graceful close
    if (n < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      break; // fatal
    }

    buffer.append(tmp, tmp + n);

    // process per line
    size_t pos;
    while ((pos = buffer.find('\n')) != std::string::npos) {
      std::string line = buffer.substr(0, pos);
      // trim CR
      if (!line.empty() && line.back() == '\r') line.pop_back();

      // Publish to ROS
      auto msg = std_msgs::msg::String();
      msg.data = line;
      pub_in_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "📥 Received from client(fd=%d): %s", client_fd, line.c_str());

      buffer.erase(0, pos + 1);
    }
  }

  // remove from list
  {
    std::lock_guard<std::mutex> lk(clients_mtx_);
    auto it = std::find(clients_.begin(), clients_.end(), client_fd);
    if (it != clients_.end()) clients_.erase(it);
  }
  ::shutdown(client_fd, SHUT_RDWR);
  ::close(client_fd);
  RCLCPP_INFO(this->get_logger(), "Client disconnected (fd=%d)", client_fd);
}

void SocketBridgeNode::broadcast_line(const std::string &line) {
  std::lock_guard<std::mutex> lk(clients_mtx_);
  for (auto it = clients_.begin(); it != clients_.end();) {
    int fd = *it;
    ssize_t n = ::send(fd, line.data(), line.size(), MSG_NOSIGNAL);
    if (n < 0) {
      // drop dead client
      ::shutdown(fd, SHUT_RDWR);
      ::close(fd);
      it = clients_.erase(it);
      continue;
    }
    ++it;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SocketBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

