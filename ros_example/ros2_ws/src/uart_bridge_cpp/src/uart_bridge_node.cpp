#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>

class UartBridgeNode : public rclcpp::Node {
public:
  UartBridgeNode() : Node("uart_bridge_node"), fd_(-1), stop_(false) {
    // Params
    port_      = this->declare_parameter<std::string>("port", "/dev/serial0");
    baud_      = this->declare_parameter<int>("baud", 115200);
    newline_   = this->declare_parameter<std::string>("newline", "\n"); // "\n" or "\r\n"
    rx_topic_  = this->declare_parameter<std::string>("rx_topic", "/uart/rx");
    tx_topic_  = this->declare_parameter<std::string>("tx_topic", "/uart/tx");
    poll_ms_   = this->declare_parameter<int>("poll_ms", 50);
    rx_chunk_  = this->declare_parameter<int>("rx_chunk", 512);
    log_bytes_ = this->declare_parameter<bool>("log_bytes", false);
    log_lines_ = this->declare_parameter<bool>("log_lines", true);

    // Pub/Sub
    pub_rx_ = this->create_publisher<std_msgs::msg::String>(rx_topic_, 10);
    sub_tx_ = this->create_subscription<std_msgs::msg::String>(
      tx_topic_, 10,
      std::bind(&UartBridgeNode::on_tx_msg, this, std::placeholders::_1));

    // Open serial
    fd_ = open_serial(port_.c_str(), baud_);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s (baud=%d): %s",
                   port_.c_str(), baud_, std::strerror(errno));
      throw std::runtime_error("serial open failed");
    }
    RCLCPP_INFO(get_logger(), "Opened %s @ %d", port_.c_str(), baud_);

    // Reader thread
    rx_thread_ = std::thread(&UartBridgeNode::rx_loop, this);

    // Shutdown hook
    rclcpp::on_shutdown([this]() { this->graceful_stop(); });
  }

  ~UartBridgeNode() override { graceful_stop(); }

private:
  static std::string sanitize_printable(const std::string &s, size_t max_len = 200) {
    std::string out;
    out.reserve(std::min(s.size(), max_len));
    for (char c : s) {
      if (c >= 32 && c <= 126) out.push_back(c);
      else if (c == '\r')      out += "\\r";
      else if (c == '\n')      out += "\\n";
      else {
        std::ostringstream oss;
        oss << "\\x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << (static_cast<int>(static_cast<unsigned char>(c)));
        out += oss.str();
      }
      if (out.size() >= max_len) { out += "..."; break; }
    }
    return out;
  }

  static std::string hex_dump(const uint8_t* data, size_t len, size_t max_len = 256) {
    std::ostringstream oss;
    size_t n = std::min(len, max_len);
    for (size_t i = 0; i < n; ++i) {
      if (i) oss << ' ';
      oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
          << static_cast<int>(data[i]);
    }
    if (len > max_len) oss << " ...";
    return oss.str();
  }

  static speed_t to_speed_t(int baud) {
    switch (baud) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
#ifdef B460800
      case 460800: return B460800;
#endif
#ifdef B921600
      case 921600: return B921600;
#endif
      default: return B115200;
    }
  }

  int open_serial(const char* dev, int baud) {
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
#ifdef TIOCEXCL
    ioctl(fd, TIOCEXCL);
#endif
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tio{};
    if (tcgetattr(fd, &tio) != 0) { ::close(fd); return -1; }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB; tio.c_cflag &= ~CSTOPB; tio.c_cflag &= ~CSIZE; tio.c_cflag |= CS8;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cflag &= ~CRTSCTS;

    speed_t spd = to_speed_t(baud);
    cfsetispeed(&tio, spd); cfsetospeed(&tio, spd);
    tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) { ::close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
  }

  void on_tx_msg(const std_msgs::msg::String::SharedPtr msg) {
    if (fd_ < 0) return;
    std::string data = msg->data;
    if (!newline_.empty()) {
      if (newline_ == "\n") { if (data.empty() || data.back() != '\n') data += "\n"; }
      else if (newline_ == "\r\n") { if (data.size() < 2 || data.substr(data.size()-2) != "\r\n") data += "\r\n"; }
    }
    if (log_lines_) RCLCPP_INFO(get_logger(), "[TX line] \"%s\"", sanitize_printable(data).c_str());
    if (log_bytes_) {
      const auto* p = reinterpret_cast<const uint8_t*>(data.data());
      RCLCPP_DEBUG(get_logger(), "[TX bytes] %zu bytes: %s", data.size(), hex_dump(p, data.size()).c_str());
    }
    std::lock_guard<std::mutex> lk(tx_mtx_);
    ssize_t n = ::write(fd_, data.data(), data.size());
    if (n < 0) RCLCPP_WARN(get_logger(), "UART write error: %s", std::strerror(errno));
  }

  void rx_loop() {
    std::string linebuf;
    while (!stop_.load()) {
      if (fd_ < 0) break;

      fd_set rfds; FD_ZERO(&rfds); FD_SET(fd_, &rfds);
      struct timeval tv{0, static_cast<suseconds_t>(poll_ms_ * 1000)};
      int rv = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
      if (rv < 0) { if (errno == EINTR) continue; if (stop_.load()) break; RCLCPP_WARN(get_logger(),"select: %s", std::strerror(errno)); continue; }
      if (rv == 0) continue;

      std::vector<uint8_t> tmp(rx_chunk_);
      ssize_t n = ::read(fd_, tmp.data(), tmp.size());
      if (n < 0) { if (errno == EAGAIN || errno == EINTR) continue; if (stop_.load()) break; RCLCPP_WARN(get_logger(),"read: %s", std::strerror(errno)); continue; }
      tmp.resize(n);

      if (n > 0 && log_bytes_) RCLCPP_DEBUG(get_logger(), "[RX bytes] %zd bytes: %s", n, hex_dump(tmp.data(), tmp.size()).c_str());

      for (uint8_t c : tmp) {
        if (c == '\n') {
          if (!linebuf.empty() && linebuf.back() == '\r') linebuf.pop_back();
          if (log_lines_) RCLCPP_INFO(get_logger(), "[RX line] \"%s\"", sanitize_printable(linebuf).c_str());
          std_msgs::msg::String msg; msg.data = linebuf; pub_rx_->publish(msg);
          linebuf.clear();
        } else {
          linebuf.push_back(static_cast<char>(c));
        }
      }
    }
    RCLCPP_INFO(get_logger(), "RX thread stopped");
  }

  void graceful_stop() {
    if (stop_.exchange(true)) return;
    RCLCPP_INFO(get_logger(), "Stopping uart_bridge_node...");
    { std::lock_guard<std::mutex> lk(tx_mtx_);
      if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    if (rx_thread_.joinable()) rx_thread_.join();
    RCLCPP_INFO(get_logger(), "Closed");
  }

private:
  // params
  std::string port_;
  int baud_;
  std::string newline_;
  std::string rx_topic_, tx_topic_;
  int poll_ms_;
  int rx_chunk_;
  bool log_bytes_;
  bool log_lines_;

  // serial
  int fd_;
  std::mutex tx_mtx_;

  // threading
  std::atomic<bool> stop_;
  std::thread rx_thread_;

  // ROS
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rx_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_tx_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<UartBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

