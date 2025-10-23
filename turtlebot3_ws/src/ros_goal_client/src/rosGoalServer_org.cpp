#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <signal.h>
#include <cerrno>
#include <atomic>

#include <memory>
#include <chrono>
#include <thread>
#include <string>
#include <algorithm>
#include <sstream>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "ros_goal_client/srv/bot3_gpio.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>   // /fork/cmd : UInt8
#include <std_msgs/msg/bool.hpp>     // /fork/busy : Bool (선택)


#define NODE_NAME   "ros_goal_server"
#define SRV_NAME    "bot3_gpio"

#define BUF_SIZE    100
#define NAME_SIZE   20
#define ARR_CNT     10
#define GOALCNT     2

#define DEFAULT_SPEED 0.22

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

void * send_msg(void * arg);
void * recv_msg(void * arg);
void error_handling(const char *msg);

// ===== 글로벌 상태 =====
static rclcpp::Node::SharedPtr g_node = nullptr;
static std::atomic_bool g_run{true};
static std::atomic<int>  g_sock_fd{-1};
static std::atomic<int>  g_listen_fd{-1};
static std::atomic<bool> g_fork_done{false};
static std::atomic<bool> g_stm_up_done{false};

// jetson 퍼블리셔
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_uart_tx_pub;
// jetson 구독
static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_uart_rx_sub;

// /cmd_vel 퍼블리셔
static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr g_cmdvel_pub;

// STM32 TX 퍼블리셔
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_stm_tx_pub;
// STM32 RX 구독
static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_stm_rx_sub;



// 포크리프트 제어용 퍼블리셔
// static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_fork_cmd_pub;

// 포크리프트 제어용 서브스크립션
// static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_fork_state_sub;

// 포크리프트 제어용 퍼블리셔(1=UP, 2=DOWN, 0=STOP)
static rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr g_fork_cmd_pub;

// 포크리프트 상태 구독(/fork/busy: Bool) — 선택 사항
static rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr g_fork_busy_sub;

// ── 맵 메타데이터 ─────────────────────────────────────────────
struct MapMeta {
  std::atomic<bool> ready{false};
  double resolution{0.0};
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
};
static MapMeta g_map;
static rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr g_map_sub;

static geometry_msgs::msg::PoseStamped g_home_pose;
static geometry_msgs::msg::PoseStamped g_second_pose;
static std::atomic<bool> g_home_ready{false};
static std::atomic<bool> g_second_ready{false};
static rclcpp_action::Client<NavigateToPose>::SharedPtr g_nav_client;

static void go_to_pose(const geometry_msgs::msg::PoseStamped &pose, bool send_down_after = false);

static void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  g_map.resolution = msg->info.resolution;
  g_map.width      = static_cast<int>(msg->info.width);
  g_map.height     = static_cast<int>(msg->info.height);
  g_map.origin_x   = msg->info.origin.position.x;
  g_map.origin_y   = msg->info.origin.position.y;
  g_map.ready.store(true);

  RCLCPP_INFO(g_node->get_logger(),
              "[MAP] res=%.4f m/px, size=%dx%d, origin=(%.3f, %.3f)",
              g_map.resolution, g_map.width, g_map.height, g_map.origin_x, g_map.origin_y);
}

// 픽셀 → 맵 좌표
static inline bool pixel_to_map(double u, double v, double &x, double &y, bool top_left_origin=true)
{
  if (!g_map.ready.load()) return false;
  if (g_map.resolution <= 0.0 || g_map.width <= 0 || g_map.height <= 0) return false;

  const double res = g_map.resolution;
  const double cx  = g_map.origin_x;
  const double cy  = g_map.origin_y;

  x = cx + (u + 0.5) * res;
  if (top_left_origin) {
    y = cy + ((g_map.height - 1 - v) + 0.5) * res;
  } else {
    y = cy + (v + 0.5) * res;
  }
  return true;
}

// SIGINT 핸들러
static void sigint_handler(int)
{
  g_run = false;
  int fd = g_sock_fd.exchange(-1);
  if (fd != -1) { ::shutdown(fd, SHUT_RDWR); ::close(fd); }
  int lfd = g_listen_fd.exchange(-1);
  if (lfd != -1) { ::shutdown(lfd, SHUT_RDWR); ::close(lfd); }
}

static inline void install_sigint_handler()
{
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = sigint_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT,  &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

template<typename FutureT>
static rclcpp::FutureReturnCode spin_until_complete_interruptible(
    const rclcpp::Node::SharedPtr& node,
    FutureT & future,
    std::chrono::milliseconds step = 200ms,
    std::chrono::milliseconds max_total = 0ms)
{
  (void)node;
  auto start = std::chrono::steady_clock::now();
  while (g_run.load() && rclcpp::ok()) {
    auto status = future.wait_for(step);
    if (status == std::future_status::ready) {
      return rclcpp::FutureReturnCode::SUCCESS;
    }
    if (status != std::future_status::timeout) {
      return rclcpp::FutureReturnCode::INTERRUPTED;
    }
    if (max_total.count() > 0 &&
        std::chrono::steady_clock::now() - start >= max_total) {
      return rclcpp::FutureReturnCode::TIMEOUT;
    }
  }
  return rclcpp::FutureReturnCode::INTERRUPTED;
}

// ====== 행동 파라미터 & 헬퍼 ======
static std::string g_cmd_vel_topic = "/cmd_vel";
static double g_turn_speed = 0.6;     // rad/s
static int    g_pulse_ms   = 400;     // ms

static inline void publish_twist(double lin_x, double ang_z) {
  if (!g_cmdvel_pub) return;
  geometry_msgs::msg::Twist t;
  t.linear.x  = lin_x;
  t.angular.z = ang_z;
  g_cmdvel_pub->publish(t);
}

static void pulse_turn(double ang_z, int duration_ms) {
  std::thread([ang_z, duration_ms](){
    using namespace std::chrono;
    auto end = steady_clock::now() + milliseconds(duration_ms);
    rclcpp::Rate r(50.0);
    while (g_run.load() && rclcpp::ok() && steady_clock::now() < end) {
      publish_twist(0.0, ang_z);
      r.sleep();
    }
    publish_twist(0.0, 0.0);
  }).detach();
}

// /uart/rx 콜백: "1" → 좌, "2" → 우
static void on_uart_rx(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string s = msg->data; // 그대로 비교 (양끝 공백 무시할 수도 있음)
  auto trim = [](std::string t){
    t.erase(t.begin(), std::find_if(t.begin(), t.end(), [](int ch){return !std::isspace(ch);}));
    t.erase(std::find_if(t.rbegin(), t.rend(), [](int ch){return !std::isspace(ch);}).base(), t.end());
    return t;
  };
  std::string v = trim(s);

  bool turned = false;

  if (v == "1") {
    RCLCPP_INFO(g_node->get_logger(), "[UART-RX] '1' → TURN LEFT (+%.3f rad/s, %d ms)", g_turn_speed, g_pulse_ms);
    pulse_turn(+g_turn_speed, g_pulse_ms);
    turned = true;
  } else if (v == "2") {
    RCLCPP_INFO(g_node->get_logger(), "[UART-RX] '2' → TURN RIGHT (-%.3f rad/s, %d ms)", g_turn_speed, g_pulse_ms);
    pulse_turn(-g_turn_speed, g_pulse_ms);
    turned = true;
  } else {
    RCLCPP_DEBUG(g_node->get_logger(), "[UART-RX] ignored: '%s'", v.c_str());
  }

  // STM32로 UP 명령 전송
  if (turned && g_stm_tx_pub) {
    std_msgs::msg::String cmd;
    cmd.data = "UP";                           // STM 프로토콜에 맞춰 수정
    g_stm_tx_pub->publish(cmd);
    RCLCPP_INFO(g_node->get_logger(), "[STM-TX] Sent UP");
  }
}




// /stm32/rx 콜백: "UP_DONE" 수신 시 g_stm_up_done = true
static void on_stm_rx(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string payload = msg->data;
  RCLCPP_INFO(g_node->get_logger(), "[STM-RX] %s", payload.c_str());

  if (payload == "STOP1") {
    g_stm_up_done.store(true);
    if (g_home_ready.load()) {
      RCLCPP_INFO(g_node->get_logger(), "[STM] Fork UP complete → return home");
      go_to_pose(g_home_pose, true);        // 귀환 후 go_to_pose 안에서 DOWN 명령 송신
    }
  } else if (payload == "STOP2") {
     if (g_second_ready.load()) {
      RCLCPP_INFO(g_node->get_logger(), "[STM] Fork DOWN complete → moving to second goal");
      go_to_pose(g_second_pose);          // 두번째 목표로 이동
    } else {
      RCLCPP_WARN(g_node->get_logger(), "[STM] second goal not ready");
    }
  }
}

// 네비게이션 목표 위치로 이동(초기 위치)
static void go_to_pose(const geometry_msgs::msg::PoseStamped &pose, bool send_down_after)
{
  if (!g_nav_client) return;
  if (!g_nav_client->wait_for_action_server(1s)) {
    RCLCPP_WARN(g_node->get_logger(), "[NAV] action server not ready");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = pose;
  goal.pose.header.stamp = g_node->now();

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
  auto gh_future = g_nav_client->async_send_goal(goal, opts);
  auto gh_rc = spin_until_complete_interruptible(g_node, gh_future, 200ms, 10s);
  if (gh_rc != rclcpp::FutureReturnCode::SUCCESS) return;
  auto gh = gh_future.get();
  if (!gh) return;

  auto res_future = g_nav_client->async_get_result(gh);
  auto res_rc = spin_until_complete_interruptible(g_node, res_future, 200ms, 600s);
  if (res_rc == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = res_future.get();
    RCLCPP_INFO(g_node->get_logger(), "[NAV] return home result=%d", (int)result.code);

    if (send_down_after && result.code == rclcpp_action::ResultCode::SUCCEEDED && g_stm_tx_pub) {
      std_msgs::msg::String cmd;
      cmd.data = "DOWN";                       // STM과 약속한 프로토콜
      g_stm_tx_pub->publish(cmd);
      RCLCPP_INFO(g_node->get_logger(), "[STM-TX] DOWN command sent");
    }
  }
}


int main(int argc, char *argv[])
{
  if (argc != 2) {
    fprintf(stderr, "Usage : ros2 run ros_goal_client rosGoalServer <port>\n");
    return 1;
  }

    // ROS 초기화
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared(NODE_NAME);
  rclcpp::on_shutdown([]{ g_run = false; });

  // 파라미터
  g_cmd_vel_topic  = g_node->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  g_turn_speed     = g_node->declare_parameter<double>("turn_speed", 0.6);
  g_pulse_ms       = g_node->declare_parameter<int>("pulse_ms", 400);

  // 네비게이션 액션 클라이언트 및 홈 포즈 초기화
  g_nav_client = rclcpp_action::create_client<NavigateToPose>(g_node, "navigate_to_pose");

  g_home_pose.header.frame_id = "map";
  g_home_pose.pose.position.x = 0.0;
  g_home_pose.pose.position.y = 0.0;
  g_home_pose.pose.position.z = 0.0;
  g_home_pose.pose.orientation.x = 0.0;
  g_home_pose.pose.orientation.y = 0.0;
  g_home_pose.pose.orientation.z = 0.0;
  g_home_pose.pose.orientation.w = 1.0;
  g_home_ready.store(true);
  g_second_ready.store(false);

  // 퍼블리셔/구독
  g_cmdvel_pub  = g_node->create_publisher<geometry_msgs::msg::Twist>(g_cmd_vel_topic, 10);
  //jetson pub/sub 초기화
  g_uart_tx_pub = g_node->create_publisher<std_msgs::msg::String>("/uart/tx", 10);
  g_uart_rx_sub = g_node->create_subscription<std_msgs::msg::String>("/uart/rx", 10, &on_uart_rx);
  // STM32 pub/sub 초기화
  g_stm_tx_pub = g_node->create_publisher<std_msgs::msg::String>("/stm32/tx", 10);
  g_stm_rx_sub = g_node->create_subscription<std_msgs::msg::String>("/stm32/rx", 10, &on_stm_rx);

  // g_fork_cmd_pub = g_node->create_publisher<std_msgs::msg::String>("/fork/cmd",10);
  // g_fork_cmd_sub = g_node->create_subscription<std_msgs::msg::String>("/fork/state",10);

  g_fork_cmd_pub = g_node->create_publisher<std_msgs::msg::UInt8>("/fork/cmd", 10);

// (선택) busy 상태 구독: true면 동작 중
g_fork_busy_sub = g_node->create_subscription<std_msgs::msg::Bool>(
  "/fork/busy", 10,
  [](const std_msgs::msg::Bool::SharedPtr m){
    RCLCPP_INFO(g_node->get_logger(), "[FORK/BUSY] %s", m->data ? "BUSY" : "IDLE");
  }
);

 

  // 맵 구독
  g_map_sub = g_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable(), map_cb);

  // 콜백을 위한 executor 스레드
  std::atomic<bool> spin_alive{true};
  std::thread spin_thread([&](){
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(g_node);
    using namespace std::chrono_literals;
    while (spin_alive.load() && g_run.load() && rclcpp::ok()) {
      exec.spin_some(50ms);
    }
  });

  // TCP 서버
  install_sigint_handler();

  int listen_fd = socket(PF_INET, SOCK_STREAM, 0);
  if (listen_fd == -1) { perror("socket"); return 1; }
  g_listen_fd = listen_fd;

  int yes = 1;
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in srv{};
  srv.sin_family = AF_INET;
  srv.sin_addr.s_addr = htonl(INADDR_ANY);
  srv.sin_port = htons(atoi(argv[1]));

  if (bind(listen_fd, (struct sockaddr*)&srv, sizeof(srv)) == -1) {
    perror("bind");
    close(listen_fd);
    g_listen_fd = -1;
    return 1;
  }
  if (listen(listen_fd, 5) == -1) {
    perror("listen");
    close(listen_fd);
    g_listen_fd = -1;
    return 1;
  }
  printf("[server] listening on 0.0.0.0:%s ...\n", argv[1]);

  int sock = -1;
  while (g_run.load()) {
    fd_set rfds; FD_ZERO(&rfds); FD_SET(listen_fd, &rfds);
    struct timeval tv{1,0};
    int ret = select(listen_fd+1, &rfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) continue;
      perror("select");
      break;
    }
    if (ret == 0) continue;
    if (FD_ISSET(listen_fd, &rfds)) {
      struct sockaddr_in cli{}; socklen_t clen = sizeof(cli);
      sock = accept(listen_fd, (struct sockaddr*)&cli, &clen);
      if (sock == -1) {
        if (errno == EINTR) continue;
        perror("accept");
        break;
      }
      char ipbuf[64]; inet_ntop(AF_INET, &cli.sin_addr, ipbuf, sizeof(ipbuf));
      printf("[server] client connected from %s:%d\n", ipbuf, ntohs(cli.sin_port));
      break;
    }
  }

  if (sock == -1) {
    int lfd = g_listen_fd.exchange(-1);
    if (lfd != -1) { ::shutdown(lfd, SHUT_RDWR); ::close(lfd); }
    return 0;
  }

  int lfd = g_listen_fd.exchange(-1);
  if (lfd != -1) { ::shutdown(lfd, SHUT_RDWR); ::close(lfd); }

  g_sock_fd = sock;
  timeval tv{1, 0};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));


  // 워커 스레드
  pthread_t snd_thread, rcv_thread;
  void *thread_return;
  pthread_create(&rcv_thread, NULL, recv_msg, (void *)&sock);
  pthread_create(&snd_thread, NULL, send_msg, (void *)&sock);

  pthread_join(snd_thread, &thread_return);
  pthread_join(rcv_thread, &thread_return);

  int fd = g_sock_fd.exchange(-1);
  if (fd != -1) { ::shutdown(fd, SHUT_RDWR); ::close(fd); }

  spin_alive = false;
  if (spin_thread.joinable()) spin_thread.join();

  g_map_sub.reset();
  g_uart_rx_sub.reset();
  g_cmdvel_pub.reset();
  g_uart_tx_pub.reset();
  g_node.reset();
  g_fork_cmd_pub.reset();
  g_fork_busy_sub.reset();


  rclcpp::shutdown();
  return 0;
}

// ===== 기존 보조 루틴들 =====
void *send_msg(void *arg)
{
  int *sock = (int *)arg;
  int ret;
  fd_set initset, newset;
  struct timeval tv;
  char msg[BUF_SIZE];
  char name_msg[NAME_SIZE + BUF_SIZE + 2];

  FD_ZERO(&initset);
  FD_SET(STDIN_FILENO, &initset);

  fputs("Input a message! [ID]msg (Default ID:ALLMSG)\n", stdout);
  while (g_run.load() && rclcpp::ok()) {
    memset(msg, 0, sizeof(msg));
    name_msg[0] = '\0';
    tv.tv_sec = 1; tv.tv_usec = 0;
    newset = initset;
    ret = select(STDIN_FILENO + 1, &newset, NULL, NULL, &tv);

    if (!g_run.load() || !rclcpp::ok() || *sock == -1) return NULL;

    if (ret < 0) {
      if (errno == EINTR) continue;
      *sock = -1; return NULL;
    }

    if (ret > 0 && FD_ISSET(STDIN_FILENO, &newset)) {
      if (fgets(msg, BUF_SIZE, stdin) == NULL) continue;

      if (!strncmp(msg, "quit\n", 5)) { *sock = -1; return NULL; }
      else if (msg[0] != '[') { strcat(name_msg, "[ALLMSG]"); strcat(name_msg, msg); }
      else { strcpy(name_msg, msg); }

      if (write(*sock, name_msg, strlen(name_msg)) <= 0) { *sock = -1; return NULL; }
    }
  }
  return NULL;
}

void *recv_msg(void *arg)
{
  int *sock = (int *)arg;
  int i;
  char *pToken;
  char *pArray[ARR_CNT] = {0};

  char name_msg[NAME_SIZE + BUF_SIZE + 1];
  char raw_msg [NAME_SIZE + BUF_SIZE + 1];
  int str_len;

  auto srv_client = g_node->create_client<ros_goal_client::srv::Bot3Gpio>(SRV_NAME);
  auto srv_req = std::make_shared<ros_goal_client::srv::Bot3Gpio::Request>();
  srv_req->a = 0; srv_req->b = 1;

  auto ac = rclcpp_action::create_client<NavigateToPose>(g_node, "navigate_to_pose");
  while (g_run.load() && rclcpp::ok() && !ac->wait_for_action_server(500ms)) {
    RCLCPP_INFO(g_node->get_logger(), "Waiting for the Nav2 action server to come up");
  }
  if (!g_run.load() || !rclcpp::ok()) return NULL;
  RCLCPP_INFO(g_node->get_logger(), "[READY] Nav2 action server is UP");

  auto speed_pub = g_node->create_publisher<nav2_msgs::msg::SpeedLimit>("speed_limit", 10);
  NavigateToPose::Goal goal; goal.pose.header.frame_id = "map";

  while (g_run.load() && rclcpp::ok()) {
    memset(name_msg, 0x0, sizeof(name_msg));
    str_len = read(*sock, name_msg, NAME_SIZE + BUF_SIZE);

    if (str_len < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
      *sock = -1; return NULL;
    }
    if (str_len == 0) { *sock = -1 ; return NULL; }

    if (name_msg[str_len - 1] == '\n' || name_msg[str_len - 1] == '\r')
      name_msg[str_len - 1] = 0;

    RCLCPP_INFO(g_node->get_logger(), "[NET][raw] '%s'", name_msg);

    strncpy(raw_msg, name_msg, sizeof(raw_msg));
    raw_msg[sizeof(raw_msg)-1] = '\0';

    { // 4 숫자(hx,hy,fx,fy) 처리 (필요 시 유지)
      double vals[4]; int cnt = 0; char *p = raw_msg;
      while (*p && cnt < 4) {
        char *endp; double v = strtod(p, &endp);
        if (endp != p) { vals[cnt++] = v; p = endp; } else { ++p; }
      }
      if (cnt == 4) {
        double hx = vals[0], hy = vals[1], fx = vals[2], fy = vals[3];

        auto looks_like_pixels = [&](double a, double b){ return (a > 20.0 || b > 20.0); };
        if (!g_map.ready.load() && (looks_like_pixels(hx,hy) || looks_like_pixels(fx,fy))) {
          for (int k=0; k<10 && rclcpp::ok() && !g_map.ready.load(); ++k) {
            std::this_thread::sleep_for(100ms);
          }
        }

        bool treat_as_pixels = false;
        if (g_map.ready.load()) {
          if ((hx >= 0 && hy >= 0 && hx <= g_map.width-1 && hy <= g_map.height-1) ||
              (fx >= 0 && fy >= 0 && fx <= g_map.width-1 && fy <= g_map.height-1)) {
            treat_as_pixels = true;
          }
        }

        double tx = hx, ty = hy;
        if (treat_as_pixels) {
          double convx, convy;
          if (pixel_to_map(hx, hy, convx, convy, true)) { tx = convx; ty = convy; }
        }
        double sx = fx, sy = fy;
        if (treat_as_pixels) {
          if (!pixel_to_map(fx, fy, sx, sy, true)) {
            RCLCPP_WARN(g_node->get_logger(),
                        "[SECOND] pixel_to_map failed for (%.3f, %.3f)", fx, fy);
            continue;  // 변환 실패 시 다음 메시지 처리
          }
        }
        g_second_pose.header.frame_id = "map";
        g_second_pose.pose.position.x = sx;
        g_second_pose.pose.position.y = sy;
        g_second_pose.pose.position.z = 0.0;
        g_second_pose.pose.orientation.x = 0.0;
        g_second_pose.pose.orientation.y = 0.0;
        g_second_pose.pose.orientation.z = 0.0;
        g_second_pose.pose.orientation.w = 1.0;
        g_second_ready.store(true);

        nav2_msgs::msg::SpeedLimit lim; lim.percentage = false; lim.speed_limit = DEFAULT_SPEED;
        speed_pub->publish(lim);

        goal.pose.header.stamp = g_node->now();
        goal.pose.pose.position.x = tx;
        goal.pose.pose.position.y = ty;
        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;
        goal.pose.pose.orientation.z = 0.0;
        goal.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
        auto gh_future = ac->async_send_goal(goal, send_goal_options);
        auto gh_rc = spin_until_complete_interruptible(g_node, gh_future, 200ms, 10s);
        if (gh_rc != rclcpp::FutureReturnCode::SUCCESS) { continue; }
        auto goal_handle = gh_future.get();
        if (!goal_handle) { continue; }

        auto result_future = ac->async_get_result(goal_handle);
        auto res_rc = spin_until_complete_interruptible(g_node, result_future, 200ms, 3000s);
        if (res_rc != rclcpp::FutureReturnCode::SUCCESS) { continue; }
        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          if (g_uart_tx_pub) {
            std_msgs::msg::String sig; sig.data = "1";
            g_uart_tx_pub->publish(sig);
          }
        }
        continue;
      }
    }

    // (기존 호환 프로토콜 필요 시 유지)
    for (int k = 0; k < ARR_CNT; ++k) pArray[k] = nullptr;
    pToken = strtok(name_msg, "[:@]");
    i = 0;
    while (pToken != NULL) {
      pArray[i] = pToken;
      if (i++ >= ARR_CNT) break;
      pToken = strtok(NULL, "[:@]");
    }

    if (pArray[1] && !strcmp(pArray[1], "GOGOAL") && (i == 5)) {
      double gx = strtod(pArray[2], NULL);
      double gy = strtod(pArray[3], NULL);
      double gw = strtod(pArray[4], NULL);

      goal.pose.header.stamp = g_node->now();
      goal.pose.pose.position.x = gx;
      goal.pose.pose.position.y = gy;
      goal.pose.pose.orientation.x = 0.0;
      goal.pose.pose.orientation.y = 0.0;
      goal.pose.pose.orientation.z = 0.0;
      goal.pose.pose.orientation.w = gw;

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
      auto gh_future = ac->async_send_goal(goal, send_goal_options);
      auto gh_rc = spin_until_complete_interruptible(g_node, gh_future, 200ms, 10s);
      if (gh_rc != rclcpp::FutureReturnCode::SUCCESS) { continue; }
      auto goal_handle = gh_future.get();
      if (!goal_handle) { continue; }

      auto result_future = ac->async_get_result(goal_handle);
      auto res_rc = spin_until_complete_interruptible(g_node, result_future, 200ms, 10s);
      (void)res_rc;
      (void)result_future;
    }
  }
  return NULL;
}

void error_handling(const char *msg)
{
  fputs(msg, stderr);
  fputc('\n', stderr);
  exit(1);
}

