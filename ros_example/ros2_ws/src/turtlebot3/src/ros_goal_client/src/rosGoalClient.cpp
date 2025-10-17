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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include "ros_goal_client/srv/bot3_gpio.hpp"

#define NODE_NAME   "ros_goal_client"
#define SRV_NAME    "bot3_gpio"

#define BUF_SIZE    100
#define NAME_SIZE   20
#define ARR_CNT     10
#define GOALCNT     2

#define DEFAULT_SPEED 0.1

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using MoveBaseClient = rclcpp_action::Client<NavigateToPose>;

void * send_msg(void * arg);
void * recv_msg(void * arg);
void error_handling(const char *msg);

// ===== 글로벌 상태 =====
static rclcpp::Node::SharedPtr g_node = nullptr;
static std::atomic_bool g_run{true};
static std::atomic<int>  g_sock_fd{-1};

// SIGINT 핸들러: 즉시 루프 종료 + 소켓 끊기
static void sigint_handler(int)
{
  g_run = false;
  int fd = g_sock_fd.exchange(-1);
  if (fd != -1) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
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

// rclcpp future를 짧은 타임아웃으로 폴링하여 Ctrl+C 즉시 탈출
template<typename FutureT>
static rclcpp::FutureReturnCode spin_until_complete_interruptible(
    const rclcpp::Node::SharedPtr& node,
    FutureT & future,
    std::chrono::milliseconds step = 200ms,
    std::chrono::milliseconds max_total = 0ms /*0이면 무제한*/)
{
  auto start = std::chrono::steady_clock::now();
  while (g_run.load() && rclcpp::ok()) {
    auto rc = rclcpp::spin_until_future_complete(node, future, step);
    if (rc != rclcpp::FutureReturnCode::TIMEOUT) return rc;
    if (max_total.count() > 0 &&
        std::chrono::steady_clock::now() - start >= max_total) {
      return rclcpp::FutureReturnCode::TIMEOUT;
    }
  }
  return rclcpp::FutureReturnCode::INTERRUPTED;
}

int main(int argc, char *argv[])
{
  // 인자: <IP> <port> (name 없음)
  if (argc != 3) {
    fprintf(stderr, "Usage : ros2 run ros_goal_client rosGoalClient <IP> <port>\n");
    return 1;
  }

  install_sigint_handler();

  int sock = -1;
  struct sockaddr_in serv_addr;
  pthread_t snd_thread, rcv_thread;
  void * thread_return;

  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared(NODE_NAME);
  rclcpp::on_shutdown([]{ g_run = false; });

  sock = socket(PF_INET, SOCK_STREAM, 0);
  if (sock == -1) error_handling("socket() error");
  g_sock_fd = sock;

  // 소켓 타임아웃(블로킹 방지)
  timeval tv{1, 0};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
  serv_addr.sin_port = htons(atoi(argv[2]));

  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
    error_handling("connect() error");

  // name 없이 패스워드 핸드셰이크
  char first_msg[BUF_SIZE];
  snprintf(first_msg, sizeof(first_msg), "[PASSWD]");
  (void)write(sock, first_msg, strlen(first_msg));

  pthread_create(&rcv_thread, NULL, recv_msg, (void *)&sock);
  pthread_create(&snd_thread, NULL, send_msg, (void *)&sock);

  pthread_join(snd_thread, &thread_return);
  pthread_join(rcv_thread, &thread_return);

  int fd = g_sock_fd.exchange(-1);
  if (fd != -1) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }

  rclcpp::shutdown();
  return 0;
}

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
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    newset = initset;
    ret = select(STDIN_FILENO + 1, &newset, NULL, NULL, &tv);

    if (!g_run.load() || !rclcpp::ok() || *sock == -1) return NULL;

    if (ret < 0) {
      if (errno == EINTR) continue; // 신호로 깸
      *sock = -1; return NULL;
    }

    if (ret > 0 && FD_ISSET(STDIN_FILENO, &newset)) {
      if (fgets(msg, BUF_SIZE, stdin) == NULL) continue;

      if (!strncmp(msg, "quit\n", 5)) {
        *sock = -1;
        return NULL;
      } else if (msg[0] != '[') {
        strcat(name_msg, "[ALLMSG]");
        strcat(name_msg, msg);
      } else {
        strcpy(name_msg, msg);
      }
      if (write(*sock, name_msg, strlen(name_msg)) <= 0) {
        *sock = -1;
        return NULL;
      }
    }
  }
  return NULL;
}

void *recv_msg(void *arg)
{
  double dSeqVal[GOALCNT][3] = {
    {2.69, 0.07, 0.99},
    {0.0,  0.0,  0.86},
  };

  int *sock = (int *)arg;
  int i;
  char *pToken;
  char *pArray[ARR_CNT] = {0};

  char name_msg[NAME_SIZE + BUF_SIZE + 1];
  char raw_msg [NAME_SIZE + BUF_SIZE + 1];
  int str_len;

  // --- ROS2: 서비스 클라이언트 ---
  auto srv_client = g_node->create_client<ros_goal_client::srv::Bot3Gpio>(SRV_NAME);
  auto srv_req = std::make_shared<ros_goal_client::srv::Bot3Gpio::Request>();
  srv_req->a = 0;
  srv_req->b = 1;

  // --- Nav2 액션 클라이언트 ---
  auto ac = rclcpp_action::create_client<NavigateToPose>(g_node, "navigate_to_pose");
  while (g_run.load() && rclcpp::ok() && !ac->wait_for_action_server(500ms)) {
    RCLCPP_INFO(g_node->get_logger(), "Waiting for the Nav2 action server to come up");
  }
  if (!g_run.load() || !rclcpp::ok()) return NULL;

  // ★ 속도 제한 퍼블리셔
  auto speed_pub = g_node->create_publisher<nav2_msgs::msg::SpeedLimit>("speed_limit", 10);

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";

  while (g_run.load() && rclcpp::ok()) {
    memset(name_msg, 0x0, sizeof(name_msg));
    str_len = read(*sock, name_msg, NAME_SIZE + BUF_SIZE);

    if (str_len < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue; // 타임아웃/신호
      }
      *sock = -1;
      return NULL;
    }
    if (str_len == 0) {
      *sock = -1; // 소켓 종료
      return NULL;
    }

    name_msg[str_len - 1] = 0;   // 줄바꿈 제거 가정
    fputs(name_msg, stdout);

    // 원문 보존본 생성 (토크나이즈 전에)
    strncpy(raw_msg, name_msg, sizeof(raw_msg));
    raw_msg[sizeof(raw_msg)-1] = '\0';

    // ─────────────────────────────────────────────────────────────
    // 숫자 4개(hx,hy,fx,fy) 어디에 있어도 추출 → 사람 좌표로 이동
    // ─────────────────────────────────────────────────────────────
    {
      double vals[4]; int cnt = 0;
      char *p = raw_msg;
      while (*p && cnt < 4) {
        char *endp;
        double v = strtod(p, &endp);
        if (endp != p) { vals[cnt++] = v; p = endp; }
        else { ++p; }
      }
      if (cnt == 4) {
        double hx = vals[0], hy = vals[1];

        nav2_msgs::msg::SpeedLimit lim;
        lim.percentage = false;            // 절대속도 모드
        lim.speed_limit = DEFAULT_SPEED;   // m/s
        speed_pub->publish(lim);

        // 사람 좌표로 이동
        goal.pose.header.stamp = g_node->now();
        goal.pose.pose.position.x = hx;
        goal.pose.pose.position.y = hy;
        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;
        goal.pose.pose.orientation.z = 0.0;
        goal.pose.pose.orientation.w = 1.0; // yaw=0

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
        auto gh_future = ac->async_send_goal(goal, send_goal_options);
        auto gh_rc = spin_until_complete_interruptible(g_node, gh_future, 200ms, 10s);
        if (gh_rc != rclcpp::FutureReturnCode::SUCCESS) {
          if (!g_run.load() || !rclcpp::ok()) return NULL;
          RCLCPP_INFO(g_node->get_logger(), "Failed to send goal (human)");
          continue;
        }
        auto goal_handle = gh_future.get();
        if (!goal_handle) {
          RCLCPP_INFO(g_node->get_logger(), "Goal was rejected by server (human)");
          continue;
        }
        auto result_future = ac->async_get_result(goal_handle);
        auto res_rc = spin_until_complete_interruptible(g_node, result_future, 200ms, 20s);
        if (res_rc != rclcpp::FutureReturnCode::SUCCESS) {
          if (!g_run.load() || !rclcpp::ok()) return NULL;
          RCLCPP_INFO(g_node->get_logger(), "Get result timeout/failure (human)");
          continue;
        }
        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
          RCLCPP_INFO(g_node->get_logger(), "Arrived at human (%.3f, %.3f)", hx, hy);
        else
          RCLCPP_INFO(g_node->get_logger(), "Failed to reach human for some reason");

        continue;
      }
    }

    // ─────────────────────────────────────────────────────────────
    // (호환) 기존 프로토콜: [ID]:GPIO@ON/OFF, [ID]:GOGOAL@x@y@w, [ID]:TURTLE
    // ─────────────────────────────────────────────────────────────
    for (int k = 0; k < ARR_CNT; ++k) pArray[k] = nullptr;
    pToken = strtok(name_msg, "[:@]");
    i = 0;
    while (pToken != NULL) {
      pArray[i] = pToken;
      if (i++ >= ARR_CNT) break;
      pToken = strtok(NULL, "[:@]");
    }

    if (pArray[1] && !strcmp(pArray[1], "GPIO")) {
      if (!srv_client->wait_for_service(1s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Service '%s' not available", SRV_NAME);
      } else {
        if (!strcmp(pArray[2], "ON")) srv_req->a = 0;
        else                          srv_req->a = 1;

        auto future = srv_client->async_send_request(srv_req);
        auto rc = spin_until_complete_interruptible(g_node, future, 200ms, 2s);
        if (rc == rclcpp::FutureReturnCode::SUCCESS) {
          auto resp = future.get();
          RCLCPP_INFO(g_node->get_logger(),
                      "send srv, req.a : %ld, recieve srv, resp.result : %ld",
                      (long) srv_req->a, (long) resp->result);
        } else if (!g_run.load() || !rclcpp::ok()) {
          return NULL;
        } else {
          RCLCPP_ERROR(g_node->get_logger(), "Failed to call service");
        }
      }
    }
    else if (pArray[1] && !strcmp(pArray[1], "GOGOAL") && (i == 5)) {
      goal.pose.header.stamp = g_node->now();

      goal.pose.pose.position.x = strtod(pArray[2], NULL);
      goal.pose.pose.position.y = strtod(pArray[3], NULL);
      goal.pose.pose.orientation.x = 0.0;
      goal.pose.pose.orientation.y = 0.0;
      goal.pose.pose.orientation.z = 0.0;
      goal.pose.pose.orientation.w = strtod(pArray[4], NULL);

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
      auto gh_future = ac->async_send_goal(goal, send_goal_options);
      auto gh_rc = spin_until_complete_interruptible(g_node, gh_future, 200ms, 10s);
      if (gh_rc != rclcpp::FutureReturnCode::SUCCESS) {
        if (!g_run.load() || !rclcpp::ok()) return NULL;
        RCLCPP_INFO(g_node->get_logger(), "Failed to send goal");
        continue;
      }
      auto goal_handle = gh_future.get();
      if (!goal_handle) {
        RCLCPP_INFO(g_node->get_logger(), "Goal was rejected by server");
        continue;
      }
      auto result_future = ac->async_get_result(goal_handle);
      auto res_rc = spin_until_complete_interruptible(g_node, result_future, 200ms, 10s);
      if (res_rc != rclcpp::FutureReturnCode::SUCCESS) {
        if (!g_run.load() || !rclcpp::ok()) return NULL;
        RCLCPP_INFO(g_node->get_logger(), "Get result timeout/failure");
        continue;
      }
      auto result = result_future.get();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(g_node->get_logger(), "Goal arrived!");
      else
        RCLCPP_INFO(g_node->get_logger(), "The base failed to move to goal for some reason");
    }
    else if (pArray[1] && !strcmp(pArray[1], "TURTLE")) {
      for (int j = 0; j < GOALCNT; j++) {
        goal.pose.header.stamp = g_node->now();
        goal.pose.pose.position.x = dSeqVal[j][0];
        goal.pose.pose.position.y = dSeqVal[j][1];
        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;
        goal.pose.pose.orientation.z = 0.0;
        goal.pose.pose.orientation.w = dSeqVal[j][2];

        RCLCPP_INFO(g_node->get_logger(), "Sending goal Seq%d", j+1);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
        auto gh_future = ac->async_send_goal(goal, send_goal_options);
        auto gh_rc = spin_until_complete_interruptible(g_node, gh_future, 200ms, 20s);
        if (gh_rc != rclcpp::FutureReturnCode::SUCCESS) {
          if (!g_run.load() || !rclcpp::ok()) return NULL;
          RCLCPP_INFO(g_node->get_logger(), "Failed to send goal");
          continue;
        }
        auto goal_handle = gh_future.get();
        if (!goal_handle) {
          RCLCPP_INFO(g_node->get_logger(), "Goal was rejected by server");
          continue;
        }
        auto result_future = ac->async_get_result(goal_handle);
        auto res_rc = spin_until_complete_interruptible(g_node, result_future, 200ms, 20s);
        if (res_rc != rclcpp::FutureReturnCode::SUCCESS) {
          if (!g_run.load() || !rclcpp::ok()) return NULL;
          RCLCPP_INFO(g_node->get_logger(), "Get result timeout/failure");
          continue;
        }
        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
          RCLCPP_INFO(g_node->get_logger(), "Goal Seq%d done!", j+1);
        else
          RCLCPP_INFO(g_node->get_logger(), "The base failed to move to goal for some reason");
      }
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

