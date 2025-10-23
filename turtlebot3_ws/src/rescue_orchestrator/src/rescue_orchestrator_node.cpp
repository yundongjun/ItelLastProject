#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>
#include <mutex>
#include <condition_variable>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class RescueOrchestrator : public rclcpp::Node {
public:
  RescueOrchestrator() : Node("rescue_orchestrator") {
    // === Parameters ===
    uart_cmd_topic_ = declare_parameter<std::string>("uart_cmd_topic", "/uart/rx");
    forklift_up_srv_ = declare_parameter<std::string>("forklift_up_service", "/turtlebot3_node/forklift_up");
    forklift_down_srv_ = declare_parameter<std::string>("forklift_down_service", "/turtlebot3_node/forklift_down");
    rotate_deg_ = declare_parameter<double>("rotate_deg", 90.0);
    home_x_ = declare_parameter<double>("home_x", 0.0);
    home_y_ = declare_parameter<double>("home_y", 0.0);
    home_yaw_deg_ = declare_parameter<double>("home_yaw", 0.0);
    nav_timeout_ms_ = declare_parameter<int>("nav_timeout_ms", 20000);
    forklift_timeout_ms_ = declare_parameter<int>("forklift_timeout_ms", 8000);
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 5000);

    // === Clients/Action/Subs ===
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    forklift_up_cli_   = create_client<std_srvs::srv::Trigger>(forklift_up_srv_);
    forklift_down_cli_ = create_client<std_srvs::srv::Trigger>(forklift_down_srv_);
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){ last_amcl_ = msg; });

    // UART cmd one-shot subscriber (활성/비활성은 서비스 호출 시 제어)
    // 여기서는 콜백만 준비
    uart_cmd_cb_ = [this](std_msgs::msg::String::SharedPtr msg){
      std::string s = msg->data;
      normalize_cmd_(s);
      if (s=="LEFT" || s=="RIGHT" || s=="PICK") {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        if (!got_cmd_) { cmd_ = s; got_cmd_ = true; }
        cmd_cv_.notify_all();
      }
    };

    // Service: run_after_signal
    srv_ = create_service<std_srvs::srv::Trigger>(
      "run_after_signal",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        (void)req;
        if (busy_.exchange(true)) {
          res->success = false;
          res->message = "Already running";
          return;
        }
        bool ok = run_sequence_once_();
        busy_ = false;
        res->success = ok;
        res->message = ok ? "DONE" : "FAILED";
      });

    RCLCPP_INFO(get_logger(), "rescue_orchestrator ready. Call /rescue_orchestrator/run_after_signal");
  }

private:
  // ----------------- Core sequence -----------------
  bool run_sequence_once_() {
    // 1) Wait one command from UART: LEFT/RIGHT/PICK
    RCLCPP_INFO(get_logger(), "Waiting for UART command (LEFT/RIGHT/PICK)...");
    got_cmd_ = false;
    cmd_.clear();

    auto sub = create_subscription<std_msgs::msg::String>(
      uart_cmd_topic_, rclcpp::QoS(10), uart_cmd_cb_);

    {
      std::unique_lock<std::mutex> lk(cmd_mtx_);
      if (!cmd_cv_.wait_for(lk, std::chrono::milliseconds(command_timeout_ms_),
                            [this]{ return got_cmd_; })) {
        RCLCPP_WARN(get_logger(), "No UART cmd within %d ms. Treating as PICK.", command_timeout_ms_);
        cmd_ = "PICK";
        got_cmd_ = true;
      }
    }

    RCLCPP_INFO(get_logger(), "Command = %s", cmd_.c_str());

    // 2) Rotate in place if LEFT or RIGHT
    if (cmd_ == "LEFT" || cmd_ == "RIGHT") {
      double sign = (cmd_ == "LEFT") ? -1.0 : +1.0;
      if (!rotate_in_place_(sign * rotate_deg_)) {
        RCLCPP_ERROR(get_logger(), "Rotate failed");
        return false;
      }
    }

    // 3) Forklift up
    if (!call_trigger_(forklift_up_cli_, forklift_timeout_ms_)) {
      RCLCPP_ERROR(get_logger(), "Forklift UP failed");
      return false;
    }

    // 4) Go home
    if (!go_to_pose_(home_x_, home_y_, deg2rad_(home_yaw_deg_))) {
      RCLCPP_ERROR(get_logger(), "Go home failed");
      return false;
    }

    // 5) Forklift down
    if (!call_trigger_(forklift_down_cli_, forklift_timeout_ms_)) {
      RCLCPP_ERROR(get_logger(), "Forklift DOWN failed");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Sequence DONE");
    return true;
  }

  // ----------------- Helpers -----------------
  static double deg2rad_(double d){ return d * M_PI / 180.0; }

  void normalize_cmd_(std::string &s) {
    // trim spaces and CR/LF
    auto trim = [](std::string &x){
      while(!x.empty() && (x.back()=='\r' || x.back()=='\n' || x.back()==' ')) x.pop_back();
      size_t i=0; while(i<x.size() && x[i]==' ') i++; x = x.substr(i);
    };
    trim(s);
    for (auto &c: s) c = ::toupper(c);
    if (s=="1") s="PICK";
  }

  bool call_trigger_(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &cli, int timeout_ms) {
    if (!cli->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Service not available");
      return false;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = cli->async_send_request(req);
    if (fut.wait_for(std::chrono::milliseconds(timeout_ms)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Service timeout");
      return false;
    }
    auto resp = fut.get();
    if (!resp->success) {
      RCLCPP_ERROR(get_logger(), "Service failed: %s", resp->message.c_str());
      return false;
    }
    return true;
    }

  bool rotate_in_place_(double deg) {
    // need current pose
    if (!last_amcl_) {
      RCLCPP_WARN(get_logger(), "No AMCL pose yet");
      return false;
    }
    const auto &p = last_amcl_->pose.pose;
    double yaw = tf2::getYaw(p.orientation);
    double yaw_t = yaw + deg2rad_(deg);
    return go_to_pose_(p.position.x, p.position.y, yaw_t);
  }

  bool go_to_pose_(double x, double y, double yaw) {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
      return false;
    }
    NavigateToPose::Goal goal;
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    tf2::Quaternion q; q.setRPY(0,0,yaw);
    goal.pose.pose.orientation = tf2::toMsg(q);

    auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
    auto fut_gh = nav_client_->async_send_goal(goal, send_opts);
    if (fut_gh.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Send goal timeout");
      return false;
    }
    auto gh = fut_gh.get();
    if (!gh) { RCLCPP_ERROR(get_logger(), "Goal rejected"); return false; }

    auto fut_res = nav_client_->async_get_result(gh);
    if (fut_res.wait_for(std::chrono::milliseconds(nav_timeout_ms_)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Nav timeout, canceling goal");
      nav_client_->async_cancel_goal(gh);
      return false;
    }
    auto res = fut_res.get();
    if (res.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "Nav failed (code %d)", (int)res.code);
      return false;
    }
    return true;
  }

  // ----------------- Members -----------------
  // params
  std::string uart_cmd_topic_;
  std::string forklift_up_srv_, forklift_down_srv_;
  double rotate_deg_, home_x_, home_y_, home_yaw_deg_;
  int nav_timeout_ms_, forklift_timeout_ms_, command_timeout_ms_;

  // state
  std::atomic<bool> busy_{false};
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr forklift_up_cli_, forklift_down_cli_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_amcl_;

  // one-shot cmd wait
  std::function<void(std_msgs::msg::String::SharedPtr)> uart_cmd_cb_;
  std::mutex cmd_mtx_;
  std::condition_variable cmd_cv_;
  bool got_cmd_{false};
  std::string cmd_;

  // service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // MultiThreadedExecutor 권장(서비스 대기 + 서브스크립션 콜백 동시에 처리)
  auto node = std::make_shared<RescueOrchestrator>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

