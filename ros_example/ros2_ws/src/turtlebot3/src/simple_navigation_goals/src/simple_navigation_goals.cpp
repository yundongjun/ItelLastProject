#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::chrono_literals;

class simple_navigation_goals_node : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // 변수명 'ac' 유지 (ROS1의 SimpleActionClient 역할)
  rclcpp_action::Client<NavigateToPose>::SharedPtr ac;

  simple_navigation_goals_node()
  : Node("simple_navigation_goals")  // 노드명 동일
  {
    ac = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  bool waitForServer(const std::chrono::seconds &timeout = 15s) {
    if (!ac->wait_for_action_server(timeout)) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
      return false;
    }
    return true;
  }

  // ROS1의 sendGoalAndWait 유사 동작
  bool sendGoalAndWait(geometry_msgs::msg::PoseStamped target_pose,
                       const std::chrono::seconds &result_timeout = 20s) {
    NavigateToPose::Goal goal;              // 변수명 'goal' 유지
    goal.pose = target_pose;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.feedback_callback =
      [this](GoalHandleNav::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "feedback: dist_remain=%.2f  eta=%ld",
                             fb->distance_remaining,
                             fb->estimated_time_remaining.sec);
      };

    auto gh_fut = ac->async_send_goal(goal, opts);
    if (rclcpp::spin_until_future_complete(shared_from_this(), gh_fut, result_timeout)
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "send_goal failed (timeout)");
      return false;
    }
    auto gh = gh_fut.get();
    if (!gh) {
      RCLCPP_ERROR(get_logger(), "goal rejected");
      return false;
    }

    auto res_fut = ac->async_get_result(gh);
    if (rclcpp::spin_until_future_complete(shared_from_this(), res_fut, result_timeout)
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "get_result failed (timeout)");
      return false;
    }
    auto wrapped = res_fut.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Goal arrived!");
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "Goal failed (code=%d)", static_cast<int>(wrapped.code));
      return false;
    }
  }

  static geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw_rad,
                                                  const std::string &frame = "map",
                                                  rclcpp::Time stamp = rclcpp::Time(0,0,RCL_ROS_TIME)) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = frame;
    p.header.stamp = stamp;

    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = 0.0;

    // yaw -> quaternion(z,w)만 사용
    double h = yaw_rad * 0.5;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = std::sin(h);
    p.pose.orientation.w = std::cos(h);
    return p;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<simple_navigation_goals_node>();

  if (!node->waitForServer(15s)) {
    rclcpp::shutdown();
    return 1;
  }

  // ====== ROS1 원본 로직과 동일하게: goal 1 -> goal 2 (goal 3은 주석 예시) ======
  // 프레임: "map", 타임스탬프: now()
  auto now = node->now();
  auto goal = simple_navigation_goals_node::makePose(0.0, 0.5, 0.0, "map", now); // goal 1
  RCLCPP_INFO(node->get_logger(), "Sending goal 1");
  node->sendGoalAndWait(goal, 20s);

  goal = simple_navigation_goals_node::makePose(0.5, 0.5, 0.0, "map", node->now()); // goal 2
  RCLCPP_INFO(node->get_logger(), "Sending goal 2");
  node->sendGoalAndWait(goal, 20s);

  /*
  // goal 3 (원본처럼 예시로 남김)
  goal = simple_navigation_goals_node::makePose(0.0, -0.5, 0.0, "map", node->now());
  RCLCPP_INFO(node->get_logger(), "Sending goal 3");
  node->sendGoalAndWait(goal, 20s);
  */

  rclcpp::shutdown();
  return 0;
}

