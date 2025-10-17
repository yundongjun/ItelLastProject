/* kccistc embedded & iot by ksh */
#include "rosnode.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include "roskccistc/srv/bot3gpio.hpp"

using std::placeholders::_1;

RosNode::RosNode(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    bot3gpioFlag = false;

    node_ = rclcpp::Node::make_shared("rosqt");

    // Publishers
    set_pub  = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::QoS(10));
    goal_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("move_base_simple/goal", rclcpp::QoS(10));

    // Subscriptions
    lds_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(&RosNode::LDSMsgCallback, this, _1));

    image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
        "cv_camera/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&RosNode::imageCallback, this, _1));

    // Service client
    kccistc_service_client = node_->create_client<roskccistc::srv::Bot3gpio>("bot3_gpio");

    // Executor (Qt 쓰레드와 공존)
    executor_.add_node(node_);
}

RosNode::~RosNode()
{
    if (rclcpp::ok()) {
        executor_.cancel();
        rclcpp::shutdown();
    }
    wait(); // QThread 종료 대기 (헤더에서 QThread 상속 중이라 가정)
}

void RosNode::run()
{
    rclcpp::WallRate loop_rate(20); // 20 Hz

    while (rclcpp::ok()) {
        executor_.spin_some();

        if (bot3gpioFlag) {
            // 서비스 준비 대기 (짧게)
            (void)kccistc_service_client->wait_for_service(std::chrono::milliseconds(100));

            if (srv_request_) {
                auto future = kccistc_service_client->async_send_request(
                    srv_request_,
                    [this](rclcpp::Client<roskccistc::srv::Bot3gpio>::SharedFuture resp) {
                        auto res = resp.get();
                        emit sigBot3GpioService(res->result);
                    }
                );
            }
            bot3gpioFlag = false;
        }

        loop_rate.sleep();
    }

    emit rosShutdown();
}

void RosNode::set_goal(QString frame, double x, double y, double z, double w)
{
    // initialpose 는 PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = frame.toStdString();
    msg.header.stamp = node_->now();

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.z = z;
    msg.pose.pose.orientation.w = w;

    // 필요시 공분산 기본값(0) 그대로 사용
    set_pub->publish(msg);
}

void RosNode::go_goal(QString frame, double x, double y, double w)
{
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = frame.toStdString();
    goal.header.stamp = node_->now();

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = w;

    goal_pub->publish(goal);
}

void RosNode::LDSMsgCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    float scanData[4] = {0,0,0,0};

    // 간단한 범위 보호
    auto n = static_cast<int>(msg->ranges.size());
    auto clamp = [n](int idx)->int {
        if (idx < 0) return 0;
        if (idx >= n) return n-1;
        return idx;
    };

    scanData[0] = msg->ranges[clamp(0)];
    scanData[1] = msg->ranges[clamp(90)];
    scanData[2] = msg->ranges[clamp(180)];
    scanData[3] = msg->ranges[clamp(270)];

    emit sigLdsReceive(scanData);
}

void RosNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();

        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        cv::line(frame,
                 cv::Point((frame.cols >> 1) - 20, frame.rows >> 1),
                 cv::Point((frame.cols >> 1) + 20, frame.rows >> 1),
                 cv::Scalar(255, 0, 0), 3);
        cv::line(frame,
                 cv::Point(frame.cols >> 1, (frame.rows >> 1) - 20),
                 cv::Point(frame.cols >> 1, (frame.rows >> 1) + 20),
                 cv::Scalar(255, 0, 0), 3);

        QImage qimg(frame.data, frame.cols, frame.rows, QImage::Format_RGB888);
        QImage repImage = qimg.scaled(pLcamView->height(), pLcamView->width(), Qt::KeepAspectRatio);
        pLcamView->setPixmap(QPixmap::fromImage(repImage));
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void RosNode::slotBot3GpioService(int a, int b)
{
    auto req = std::make_shared<roskccistc::srv::Bot3gpio::Request>();
    req->a = a;
    req->b = b;
    srv_request_ = req;      // 헤더에: rclcpp::Client<...>::SharedPtr와 함께 Request::SharedPtr 멤버 선언 가정
    bot3gpioFlag = true;
}

