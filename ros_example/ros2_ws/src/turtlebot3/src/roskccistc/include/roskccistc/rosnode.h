/* kccistc embedded & iot by ksh */
#ifndef __ROSNODE__H_
#define __ROSNODE__H_

#include <QThread>
#include <QSettings>
#include <QLabel>
#include <QDebug>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "roskccistc/srv/bot3gpio.hpp"

class RosNode : public QThread {
    Q_OBJECT
public:
    RosNode(int argc, char **argv);
    ~RosNode();

    bool init();
    void run() override;
    void startHandle();

    void set_goal(QString frame, double x, double y, double z, double w);
    void go_goal(QString frame, double x, double y, double w);

    void LDSMsgCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

private:
    int init_argc;
    char **init_argv;
    QString goal_topic;

    bool bot3gpioFlag;

    // ROS2 core
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    // Publishers / Subscribers / Service client
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr               goal_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr               lds_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr                   image_sub;
    rclcpp::Client<roskccistc::srv::Bot3gpio>::SharedPtr                       kccistc_service_client;

    // Service request buffer
    roskccistc::srv::Bot3gpio::Request::SharedPtr                               srv_request_;

public:
    QLabel* pLcamView;
    void slotBot3GpioService(int, int);

signals:
    void rosShutdown();
    void sigLdsReceive(float *);
    void sigBot3GpioService(int);

private slots:
};

#endif

