cmake_minimum_required(VERSION 3.8)
project(simple_navigation_goals)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
ament_target_dependencies(simple_navigation_goals
  rclcpp rclcpp_action nav2_msgs geometry_msgs
)

install(TARGETS simple_navigation_goals
  DESTINATION lib/${PROJECT_NAME})

ament_package()

