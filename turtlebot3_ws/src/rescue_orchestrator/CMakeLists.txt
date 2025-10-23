cmake_minimum_required(VERSION 3.8)
project(rescue_orchestrator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(rescue_orchestrator_node src/rescue_orchestrator_node.cpp)
ament_target_dependencies(rescue_orchestrator_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs std_srvs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
)

install(TARGETS rescue_orchestrator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

