cmake_minimum_required(VERSION 3.8)
project(ros_goal_client)

# 컴파일 경고 옵션(선택)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- srv 인터페이스 생성 (CamelCase 파일명 주의) ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Bot3Gpio.srv"
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

# ---- 실행 파일 타깃 ----
add_executable(rosGoalServer src/rosGoalServer.cpp)
add_executable(rosGoalClient src/rosGoalClient.cpp)  # 소스가 있다면 함께 빌드

ament_target_dependencies(rosGoalServer
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)
ament_target_dependencies(rosGoalClient
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

# 생성된 타입서포트 링크 (최신 방식)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rosGoalServer ${cpp_typesupport_target})
target_link_libraries(rosGoalClient ${cpp_typesupport_target})

# ---- 설치 (ros2 run 가능하게) ----
install(TARGETS
  rosGoalServer
  rosGoalClient
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

