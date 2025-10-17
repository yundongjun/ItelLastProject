cmake_minimum_required(VERSION 3.8)
project(ros_tutorials_topic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ★ 메시지 생성기/타입 의존
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ★ 커스텀 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MsgTutorial.msg"
  DEPENDENCIES builtin_interfaces
)

# 실행 파일
add_executable(topic_publisher src/topic_publisher.cpp)
ament_target_dependencies(topic_publisher rclcpp std_msgs)

add_executable(topic_subscriber src/topic_subscriber.cpp)
ament_target_dependencies(topic_subscriber rclcpp std_msgs)

# ★ 실행 파일이 생성된 인터페이스에 의존하도록 연결(동일 패키지 사용 시 필수)
rosidl_target_interfaces(topic_publisher  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(topic_subscriber ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 설치
install(TARGETS
  topic_publisher
  topic_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

