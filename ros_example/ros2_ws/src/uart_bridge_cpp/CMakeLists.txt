cmake_minimum_required(VERSION 3.8)
project(uart_bridge_cpp)

# 권장: C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 기존 기본 브리지 (/dev/serial0 등 파라미터형)
add_executable(uart_bridge_node src/uart_bridge_node.cpp)
ament_target_dependencies(uart_bridge_node rclcpp std_msgs)

# ★ 신규: /dev/ttyAMA1 고정 브리지
add_executable(uart_bridge_ama1_node src/uart_bridge_ama1_node.cpp)
ament_target_dependencies(uart_bridge_ama1_node rclcpp std_msgs)

# 설치
install(TARGETS
  uart_bridge_node
  uart_bridge_ama1_node
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일(멀티, ama1 포함) 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

