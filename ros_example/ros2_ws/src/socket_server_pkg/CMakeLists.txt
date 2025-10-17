cmake_minimum_required(VERSION 3.8)
project(socket_server_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(socket_server_node
  src/socket_server_node.cpp
)
ament_target_dependencies(socket_server_node rclcpp std_msgs)

# pthread (일부 환경에서 필요)
if(UNIX AND NOT APPLE)
  target_link_libraries(socket_server_node pthread)
endif()

install(TARGETS socket_server_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
