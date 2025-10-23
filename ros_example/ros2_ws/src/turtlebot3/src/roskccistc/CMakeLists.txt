cmake_minimum_required(VERSION 3.8)
project(roskccistc)

# ---------- Language ----------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------- Core ROS 2 ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# (optional) TurtleBot3 messages if your sources include them
find_package(turtlebot3_msgs QUIET)

# ---------- Interfaces (srv/msg) ----------
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/bot3gpio.srv"
  # If your srv uses std_msgs types, uncomment the next line:
  # DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ---------- Qt5 ----------
# Prefer system Qt; if not found, allow user to set QT5_DIR or extend CMAKE_PREFIX_PATH externally.
set(QT_COMPONENTS Core Gui Widgets Multimedia Network)
find_package(Qt5 REQUIRED COMPONENTS ${QT_COMPONENTS})

# ---------- Sources ----------
file(GLOB QT_FORMS        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_QRCS         RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.qrc)
file(GLOB_RECURSE QT_MOCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
     FOLLOW_SYMLINKS include/roskccistc/*.hpp include/roskccistc/*.h)
file(GLOB_RECURSE CPP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
     FOLLOW_SYMLINKS src/*.cpp)

qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_wrap_cpp(QT_MOC_HPP  ${QT_MOCS})

if(QT_QRCS)
  qt5_add_resources(QT_QRC_CPP ${QT_QRCS})
endif()

# ---------- Target ----------
add_executable(rosqt
  ${CPP_SOURCES}
  ${QT_FORMS_HPP}
  ${QT_MOC_HPP}
  ${QT_QRC_CPP}
)

target_include_directories(rosqt PRIVATE
  include
  include/roskccistc
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(rosqt
  Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Multimedia Qt5::Network
  ${OpenCV_LIBS}
)

ament_target_dependencies(rosqt
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
)

if(turtlebot3_msgs_FOUND)
  ament_target_dependencies(rosqt turtlebot3_msgs)
endif()

# Make sure the node sees the generated srv headers/typesupport
rosidl_target_interfaces(rosqt ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---------- Install ----------
install(TARGETS rosqt
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ---------- Package ----------
ament_package()

# ---------- (Optional) Debug Flags ----------
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -Wall -g -ggdb")

