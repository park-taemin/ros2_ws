cmake_minimum_required(VERSION 3.5)
project(linedetect_sub_7)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(sub_7 src/sub_7.cpp)
ament_target_dependencies(sub_7 rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS sub_7
  DESTINATION lib/${PROJECT_NAME})

ament_package()
