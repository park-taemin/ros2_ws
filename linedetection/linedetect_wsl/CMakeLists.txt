cmake_minimum_required(VERSION 3.5)
project(linedetect_wsl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(sub src/sub.cpp)
ament_target_dependencies(sub
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
