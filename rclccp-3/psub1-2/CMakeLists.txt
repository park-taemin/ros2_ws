cmake_minimum_required(VERSION 3.16)
project(psub1-2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#build
add_executable(pub1-2 src/pub1-2.cpp)
ament_target_dependencies(pub1-2 rclcpp std_msgs)

add_executable(sub1-2 src/sub1-2.cpp)
ament_target_dependencies(sub1-2 rclcpp std_msgs)

# Install
install(TARGETS
  pub1-2
  sub1-2
  DESTINATION lib/${PROJECT_NAME})
ament_package()