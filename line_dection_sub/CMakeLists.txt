cmake_minimum_required(VERSION 3.8)
project(linedetect_sub)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

add_executable(sub_node src/sub_node.cpp)
ament_target_dependencies(sub_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
)
target_link_libraries(sub_node ${OpenCV_LIBS})

install(TARGETS
  sub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
