cmake_minimum_required(VERSION 3.8)
project(linedetect_pub)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

add_executable(pub_node src/pub_node.cpp)
ament_target_dependencies(pub_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
)
target_link_libraries(pub_node ${OpenCV_LIBS})

install(TARGETS
  pub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
