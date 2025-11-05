#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/time_source.hpp"
#include <memory>
#include <chrono>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);  // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("node_pub1");  // 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));  // QoS 설정
    auto mypub = node->create_publisher<std_msgs::msg::Int32>("topic_pub1", qos_profile);  // 퍼블리셔 생성

    auto count = std::make_shared<int>(0);  // 발행할 정수 값 초기화
    auto timer = node->create_wall_timer(std::chrono::milliseconds(50), [node, mypub, count]() {
        std_msgs::msg::Int32 message;
        message.data = (*count)++;  // 0부터 1씩 증가
        RCLCPP_INFO(node->get_logger(), "Publish: %d", message.data);  // 콘솔 출력
        mypub->publish(message);  // 메시지 발행
    });

    rclcpp::spin(node);  // 콜백 실행 대기
    rclcpp::shutdown();  // ROS 2 종료
    return 0;
}
