#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/time_source.hpp"
#include <memory>
#include <chrono>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);  // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("node_pub1");  // 퍼블리셔 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));  // QoS 설정: 최근 10개 메시지 유지
    auto mypub = node->create_publisher<std_msgs::msg::String>("topic_pub1", qos_profile);  // 퍼블리셔 생성

    auto message = std::make_shared<std_msgs::msg::String>();
    message->data = "Hello world!";  // 발행할 문자열 메시지

    // 100ms 주기로 타이머 콜백 실행
    auto timer = node->create_wall_timer(std::chrono::milliseconds(100), [node, mypub, message]() {
        RCLCPP_INFO(node->get_logger(), "Publish: %s", message->data.c_str());  // 메시지 출력
        mypub->publish(*message);  // 메시지 발행
    });

    rclcpp::spin(node);  // 콜백 함수 실행 대기
    rclcpp::shutdown();  // ROS 2 종료
    return 0;
}
