#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <functional>
using std::placeholders::_1;
void mysub_callback(rclcpp::Node::SharedPtr node, const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Received message: %d", msg->data);  // 수신한 메시지 출력
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);  // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("node_sub1");  // 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));  // QoS 설정
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);  // 콜백 함수 바인딩
    auto mysub = node->create_subscription<std_msgs::msg::Int32>("topic_pub1", qos_profile, fn);  // 구독자 생성
    rclcpp::spin(node);  // 콜백 실행 대기
    rclcpp::shutdown();  // ROS 2 종료
    return 0;
}
