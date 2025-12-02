#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LineSubscriber : public rclcpp::Node
{
public:
    LineSubscriber() : Node("line_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "video", 10,
            std::bind(&LineSubscriber::image_callback, this, std::placeholders::_1)
        );
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // 우선 영상만 출력
        cv::imshow("Receive Video", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineSubscriber>());
    rclcpp::shutdown();
    return 0;
}
