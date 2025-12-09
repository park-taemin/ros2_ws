#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

class VideoPublisher7 : public rclcpp::Node
{
public:
    VideoPublisher7(const std::string &video_path) 
    : Node("video_pub_7")
    {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw_7", 10);

        cap.open(video_path);
        if(!cap.isOpened()){
            RCLCPP_ERROR(this->get_logger(), "Cannot open video file: %s", video_path.c_str());
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            33ms, std::bind(&VideoPublisher7::timer_callback, this)
        );
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap;

    void timer_callback()
    {
        cv::Mat frame;
        cap >> frame;

        if(frame.empty()){
            RCLCPP_INFO(this->get_logger(), "Video finished.");
            rclcpp::shutdown();
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_->publish(*msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if(argc < 2){
        std::cerr << "Usage: ros2 run linedetect_pub_7 pub_7 <video_path>" << std::endl;
        return -1;
    }

    auto node = std::make_shared<VideoPublisher7>(argv[1]);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
