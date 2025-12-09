#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

class VideoPublisher : public rclcpp::Node {
public:
  VideoPublisher(const std::string& video_path)
  : Node("linedetect_pub")
  {
    pub_ = image_transport::create_publisher(this, "camera/image_raw");

    // 🔥 영상 열기 (명령줄 경로 사용)
    cap_.open(video_path);

    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to open video: %s", video_path.c_str());
      rclcpp::shutdown();
      return;
    }

    // FPS 읽기
    double fps = cap_.get(cv::CAP_PROP_FPS);
    if (fps <= 0.1) {
      RCLCPP_WARN(this->get_logger(), "⚠ FPS read failed. Using 30 FPS instead.");
      fps = 30.0;
    }

    int interval_ms = static_cast<int>(1000.0 / fps);
    RCLCPP_INFO(this->get_logger(), "🎬 Video FPS = %.2f | Timer interval = %d ms", fps, interval_ms);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(interval_ms),
      std::bind(&VideoPublisher::timer_cb, this)
    );
  }

private:
  void timer_cb()
  {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      // 🔁 영상 끝나면 처음으로 되감기
      RCLCPP_INFO(this->get_logger(), "🔁 Restarting video playback...");
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      return;
    }

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    pub_.publish(*msg);
  }

  image_transport::Publisher pub_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: ros2 run linedetect_pub pub_node <video_path>\n";
    return 0;
  }

  auto node = std::make_shared<VideoPublisher>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
