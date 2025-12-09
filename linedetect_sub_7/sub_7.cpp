#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace cv;
using namespace std;

class LineDetector7 : public rclcpp::Node
{
public:
  LineDetector7() : Node("line_detector_7")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw_7",    // ★ 반드시 pub과 일치
      10,
      std::bind(&LineDetector7::topic_callback, this, std::placeholders::_1)
    );

    prev_line_x_ = 320;   // ★ 초기 라인은 영상 중앙
    line_found_prev_ = true;
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto start = std::chrono::steady_clock::now();

    Mat img;
    try {
      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error");
      return;
    }

    if(img.empty()) return;

    // ----- ROI 영역(하단 1/4) -----
    int roi_y = img.rows * 3 / 4;
    int roi_h = img.rows - roi_y;

    Mat roi = img(Rect(0, roi_y, img.cols, roi_h));

    // ----- 전처리 -----
    Mat gray, bin;
    cvtColor(roi, gray, COLOR_BGR2GRAY);

    // 밝기 보정
    double meanVal = mean(gray)[0];
    gray = gray + (100 - meanVal);

    threshold(gray, bin, 160, 255, THRESH_BINARY);

    Mat labels, stats, centroids;
    int num = connectedComponentsWithStats(bin, labels, stats, centroids);

    Mat show;
    cvtColor(bin, show, COLOR_GRAY2BGR);

    int best_idx = -1;
    int best_dist = 999999;

    // ----- 모든 후보 그리기 + 후보 선정 -----
    for(int i=1; i<num; i++)
    {
        int area = stats.at<int>(i, CC_STAT_AREA);
        if(area < 80 || area > 8000) continue;

        int x = stats.at<int>(i, CC_STAT_LEFT);
        int y = stats.at<int>(i, CC_STAT_TOP);
        int w = stats.at<int>(i, CC_STAT_WIDTH);
        int h = stats.at<int>(i, CC_STAT_HEIGHT);
        int cx = centroids.at<double>(i, 0);

        // 파란 박스 (모든 후보)
        rectangle(show, Rect(x,y,w,h), Scalar(255,0,0), 1);

        // 이전 라인(prev_line_x)과 가장 가까운 후보 찾기
        int dist = abs(cx - prev_line_x_);
        if(dist < best_dist)
        {
            best_dist = dist;
            best_idx = i;
        }
    }

    int cx_line = prev_line_x_; // 기본값 = 이전 라인 위치
    bool line_found_now = false;

    // ---- 후보 중 prev_line_x 근처에 있는 경우만 빨간 라인으로 인정 ----
    if(best_idx != -1 && best_dist < 80)   // ★ 80픽셀 이하만 같은 라인으로 인정
    {
        int x = stats.at<int>(best_idx, CC_STAT_LEFT);
        int y = stats.at<int>(best_idx, CC_STAT_TOP);
        int w = stats.at<int>(best_idx, CC_STAT_WIDTH);
        int h = stats.at<int>(best_idx, CC_STAT_HEIGHT);

        cx_line = centroids.at<double>(best_idx, 0);

        // 빨간 박스 + 중심점
        rectangle(show, Rect(x,y,w,h), Scalar(0,0,255), 2);
        circle(show, Point(cx_line, y + h/2), 4, Scalar(0,0,255), -1);

        prev_line_x_ = cx_line;  // 라인 저장
        line_found_now = true;
    }
    else
    {
        // ---- 라인을 못 찾았을 경우: 이전 라인 위치를 그대로 빨간색으로 유지 ----
        // show는 ROI 부분만 있으므로 ROI 좌표에서만 붉은 점 찍음
        circle(show,
               Point(prev_line_x_, roi_h/2),
               4, Scalar(0,0,255), -1);

        line_found_now = false;
    }

    // error 계산
    int robot_center = img.cols / 2;
    int error = robot_center - prev_line_x_;

    auto end = std::chrono::steady_clock::now();
    float ms = std::chrono::duration<float, milli>(end - start).count();

    cout << "error: " << error << ", time: " << ms << " ms" << endl;

    // ROI 결과를 합쳐서 출력
    Mat full_show = img.clone();
    show.copyTo(full_show(Rect(0, roi_y, img.cols, roi_h)));

    imshow("Result7", full_show);
    waitKey(1);

    line_found_prev_ = line_found_now;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int prev_line_x_;
  bool line_found_prev_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineDetector7>());
  rclcpp::shutdown();
  return 0;
}
