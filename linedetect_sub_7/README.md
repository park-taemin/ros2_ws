linedetect_sub_7.cpp 설명

<img width="615" height="97" alt="image" src="https://github.com/user-attachments/assets/05b204de-01ff-41c7-b021-d4d109634fc8" />

LineDetector7 클래스

<img width="596" height="273" alt="image" src="https://github.com/user-attachments/assets/d389fb64-c6a1-48c3-8acd-7d837cca1f82" />

LineDetector7 클래스는 rclcpp::Node를 상속받아 ROS2 노드로 기능을 합니다.
**image_raw_7**라는 토픽을 구독합니다. 이 토픽은 앞서 VideoPublisher7 노드에서 퍼블리시된 이미지입니다.
구독자(subscription_)는 이미지 메시지를 받을 때마다 topic_callback 메소드를 호출합니다.

topic_callback()

<img width="520" height="230" alt="image" src="https://github.com/user-attachments/assets/575b7fc6-fda3-491b-9155-cedb5ea5b47f" />

topic_callback 메소드는 ROS2 메시지인 sensor_msgs::msg::Image를 OpenCV 이미지(Mat 객체)로 변환합니다.
변환 시 cv_bridge::toCvCopy()를 사용하여 bgr8 형식의 이미지를 받아옵니다.

ROI 설정

<img width="411" height="108" alt="image" src="https://github.com/user-attachments/assets/78eb6e2b-1d52-4964-ba9d-29f7b4de426d" />

ROI(Region of Interest): 영상 하단 1/4을 선택하여 라인 추적을 위한 영역을 설정합니다.
roi_y는 이미지의 3/4 지점, roi_h는 나머지 하단 영역의 높이입니다. Mat roi는 이미지의 하단 1/4을 잘라낸 영역입니다.

전처리

<img width="390" height="180" alt="image" src="https://github.com/user-attachments/assets/70af38a8-26bc-4d2c-8548-eb717cc3464a" />

ROI 영역을 그레이스케일로 변환한 후, 밝기 보정을 진행합니다. threshold()를 사용하여 이진화하여 라인을 잘 분리할 수 있도록 합니다.

연결된 컴포넌트 분석

<img width="570" height="56" alt="image" src="https://github.com/user-attachments/assets/1e462590-3b48-4bd3-854f-9f04a0b72924" />

이진화된 이미지를 connectedComponentsWithStats() 함수로 연결된 컴포넌트들을 찾습니다.
stats는 각 컴포넌트의 위치와 크기를 포함하고, centroids는 각 컴포넌트의 중심을 나타냅니다.

후보 라인 검출

<img width="448" height="504" alt="image" src="https://github.com/user-attachments/assets/ad02825a-6f05-412a-91db-e3b4577e8805" />

연결된 컴포넌트들 중에서 면적이 80~8000 픽셀 사이인 후보들을 찾습니다.
prev_line_x_와의 거리 차이가 가장 작은 컴포넌트를 라인으로 선택합니다.

라인 추적 및 시각화

<img width="616" height="556" alt="image" src="https://github.com/user-attachments/assets/216deb36-fe57-49d3-82f9-306fdebcd512" />

가장 가까운 후보를 찾으면 빨간 사각형과 중심점을 그려 라인을 시각화합니다. 라인을 못 찾으면 이전의 라인 위치에 붉은 점을 찍어 이전 상태를 유지합니다.

오차 계산 및 출력

<img width="327" height="48" alt="image" src="https://github.com/user-attachments/assets/66cfc86d-2c7b-4467-bc23-8f5c900625bf" />

로봇 중심과 라인의 중심의 차이를 계산하여 **오차(error)**를 구하고 출력합니다.

결과 출력

<img width="450" height="177" alt="image" src="https://github.com/user-attachments/assets/adb4d5e1-cfe6-4854-96fd-d6f02e9b78a0" />

imshow를 사용하여 결과 영상을 실시간으로 화면에 출력합니다.
