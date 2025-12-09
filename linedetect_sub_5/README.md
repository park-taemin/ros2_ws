sub_node.cpp 코드 설명

class LineDetector : public rclcpp::Node
ROS2 노드 역할 수행, subscriber 등록, callback 실행, 라인 분석 작업 수행

생성자

<img width="662" height="236" alt="image" src="https://github.com/user-attachments/assets/77c5fb54-9219-4d36-8645-e9b5a0c39b21" />

부모 클래스 rclcpp::Node의 생성자를 호출하며, 노드 이름을 "line_detector"로 설정함. 
"camera/image_raw" 토픽을 구독하고, 콜백 함수로 topic_callback()을 등록함. 큐 사이즈는 10.
줄 19: 초기 라인의 x좌표를 영상 가운데(320px) 로 설정. "현재 인식된 라인의 위치"가 사라졌을 때도 추적이 유지되도록 하기 위함.

이미지 수신 + 복구

<img width="593" height="272" alt="image" src="https://github.com/user-attachments/assets/eb961845-ad51-499c-bbed-91a74245fd28" />

auto start_time 프레임 처리 시간을 측정하기 위해 시작 시간 기록.
ROS2 이미지 메시지를 OpenCV Mat으로 변환. 오류 발생 시 예외 처리 후 콜백 종료.

ROI (하단 1/4 영역) 선택

<img width="447" height="143" alt="image" src="https://github.com/user-attachments/assets/7eaedc12-f2b0-4007-bb91-ff7abb20d26b" />

int roi: ROI는 영상 하단 1/4부분으로 고정. ROI가 이미지 범위를 벗어나지 않도록 보정. ROI를 실제 Mat으로 잘라냄

전처리 (Grayscale → 밝기 보정 → Threshold)

<img width="481" height="334" alt="image" src="https://github.com/user-attachments/assets/39263dac-9e54-4353-bb9a-0e978115d3b0" />

ROI를 흑백영상으로 변환.
전체 평균 밝기를 100으로 맞추는 밝기 보정.
임계값 170으로 이진화(Threshold).
후처리를 위해 컬러(BGR)로 변환 (박스 표시용).

레이블링(Labeling) + 후보 탐색 

<img width="679" height="564" alt="image" src="https://github.com/user-attachments/assets/e19b5dd2-7bdf-4692-9d4b-d2f807303389" />

connectedComponentsWithStats를 이용해라인 후보를 찾고 위치/넓이/중심좌표 등을 계산함.

최종 라인 선택 + 에러 계산

<img width="669" height="355" alt="image" src="https://github.com/user-attachments/assets/8ec407d8-eb34-430b-87cb-f8b2b8d93781" />

선택된 진짜 라인은 빨간 박스로 강조. 중심 기준(320px)과의 오차 계산 → 로봇 제어에 사용 가능. 현재 라인 위치를 다음 프레임을 위한 prev_line_x 로 저장.

main 함수

<img width="397" height="168" alt="image" src="https://github.com/user-attachments/assets/5d3d8067-043c-4840-80e6-3d85d14cab9e" />

rclcpp::init(argc, argv); = ROS2 초기화
rclcpp::spin(std::make_shared<LineDetector>()); = LineDetector 노드 실행 (콜백이 계속 동작하도록 spin)
rclcpp::shutdown(); = 종료 시 ROS2 해제
