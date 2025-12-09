
5_lt_cw_100rpm_out.mp4에 대한 시뮬레이션 실행

https://www.youtube.com/watch?v=KeQGOpXlupc

ROS 2 환경에서 비디오 파일을 이미지 토픽으로 발행하는 역할
pub_node.cpp 
코드는 ROS 2 노드인 VideoPublisher 클래스를 중심으로 구성됩니다. 이 클래스는 ROS 2의 핵심 기능인 **노드(Node)**의 역할을 수행하며, 비디오 데이터를 이미지 토픽으로 발행하는 모든 로직을 캡슐화합니다.

<img width="704" height="300" alt="image" src="https://github.com/user-attachments/assets/a5317c28-6375-4cb6-8350-8d36020cc05f" />

상속: public rclcpp::Node를 상속받아 ROS 2의 기본 노드 기능 (토픽 발행/구독, 타이머, 로깅 등)을 사용합니다.
생성자: 노드 이름 ("linedetect_pub")으로 부모 클래스를 초기화하고, 명령줄 인수로 받은 비디오 경로를 사용하여 cv::VideoCapture 객체를 설정합니다.

<img width="680" height="364" alt="image" src="https://github.com/user-attachments/assets/d8c5afcd-af5e-4c90-aaf4-bcddba78ce81" />

멤버 변수 (Private):pub_: 이미지를 발행하는 퍼블리셔입니다. cap_: cv::VideoCapture 객체로, 비디오 파일 관리를 담당합니다. timer_: rclcpp::TimerBase::SharedPtr 타입의 스마트 포인터로, 일정한 주기로 프레임 발행 콜백 함수(timer_cb)를 실행하는 타이머를 관리합니다.
timer_cb 함수는 클래스 내부에 정의된 **메서드(Method)**이며, 비디오 프레임을 처리하고 발행하는 핵심 로직을 담당합니다. 
객체 지향: 이 메서드는 클래스 멤버 변수(cap_, pub_)에 직접 접근하여 비디오를 읽고 메시지를 발행하는 캡슐화된 작업을 수행합니다.

<img width="584" height="256" alt="image" src="https://github.com/user-attachments/assets/9a5d31f8-7223-412c-8325-08fb7ab5b5be" />

std::make_shared: 노드 객체를 **힙(Heap)**에 할당하고, 그 객체를 가리키는 스마트 포인터를 생성하여 메모리 누수를 방지하고 수명 관리를 용이하게 합니다.

CMakeLists.txt

<img width="374" height="500" alt="image" src="https://github.com/user-attachments/assets/2e880436-4ae5-4be6-8eaf-71497781ba48" />

1. ROS2 노드를 빌드하기 위해 필요한 의존성(rclcpp, cv_bridge, OpenCV, image_transport)을 찾아 연결한다.
2. pub_node 실행 파일을 생성하고 OpenCV 라이브러리와 링크하여 영상 처리 기능을 사용할 수 있게 한다.
3. 빌드된 실행 파일을 ROS2 패키지 경로에 설치하도록 설정한다.

package.xml

<img width="504" height="479" alt="image" src="https://github.com/user-attachments/assets/9f7fb109-c889-4db0-a60f-4667cabbfc64" />

영상 퍼블리셔 노드를 구성하기 위해 필요한 ROS2 의존성(rclcpp, sensor_msgs, cv_bridge, image_transport, OpenCV)을 선언하는 메타데이터 파일이다.
패키지 이름, 버전, 라이선스, 메인테이너 정보를 포함하며, ament_cmake 빌드 방식을 사용하도록 정의한다.
