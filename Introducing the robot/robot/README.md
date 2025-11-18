1) Nvidia사의 Jetson nano 보드
Jetson Nano는 NVIDIA가 개발한 소형 AI 컴퓨팅 보드로, 딥러닝·컴퓨터비전·로봇 프로젝트를 위해 만들어진 장치이다.
특징
CPU: Quad-core ARM Cortex-A57
GPU: 128-core Maxwell GPU → AI 연산에 매우 유리
RAM: 4GB / 2GB 모델 존재
Storage: microSD 카드 사용
OS: Ubuntu 기반 JetPack SDK (CUDA, cuDNN, TensorRT 포함)
입출력: GPIO, I2C, UART, SPI 등
카메라 인터페이스: MIPI-CSI 지원

용도
자율주행, 객체 인식, SLAM, 로봇 제어(Ros + Jetson), 컴퓨터 비전 프로젝트

2) IMX219 카메라의 사양을 조사하라
IMX219는 Sony에서 만든 8MP 카메라 센서로 Jetson Nano와 Raspberry Pi에서 많이 사용된다.
주요 사양
해상도: 8 Megapixel (3280 × 2464)
센서 크기: 1/4"
렌즈 시야각(FOV): 약 62°(기본)
포커스: Fixed Focus
프레임레이트:
1080p @ 30fps
720p @ 60fps
VGA @ 90fps
커넥터: MIPI CSI-2 (15핀 리본 케이블)
제조사: Sony Exmor 센서

3) 카메라의 리눅스 장치파일 이름은?

리눅스에서 카메라는 V4L2(Video4Linux2) 프레임워크를 통해 관리된다.

따라서 카메라 장치는 운영체제에서 특수 파일(디바이스 파일) 형태로 나타난다.

Jetson Nano의 경우 MIPI CSI 카메라는 항상 V4L2 장치 파일로 인식되지만, 특징적으로

기본적으로 /dev/video0 또는 /dev/video1로 인식된다.

4) CSI(camera serial interface)에 대하여 조사하라
CSI(Camera Serial Interface)**는 MIPI Alliance에서 만든 고속 카메라 인터페이스 규격이다

특징
스마트폰, 임베디드 보드에서 가장 많이 사용되는 카메라 인터페이스

고속 직렬 통신 방식

CSI-2가 가장 널리 사용

데이터 라인(lane) 수에 따라 대역폭 증가

낮은 전력 소비

Jetson Nano에서는?
MIPI CSI-2 2-lane 또는 4-lane 지원
→ 고해상도 카메라를 지연 없이 사용할 수 있음.

5) Gstreamer에 대하여 자세히 조사하라
GStreamer는 오픈소스 멀티미디어 프레임워크
영상/음성 캡처

디코딩/인코딩

스트리밍 (RTSP, RTP 등)

필터링, 후처리

플러그인 기반 구조

OpenCV, ROS, Jetson Nano 등 다양한 시스템에서 사용

Jetson Nano에서 중요한 이유
NVIDIA는 하드웨어 가속 디코딩/인코딩을 GStreamer 플러그인으로 제공한다.

6) Dynamixel에 대하여 자세히 조사하라
Dynamixel은 로보티즈(ROBOTIS)에서 개발한 스마트 서보 모터 브랜드이다.

특징
위치·속도·토크 제어 모두 가능

PID 제어 내장

Daisy-chain 방식(직렬 연결)

피드백 가능(각도, 온도, 전류 등)

명령 프로토콜 제공

시리즈 예시
AX 시리즈, XL, XM, XH 시리즈, Pro 시리즈

인터페이스
TTL, RS485
제어: USB2Dynamixel, U2D2 모듈 사용

7) U2D2 장치의 역할은 무엇인가?

U2D2는 Robotis에서 만든 USB ↔ Dynamixel 통신 모듈이다.

역할
PC USB 신호를
→ Dynamixel에서 사용하는 TTL 또는 RS485 신호로 변환

Dynamixel 모터 제어용

기능
USB ↔ UART 변환

TTL/RS485 선택

PC에서 Dynamixel Wizard 사용 가능

여러 Dynamixel 연결 가능

8) U2D2 리눅스 장치파일의 이름은?
U2D2는 USB ↔ TTL/RS485 변환기 이므로, 리눅스에서는 USB 시리얼 장치로 인식됨.
USB 연결 → USB-Serial 변환 → ttyUSBX 장치파일 생성.
U2D2의 장치파일 이름은 보통 다음과 같음.
/dev/ttyUSB0

USB 장치 목록 확인

ls /dev/ttyUSB*
