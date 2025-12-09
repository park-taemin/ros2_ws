7_lt_ccw_100rpm_in.mp4 에 대한 시뮬레이션 실행

https://youtu.be/VHFRFAoNa3Q

pub_7.cpp 설명

<img width="621" height="85" alt="image" src="https://github.com/user-attachments/assets/676516b5-4c92-4cf3-a454-98b855af188b" />

생성자

<img width="743" height="352" alt="image" src="https://github.com/user-attachments/assets/cdd7b823-b146-4088-bb3e-733e8b8c9174" />


노드 이름은 "video_pub_7". 퍼블리시 토픽 이름: image_raw_7, 큐 사이즈: 10 타이머가 불릴 때마다 timer_callback() 실행\

<img width="713" height="388" alt="image" src="https://github.com/user-attachments/assets/2818b80b-1df1-4c12-b0fc-e0352a9e89fb" />

timer_callback(): 프레임 읽어서 publish, 영상이 끝났다면 종료

