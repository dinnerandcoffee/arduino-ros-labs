# Arduino-ROS 실습 교재

## 개요
이 교재는 ROS/AI 자율주행 로봇 과정에서 아두이노를 활용해:
- 센서/모터 등 하부 하드웨어를 제어하고
- ROS(rosserial 또는 micro-ROS)로 상위 시스템과 통신하며
- 간단한 폐루프 제어(PID)까지 구현

하도록 설계되었습니다.

## 목차

### [Lab 1: 센서 읽기 (Sensor Reading)](lab1_sensors/)
- 초음파 센서 (HC-SR04)
- 적외선 센서 (IR Sensor)
- 시리얼 통신 기초

### [Lab 2: 모터 제어 (Motor Control)](lab2_motors/)
- DC 모터 제어 (L298N)
- 서보 모터 제어
- PWM 및 H-브리지 이해

### [Lab 3: rosserial 설정 (rosserial Setup)](lab3_rosserial_setup/)
- ROS 설치 및 설정
- rosserial_arduino 라이브러리
- Publisher/Subscriber 패턴

### [Lab 4: ROS 센서 통합 (ROS Sensor Integration)](lab4_ros_sensors/)
- 센서 데이터의 ROS 토픽 발행
- 다중 센서 통합
- rqt 도구를 사용한 시각화

### [Lab 5: ROS 모터 제어 (ROS Motor Control)](lab5_ros_motors/)
- ROS 토픽을 통한 모터 제어
- 차동 구동 로봇 구현
- geometry_msgs/Twist 이해

### [Lab 6: PID 제어 (PID Control)](lab6_pid_control/)
- PID 알고리즘 이해
- 모터 속도 제어
- 라인 트레이싱 로봇
- ROS와 PID 통합

## 필요한 하드웨어

### 필수 구성품
1. **Arduino Uno** (또는 호환 보드)
2. **USB 케이블** (Arduino 연결용)
3. **브레드보드** 및 점퍼 와이어

### 센서류
- HC-SR04 초음파 센서
- IR 장애물 감지 센서
- 5채널 라인 센서 어레이 (선택)
- 로터리 엔코더 (선택)

### 액추에이터
- DC 모터 (2개)
- 서보 모터 (SG90)
- L298N 모터 드라이버

### 전원
- 12V 배터리 또는 어댑터 (모터용)
- 9V 배터리 (Arduino용, 선택)

## 필요한 소프트웨어

### Arduino 개발 환경
1. **Arduino IDE** (1.8.x 또는 2.x)
   - 다운로드: https://www.arduino.cc/en/software

2. **필요한 라이브러리**
   - Servo (기본 제공)
   - rosserial_arduino
   - PID (선택, Brett Beauregard's PID Library)

### ROS 환경
1. **ROS Noetic** (Ubuntu 20.04) 또는 **ROS Melodic** (Ubuntu 18.04)
   - 설치 가이드: http://wiki.ros.org/noetic/Installation

2. **rosserial 패키지**
   ```bash
   sudo apt-get install ros-noetic-rosserial-arduino
   sudo apt-get install ros-noetic-rosserial
   ```

3. **유용한 도구**
   ```bash
   sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins
   sudo apt-get install ros-noetic-teleop-twist-keyboard
   ```

## 시작하기

### 1. Arduino 설정
1. Arduino IDE 설치
2. Arduino Uno를 USB로 연결
3. Tools → Board → Arduino Uno 선택
4. Tools → Port → 해당 포트 선택 (예: /dev/ttyUSB0)

### 2. rosserial 라이브러리 설치
```bash
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```

또는 Arduino IDE에서:
- Sketch → Include Library → Manage Libraries
- "Rosserial Arduino Library" 검색 및 설치

### 3. 첫 번째 실습
[Lab 1: 센서 읽기](lab1_sensors/)부터 시작하세요!

## 학습 경로

### 초급 (1-2주)
1. Lab 1: 기본 센서 읽기
2. Lab 2: 모터 제어
3. Arduino 기초 복습

### 중급 (2-3주)
4. Lab 3: ROS 통신 설정
5. Lab 4: ROS 센서 통합
6. Lab 5: ROS 모터 제어

### 고급 (2-3주)
7. Lab 6: PID 제어
8. 통합 프로젝트: 자율주행 로봇

## 문제 해결

### Arduino 업로드 오류
```bash
# 포트 권한 설정
sudo chmod 666 /dev/ttyUSB0
# 또는 사용자를 dialout 그룹에 추가
sudo usermod -a -G dialout $USER
# 재로그인 필요
```

### ROS 연결 오류
```bash
# rosserial 노드 재시작
rosrun rosserial_python serial_node.py /dev/ttyUSB0

# 다른 baud rate 시도
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=57600
```

### 메모리 부족
Arduino Uno의 메모리는 제한적입니다 (SRAM 2KB).
- 불필요한 전역 변수 제거
- F() 매크로로 문자열을 PROGMEM에 저장
- Serial.print 줄이기

## 프로젝트 아이디어

### 입문 프로젝트
1. 장애물 회피 로봇
2. 라인 트레이싱 로봇
3. 벽 따라가기 로봇

### 중급 프로젝트
4. ROS 원격 제어 로봇
5. 센서 퓨전 로봇
6. 자율 주차 로봇

### 고급 프로젝트
7. SLAM 기반 자율주행
8. 물체 인식 및 추적
9. 다중 로봇 협업

## 추가 자료

### 온라인 리소스
- [Arduino 공식 문서](https://www.arduino.cc/reference/en/)
- [ROS Wiki](http://wiki.ros.org/)
- [rosserial Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials)

### 권장 도서
- "Programming Robots with ROS" by Morgan Quigley
- "Arduino Cookbook" by Michael Margolis
- "Robotics, Vision and Control" by Peter Corke

### 커뮤니티
- [Arduino Forum](https://forum.arduino.cc/)
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)

## 기여하기
이 교재에 대한 개선 사항이나 추가 예제가 있으시면 Pull Request를 보내주세요!

## 라이선스
이 교재는 교육 목적으로 자유롭게 사용할 수 있습니다.

## 지원
질문이나 문제가 있으시면 Issues 탭에서 문의해 주세요.

---
**즐거운 로봇 프로그래밍 되세요! 🤖**
