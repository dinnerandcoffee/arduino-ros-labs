# Arduino-ROS 실습 교재

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Arduino](https://img.shields.io/badge/Arduino-Compatible-green.svg)](https://www.arduino.cc/)

이 교재는 **ROS/AI 자율주행 로봇 과정**에서 아두이노를 활용하여:
- 🔧 **센서/모터** 등 하부 하드웨어를 제어하고
- 🤖 **ROS**(rosserial 또는 micro-ROS)로 상위 시스템과 통신하며
- 🎯 간단한 **폐루프 제어(PID)**까지 구현

하도록 설계된 종합 실습 교재입니다.

## 📚 목차

### [Lab 1: 센서 읽기](lab1_sensors/)
초음파 센서와 IR 센서를 사용하여 환경 정보를 수집하는 방법을 학습합니다.

### [Lab 2: 모터 제어](lab2_motors/)
DC 모터와 서보 모터를 제어하여 로봇을 움직이는 방법을 학습합니다.

### [Lab 3: rosserial 설정](lab3_rosserial_setup/)
Arduino와 ROS 간의 통신을 설정하고 기본적인 메시지 송수신을 학습합니다.

### [Lab 4: ROS 센서 통합](lab4_ros_sensors/)
센서 데이터를 ROS 토픽으로 발행하여 상위 시스템과 공유하는 방법을 학습합니다.

### [Lab 5: ROS 모터 제어](lab5_ros_motors/)
ROS 토픽을 구독하여 모터를 제어하고 차동 구동 로봇을 구현합니다.

### [Lab 6: PID 제어](lab6_pid_control/)
PID 알고리즘을 이해하고 모터 속도 제어 및 라인 트레이싱에 적용합니다.

## 🚀 빠른 시작

### 필수 하드웨어
- Arduino Uno (또는 호환 보드)
- HC-SR04 초음파 센서
- IR 센서
- DC 모터 + L298N 모터 드라이버
- 서보 모터 (SG90)
- 브레드보드 및 점퍼 와이어

### 필수 소프트웨어
- [Arduino IDE](https://www.arduino.cc/en/software)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation) (Ubuntu 20.04)
- rosserial_arduino 패키지

### 설치 가이드
1. **[시작 가이드](docs/GETTING_STARTED.md)** - 전체 개요 및 학습 경로
2. **[ROS 설정](docs/ROS_SETUP.md)** - ROS 및 rosserial 설치
3. **[하드웨어 연결](docs/HARDWARE_SETUP.md)** - 회로 연결 방법

## 📖 학습 경로

### 🌱 초급 (1-2주)
- Lab 1: 기본 센서 읽기
- Lab 2: 모터 제어

### 🌿 중급 (2-3주)
- Lab 3: ROS 통신 설정
- Lab 4: ROS 센서 통합
- Lab 5: ROS 모터 제어

### 🌳 고급 (2-3주)
- Lab 6: PID 제어
- 통합 프로젝트: 자율주행 로봇

## 💡 프로젝트 아이디어

- 🚗 장애물 회피 로봇
- 🛤️ 라인 트레이싱 로봇
- 🎮 ROS 원격 제어 로봇
- 🗺️ SLAM 기반 자율주행

## 🔧 문제 해결

### Arduino 업로드 오류
```bash
sudo chmod 666 /dev/ttyUSB0
# 또는
sudo usermod -a -G dialout $USER
```

### ROS 연결 오류
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

자세한 내용은 각 Lab의 README 및 [문서](docs/)를 참조하세요.

## 📚 추가 자료

- [Arduino 공식 문서](https://www.arduino.cc/reference/en/)
- [ROS Wiki](http://wiki.ros.org/)
- [rosserial Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials)

## 🤝 기여하기

개선 사항이나 추가 예제가 있으시면 Pull Request를 보내주세요!

## 📄 라이선스

이 교재는 교육 목적으로 자유롭게 사용할 수 있습니다.

## 📧 지원

질문이나 문제가 있으시면 Issues 탭에서 문의해 주세요.

---

**즐거운 로봇 프로그래밍 되세요! 🤖✨**
