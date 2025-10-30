# Arduino-ROS Labs (아두이노-ROS 실습)

이 교재는 ROS/AI 자율주행 로봇 과정에서 아두이노를 활용해:
- 센서/모터 등 하부 하드웨어를 제어하고
- ROS(rosserial 또는 micro-ROS)로 상위 시스템과 통신하며
- 간단한 폐루프 제어(PID)까지 구현

하도록 설계되었습니다.

## 📚 과정 개요

본 과정은 7개의 실습 Lab으로 구성되어 있으며, 각 Lab은 순차적으로 진행됩니다:

1. **Lab 1**: 기본 센서 제어 (초음파, IR 센서)
2. **Lab 2**: DC 모터 제어 (L298N 드라이버)
3. **Lab 3**: rosserial 통신 설정
4. **Lab 4**: 센서 데이터 ROS 퍼블리싱
5. **Lab 5**: ROS 명령으로 모터 제어
6. **Lab 6**: PID 폐루프 제어 구현
7. **Lab 7**: 자율주행 통합 시스템

## 🎯 학습 목표

- Arduino 기본 프로그래밍
- 센서/액추에이터 하드웨어 제어
- ROS 통신 프로토콜 이해
- Publisher/Subscriber 패턴 구현
- PID 제어 알고리즘 이해 및 튜닝
- 자율주행 로봇 시스템 통합

## 🛠️ 필요 하드웨어

### 필수 부품
- Arduino Uno (또는 호환 보드)
- L298N 모터 드라이버 모듈
- DC 기어드 모터 2개
- HC-SR04 초음파 센서
- 2륜 로봇 섀시
- 7.4V~12V 배터리 (LiPo 2S 권장)
- 점퍼선 (Male-Female, Male-Male)
- USB 케이블 (Arduino 연결용)

### 선택 부품 (Lab 6-7용)
- 로터리 인코더 2개 (모터 속도 측정)
- IR 적외선 센서

**상세 부품 목록**: [docs/PARTS_LIST.md](docs/PARTS_LIST.md)

## 💻 소프트웨어 요구사항

- **OS**: Ubuntu 18.04 / 20.04 / 22.04
- **ROS**: ROS1 Noetic 또는 ROS2 Humble
- **Arduino IDE**: 1.8.x 또는 2.x
- **rosserial**: rosserial-arduino 패키지

**설치 가이드**: [docs/setup/INSTALLATION.md](docs/setup/INSTALLATION.md)

## 🚀 빠른 시작

### 1. 환경 설정
```bash
# ROS 설치 (Ubuntu 20.04 + ROS Noetic 예시)
sudo apt install ros-noetic-desktop-full

# rosserial 설치
sudo apt install ros-noetic-rosserial-arduino ros-noetic-rosserial

# Arduino 라이브러리 생성
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```

### 2. 하드웨어 연결
회로 연결 가이드를 참고하여 하드웨어를 연결합니다:
- [docs/circuits/WIRING_GUIDE.md](docs/circuits/WIRING_GUIDE.md)

### 3. Lab 실습 시작
```bash
# Lab 1: 센서 테스트
cd lab1_basic_sensors
# Arduino IDE로 ultrasonic_sensor.ino 업로드

# Lab 3 이후: ROS 실행
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

## 📖 Lab 가이드

각 Lab 디렉토리에는 다음이 포함되어 있습니다:
- **Arduino 코드** (.ino 파일)
- **README.md**: 상세 실습 가이드
- 회로 연결 방법
- 실행 및 테스트 방법

### Lab 진행 순서
1. 각 Lab의 README.md 읽기
2. 회로 연결 확인
3. Arduino 코드 업로드
4. 테스트 및 검증
5. 다음 Lab으로 진행

## 🔧 ROS Launch 파일

편리한 실행을 위한 launch 파일들이 제공됩니다:

```bash
# 기본 rosserial 연결
roslaunch arduino_ros_labs basic_serial.launch

# 센서 모니터링 (rqt_plot 포함)
roslaunch arduino_ros_labs sensor_monitor.launch

# Teleop 키보드 제어
roslaunch arduino_ros_labs teleop_control.launch

# 자율주행 전체 시스템
roslaunch arduino_ros_labs autonomous_robot.launch
```

## 📊 시스템 구조

```
┌─────────────────┐
│   ROS (PC)      │
│  - Navigation   │
│  - Planning     │
│  - Perception   │
└────────┬────────┘
         │ rosserial
         │ (USB)
┌────────▼────────┐
│   Arduino       │
│  - Sensor Read  │
│  - Motor Ctrl   │
│  - PID Control  │
└────────┬────────┘
         │
    ┌────┴────┐
    │         │
┌───▼──┐  ┌──▼───┐
│Sensor│  │Motor │
└──────┘  └──────┘
```

## 🎓 학습 경로

### 초급 (Lab 1-3)
- Arduino 기본 프로그래밍
- 센서 데이터 읽기
- 모터 기본 제어
- ROS 통신 기초

### 중급 (Lab 4-5)
- ROS Publisher/Subscriber
- 센서 데이터 통합
- cmd_vel 명령 처리
- Teleop 제어

### 고급 (Lab 6-7)
- PID 제어 이론
- 인코더 기반 속도 제어
- 자율주행 알고리즘
- 시스템 통합

## 🔍 문제 해결

자주 발생하는 문제와 해결 방법:

### Arduino 업로드 실패
```bash
# 포트 권한 설정
sudo usermod -a -G dialout $USER
# 로그아웃 후 재로그인
```

### rosserial 연결 실패
```bash
# Arduino 리셋 후 재시도
# USB 케이블 교체
# Baud rate 확인 (57600)
```

### 모터가 회전하지 않음
- 전원 연결 확인
- L298N GND와 Arduino GND 공통 연결 확인
- ENA, ENB 점퍼 캡 제거 확인

**상세 문제 해결**: 각 Lab의 README.md 참고

## 📚 참고 자료

### 공식 문서
- [ROS Wiki](http://wiki.ros.org/)
- [rosserial Arduino Tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials)
- [Arduino Reference](https://www.arduino.cc/reference/en/)

### 추가 학습 자료
- ROS Navigation Stack
- SLAM (Hector SLAM, GMapping)
- Computer Vision (OpenCV)
- Deep Learning (TensorFlow, PyTorch)

## 🤝 기여

이슈나 개선 사항이 있다면 GitHub Issues를 통해 알려주세요.

## 📝 라이선스

이 프로젝트는 교육 목적으로 만들어졌습니다.

## 📧 문의

질문이나 피드백은 GitHub Issues를 이용해 주세요.

---

**Happy Coding! 즐거운 로봇 개발 되세요! 🤖**
