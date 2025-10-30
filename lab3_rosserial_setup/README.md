# Lab 3: rosserial 설정 (rosserial Setup)

## 목표
Arduino와 ROS 간의 통신을 설정하고 기본적인 Publisher/Subscriber 패턴을 학습합니다.

## 사전 준비

### ROS 설치 (Ubuntu)
```bash
# ROS Noetic (Ubuntu 20.04) 또는 ROS Melodic (Ubuntu 18.04)
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```

### Arduino 라이브러리 설치
```bash
# ros_lib 생성
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```

또는 Arduino IDE에서:
1. Sketch → Include Library → Manage Libraries
2. "Rosserial Arduino Library" 검색 및 설치

## 실습 내용

### 1. Hello World Publisher
- **파일**: `hello_world.ino`
- **기능**: Arduino에서 ROS로 문자열 메시지 발행
- **토픽**: `/chatter` (std_msgs/String)

#### 실행 방법
```bash
# Terminal 1: ROS 마스터 실행
roscore

# Terminal 2: rosserial 노드 실행
rosrun rosserial_python serial_node.py /dev/ttyUSB0

# Terminal 3: 토픽 확인
rostopic echo /chatter
```

### 2. LED Subscriber
- **파일**: `led_control.ino`
- **기능**: ROS 토픽을 구독하여 LED 제어
- **토픽**: `/led` (std_msgs/Bool)
- **하드웨어**: Pin 13 (내장 LED)

#### 실행 방법
```bash
# rosserial 노드 실행 후
# LED ON
rostopic pub /led std_msgs/Bool "data: true"

# LED OFF
rostopic pub /led std_msgs/Bool "data: false"
```

## 학습 내용
1. ros::NodeHandle 초기화
2. Publisher 생성 및 메시지 발행
3. Subscriber 생성 및 콜백 함수
4. rosserial_python 사용법

## 문제 해결

### 포트 권한 오류
```bash
sudo chmod 666 /dev/ttyUSB0
# 또는
sudo usermod -a -G dialout $USER
# 재로그인 필요
```

### 포트 찾기
```bash
ls /dev/ttyUSB*
# 또는
ls /dev/ttyACM*
```

### 연결 확인
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=57600
```

## 다음 단계
Lab 4에서는 센서 데이터를 ROS로 전송하는 방법을 학습합니다.
