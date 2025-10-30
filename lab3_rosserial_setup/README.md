# Lab 3: rosserial 설정

## 목표
Arduino와 ROS 간의 통신을 설정하고 기본적인 Publisher/Subscriber 패턴을 학습합니다.

## 필요 환경
- **ROS** (Noetic, Melodic, 또는 Humble)
- **Arduino IDE**
- **rosserial 패키지**

## rosserial 설치

### ROS1 (Noetic/Melodic)
```bash
# rosserial 설치
sudo apt-get install ros-$ROS_DISTRO-rosserial-arduino
sudo apt-get install ros-$ROS_DISTRO-rosserial

# ros_lib 라이브러리 생성
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```

### ROS2 (Humble/Foxy)
```bash
# micro-ROS 사용 권장
# 또는 rosserial for ROS2
sudo apt-get install ros-$ROS_DISTRO-rosserial
```

## 예제 1: Hello World Publisher

### Arduino 업로드
1. `hello_world.ino` 파일 열기
2. ros_lib 라이브러리가 설치되었는지 확인
3. Arduino에 업로드

### ROS 실행
```bash
# 터미널 1: roscore 실행
roscore

# 터미널 2: rosserial 노드 실행
rosrun rosserial_python serial_node.py /dev/ttyACM0

# 터미널 3: 메시지 확인
rostopic echo /chatter
```

**예상 출력:**
```
data: "hello world!"
---
data: "hello world!"
```

## 예제 2: LED Subscriber

### Arduino 업로드
1. `led_subscriber.ino` 파일 열기
2. Arduino에 업로드

### ROS 실행
```bash
# 터미널 1: roscore 실행
roscore

# 터미널 2: rosserial 노드 실행
rosrun rosserial_python serial_node.py /dev/ttyACM0

# 터미널 3: LED 켜기
rostopic pub /led_control std_msgs/Bool "data: true"

# LED 끄기
rostopic pub /led_control std_msgs/Bool "data: false"
```

## 통신 프로토콜
- **Baud Rate**: 57600 (기본값)
- **토픽 기반 통신**: Publisher/Subscriber 패턴
- **메시지 타입**: ROS 표준 메시지 타입 사용

## 문제 해결

### 포트 권한 오류
```bash
sudo chmod 666 /dev/ttyACM0
# 또는
sudo usermod -a -G dialout $USER
```

### 연결 실패
1. Arduino 재부팅
2. rosserial_python 재시작
3. USB 케이블 확인

## 주요 개념
- **NodeHandle**: ROS와 Arduino 간의 통신 핸들러
- **Publisher**: 데이터를 ROS로 전송
- **Subscriber**: ROS로부터 데이터 수신
- **spinOnce()**: 콜백 함수 처리

## 다음 단계
Lab 4에서는 센서 데이터를 ROS로 전송합니다.
