# Lab 4: 센서 데이터 퍼블리셔

## 목표
Arduino에서 센서 데이터를 읽어 ROS 토픽으로 전송하는 방법을 학습합니다.

## 예제 1: 초음파 센서 Publisher

### 하드웨어 연결
```
HC-SR04 초음파 센서:
- VCC  -> 5V
- GND  -> GND
- TRIG -> Pin 9
- ECHO -> Pin 10
```

### Arduino 업로드
`ultrasonic_publisher.ino` 파일을 Arduino에 업로드

### ROS 실행
```bash
# 터미널 1: roscore
roscore

# 터미널 2: rosserial 노드
rosrun rosserial_python serial_node.py /dev/ttyACM0

# 터미널 3: 토픽 확인
rostopic echo /ultrasonic

# 터미널 4: 토픽 정보 확인
rostopic info /ultrasonic
rostopic hz /ultrasonic
```

### 예상 출력
```
header: 
  seq: 123
  stamp: 
    secs: 1234567890
    nsecs: 123456789
  frame_id: "ultrasonic_sensor"
radiation_type: 0
field_of_view: 0.26
min_range: 0.02
max_range: 4.0
range: 0.35
---
```

## 예제 2: 다중 센서 Publisher

### 하드웨어 연결
```
초음파 센서 (위와 동일)
IR 센서:
- VCC -> 5V
- GND -> GND
- OUT -> Pin 7
```

### Arduino 업로드
`multi_sensor_publisher.ino` 파일을 Arduino에 업로드

### ROS 실행
```bash
# rosserial 실행 후
rostopic list

# 예상 토픽:
# /sensors/ultrasonic
# /sensors/ir_obstacle

# 각 토픽 모니터링
rostopic echo /sensors/ultrasonic
rostopic echo /sensors/ir_obstacle
```

## ROS 시각화

### rqt_plot으로 거리 데이터 시각화
```bash
rqt_plot /sensors/ultrasonic/range
```

### rviz로 센서 데이터 시각화
```bash
rviz
# Add -> Range 추가
# Topic: /sensors/ultrasonic
```

## 메시지 타입 설명

### sensor_msgs/Range
```
Header header
  uint32 seq
  time stamp
  string frame_id
uint8 radiation_type    # ULTRASOUND=0, INFRARED=1
float32 field_of_view   # 센서 시야각 (라디안)
float32 min_range       # 최소 측정 거리 (m)
float32 max_range       # 최대 측정 거리 (m)
float32 range           # 측정된 거리 (m)
```

### std_msgs/Bool
```
bool data    # true 또는 false
```

## 응용 과제
1. 센서 데이터 필터링 (이동 평균)
2. 여러 개의 초음파 센서 사용
3. 센서 데이터 기반 경고 메시지 발행

## 다음 단계
Lab 5에서는 ROS 명령을 받아 모터를 제어합니다.
