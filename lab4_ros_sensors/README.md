# Lab 4: ROS 센서 통합 (ROS Sensor Integration)

## 목표
센서 데이터를 ROS 토픽으로 발행하여 상위 시스템과 통신합니다.

## 실습 내용

### 1. 초음파 센서 ROS 통합
- **파일**: `ultrasonic_ros.ino`
- **토픽**: `/ultrasonic` (std_msgs/Float32)
- **데이터**: 거리 (cm)
- **주기**: 10Hz (100ms)

#### 핀 연결
- Trig → Pin 9
- Echo → Pin 10

#### ROS 실행
```bash
# Terminal 1
roscore

# Terminal 2
rosrun rosserial_python serial_node.py /dev/ttyUSB0

# Terminal 3: 데이터 확인
rostopic echo /ultrasonic

# Terminal 4: 주기 확인
rostopic hz /ultrasonic
```

### 2. 다중 센서 ROS 통합
- **파일**: `multi_sensors_ros.ino`
- **토픽**: 
  - `/ultrasonic` (std_msgs/Float32) - 거리
  - `/ir_sensor` (std_msgs/Bool) - 장애물 감지

#### 핀 연결
- Ultrasonic Trig → Pin 9
- Ultrasonic Echo → Pin 10
- IR Sensor → Pin 7

#### ROS 실행
```bash
# 데이터 시각화 (rqt)
rqt_plot /ultrasonic/data

# 다중 토픽 확인
rostopic list

# 토픽 정보 확인
rostopic info /ultrasonic
rostopic info /ir_sensor
```

## 학습 내용
1. 센서 데이터의 ROS 메시지 변환
2. 다중 Publisher 사용
3. 적절한 발행 주기 설정
4. ROS 토픽 모니터링 및 디버깅

## ROS 도구 활용

### rqt_graph
```bash
rosrun rqt_graph rqt_graph
```
노드와 토픽의 연결 관계를 시각화합니다.

### rqt_plot
```bash
rosrun rqt_plot rqt_plot
```
센서 데이터를 실시간 그래프로 표시합니다.

### rostopic
```bash
# 토픽 목록
rostopic list

# 토픽 정보
rostopic info /ultrasonic

# 토픽 타입
rostopic type /ultrasonic

# 발행 주기
rostopic hz /ultrasonic

# 데이터 확인
rostopic echo /ultrasonic
```

## 응용 과제
1. 추가 센서 통합 (온도, 습도, 가속도 등)
2. 센서 데이터 필터링 (이동 평균, 칼만 필터)
3. 센서 퓨전 (다중 센서 데이터 결합)

## 다음 단계
Lab 5에서는 ROS를 통한 모터 제어를 학습합니다.
