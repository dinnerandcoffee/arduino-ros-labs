# Lab 5: ROS 모터 제어 (ROS Motor Control)

## 목표
ROS 토픽을 구독하여 모터를 제어하고, 차동 구동 로봇을 구현합니다.

## 실습 내용

### 1. DC 모터 ROS 제어
- **파일**: `motor_control_ros.ino`
- **토픽**: `/motor_cmd` (std_msgs/Int16)
- **값 범위**: -255 ~ 255
  - 양수: 전진 (값이 클수록 빠름)
  - 0: 정지
  - 음수: 후진 (절대값이 클수록 빠름)

#### 핀 연결
- ENA → Pin 9 (PWM)
- IN1 → Pin 8
- IN2 → Pin 7

#### ROS 명령
```bash
# 전진 (최대 속도)
rostopic pub /motor_cmd std_msgs/Int16 "data: 255"

# 전진 (절반 속도)
rostopic pub /motor_cmd std_msgs/Int16 "data: 128"

# 정지
rostopic pub /motor_cmd std_msgs/Int16 "data: 0"

# 후진 (최대 속도)
rostopic pub /motor_cmd std_msgs/Int16 "data: -255"
```

### 2. 서보 모터 ROS 제어
- **파일**: `servo_control_ros.ino`
- **토픽**: `/servo_cmd` (std_msgs/Int16)
- **값 범위**: 0 ~ 180 (각도)

#### 핀 연결
- Signal → Pin 6
- VCC → 5V
- GND → GND

#### ROS 명령
```bash
# 중앙 위치
rostopic pub /servo_cmd std_msgs/Int16 "data: 90"

# 최소 각도
rostopic pub /servo_cmd std_msgs/Int16 "data: 0"

# 최대 각도
rostopic pub /servo_cmd std_msgs/Int16 "data: 180"
```

### 3. 차동 구동 로봇
- **파일**: `differential_drive_ros.ino`
- **토픽**: `/cmd_vel` (geometry_msgs/Twist)
- **기능**: 선속도와 각속도로 로봇 제어

#### 핀 연결
**Left Motor:**
- ENA → Pin 9 (PWM)
- IN1 → Pin 8
- IN2 → Pin 7

**Right Motor:**
- ENB → Pin 3 (PWM)
- IN3 → Pin 5
- IN4 → Pin 4

#### ROS 명령
```bash
# 전진
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'

# 후진
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: -0.5}, angular: {z: 0.0}}'

# 좌회전 (제자리)
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 1.0}}'

# 우회전 (제자리)
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: -1.0}}'

# 정지
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

## 학습 내용
1. ROS Subscriber를 통한 모터 제어
2. 콜백 함수에서의 액추에이터 제어
3. 차동 구동 로봇의 운동학
4. geometry_msgs/Twist 메시지 이해
5. 선속도와 각속도의 바퀴 속도 변환

## 차동 구동 운동학

### 수식
```
left_speed = linear_velocity - angular_velocity * wheel_base / 2
right_speed = linear_velocity + angular_velocity * wheel_base / 2
```

### 동작 원리
- **전진/후진**: 양쪽 바퀴 동일 속도
- **좌회전**: 왼쪽 바퀴 느리게, 오른쪽 바퀴 빠르게
- **우회전**: 왼쪽 바퀴 빠르게, 오른쪽 바퀴 느리게
- **제자리 회전**: 한쪽 전진, 한쪽 후진

## 응용 과제
1. 키보드 원격 조종 (teleop_twist_keyboard)
2. 조이스틱 제어
3. 속도 제한 및 가감속 구현
4. 오도메트리 계산 및 발행

## 다음 단계
Lab 6에서는 PID 제어를 학습합니다.
