# Lab 7: 자율주행 통합

## 목표
모든 이전 Lab의 내용을 통합하여 간단한 자율주행 로봇을 구현합니다.

## 시스템 구성
- **센서**: 초음파 센서 (장애물 감지)
- **액추에이터**: DC 모터 2개 (인코더 포함)
- **제어**: PID 기반 속도 제어
- **통신**: ROS (rosserial)

## 하드웨어 연결

### 모터 드라이버 (L298N)
```
ENA -> Pin 3 (PWM)
IN1 -> Pin 5
IN2 -> Pin 6
ENB -> Pin 9 (PWM)
IN3 -> Pin 10
IN4 -> Pin 11
```

### 초음파 센서 (HC-SR04)
```
TRIG -> Pin 9
ECHO -> Pin 10
```

### 인코더
```
왼쪽 인코더 A상 -> Pin 2
오른쪽 인코더 A상 -> Pin 18 (A4)
```

## 동작 모드

### 1. 자동 모드 (기본)
로봇이 스스로 장애물을 감지하고 회피합니다.

- **전진**: 장애물이 50cm 이상 떨어져 있을 때
- **회피**: 장애물이 30-50cm 범위에 있을 때 좌회전
- **정지**: 장애물이 30cm 이내에 있을 때

### 2. 수동 모드
ROS cmd_vel 토픽을 받으면 자동으로 수동 모드로 전환됩니다.

## ROS 실행

### 기본 실행
```bash
# 터미널 1: roscore
roscore

# 터미널 2: rosserial
rosrun rosserial_python serial_node.py /dev/ttyACM0

# 터미널 3: 상태 모니터링
rostopic echo /robot/status

# 터미널 4: 센서 데이터 확인
rostopic echo /sensors/ultrasonic
```

### 수동 제어로 전환
```bash
# teleop으로 제어
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 다시 자동 모드로: Arduino 리셋 또는 잠시 대기
```

## ROS 토픽 구조

### 퍼블리시 토픽
```
/sensors/ultrasonic (sensor_msgs/Range)
  - 초음파 센서 거리 데이터
  
/robot/status (std_msgs/String)
  - 로봇 현재 상태
  - 예: "AUTO_FORWARD (1.23m)", "AUTO_AVOID (0.35m)"
```

### 서브스크라이브 토픽
```
/cmd_vel (geometry_msgs/Twist)
  - 속도 명령 (수동 제어)
```

## 시각화

### RViz 설정
```bash
rosrun rviz rviz
```

RViz 설정:
1. Fixed Frame: "ultrasonic_link"
2. Add -> Range
3. Topic: /sensors/ultrasonic

### rqt_plot
```bash
rqt_plot /sensors/ultrasonic/range
```

## 장애물 회피 알고리즘

```cpp
if (distance < 0.3m) {
    // 정지
    stop();
} else if (distance < 0.5m) {
    // 좌회전 (장애물 회피)
    turnLeft();
} else {
    // 전진
    moveForward();
}
```

## PID 파라미터
현재 설정된 값:
- Kp = 2.0
- Ki = 0.5
- Kd = 0.1

필요시 코드에서 조정 가능합니다.

## 테스트 절차

### 1. 하드웨어 테스트
```bash
# Lab 1-2 예제로 각 센서/모터 개별 테스트
```

### 2. 센서 캘리브레이션
```bash
# 센서 데이터 확인
rostopic echo /sensors/ultrasonic

# 다양한 거리에서 측정값 확인
```

### 3. 자율주행 테스트
```bash
# rosserial 실행 후 로봇을 바닥에 놓기
# 로봇이 전진하다가 장애물을 만나면 회피하는지 확인
```

### 4. 안전 테스트
- 로봇이 책상에서 떨어지지 않도록 주의
- 비상 정지를 위해 USB 케이블을 빠르게 뽑을 준비

## 확장 아이디어

### 1. 다중 센서
```cpp
// 전면, 좌측, 우측 센서 추가
float front_distance = readUltrasonic(FRONT_TRIG, FRONT_ECHO);
float left_distance = readUltrasonic(LEFT_TRIG, LEFT_ECHO);
float right_distance = readUltrasonic(RIGHT_TRIG, RIGHT_ECHO);

// 가장 가까운 장애물 방향으로 회피
```

### 2. 벽 따라가기 (Wall Following)
```cpp
// 왼쪽 벽과 일정 거리 유지
float target_wall_distance = 0.3;
float error = target_wall_distance - left_distance;
// PID로 조향 제어
```

### 3. 라인 트레이싱
```cpp
// IR 센서 배열로 라인 감지
// PID로 라인 중앙 유지
```

### 4. SLAM (ROS 패키지 활용)
```bash
# Hector SLAM 또는 GMapping 사용
roslaunch hector_slam_launch tutorial.launch
```

## 문제 해결

### 로봇이 회전만 함
- 모터 방향 확인 (IN1/IN2, IN3/IN4 교체)
- 양쪽 바퀴 속도 균형 확인

### 센서 값이 불안정
- 초음파 센서 배치 확인 (장애물과 수직)
- 이동 평균 필터 추가 고려

### ROS 연결 끊김
- USB 케이블 품질 확인
- rosserial 재시작

## 성능 최적화
- 제어 주파수 조정 (현재 20Hz)
- PID 파라미터 튜닝
- 센서 노이즈 필터링

## 다음 단계
- ROS Navigation Stack 통합
- SLAM을 활용한 맵핑
- AI 기반 경로 계획
- 비전 센서 (카메라) 추가

## 안전 주의사항
⚠️ 로봇을 테이블 위에서 테스트할 때 낙하 방지
⚠️ 모터 과열 주의 (장시간 운영 시 휴식)
⚠️ 배터리 전압 확인 (7.4V~12V 권장)
