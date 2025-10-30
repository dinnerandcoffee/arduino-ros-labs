# Lab 5: 모터 Subscriber

## 목표
ROS에서 전송한 속도 명령을 받아 모터를 제어하는 방법을 학습합니다.

## 하드웨어 연결
Lab 2와 동일한 L298N 모터 드라이버 연결 사용

## 예제 1: 기본 cmd_vel Subscriber

### Arduino 업로드
`motor_subscriber.ino` 파일을 Arduino에 업로드

### ROS 실행
```bash
# 터미널 1: roscore
roscore

# 터미널 2: rosserial
rosrun rosserial_python serial_node.py /dev/ttyACM0

# 터미널 3: 키보드 제어 (teleop 패키지 필요)
sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 수동 명령 전송
```bash
# 전진
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# 정지
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# 회전
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

## 예제 2: 인코더가 있는 모터 제어

### 하드웨어 추가
```
왼쪽 인코더 A상 -> Pin 2 (인터럽트)
오른쪽 인코더 A상 -> Pin 3 (인터럽트)
```

### Arduino 업로드
`motor_with_encoder.ino` 파일을 Arduino에 업로드

### 인코더 데이터 확인
```bash
# 인코더 카운트 확인
rostopic echo /encoder/left
rostopic echo /encoder/right
```

## geometry_msgs/Twist 메시지 구조
```
geometry_msgs/Vector3 linear
  float64 x    # 전진/후진 속도 (m/s)
  float64 y    # 좌우 이동 (사용 안 함)
  float64 z    # 상하 이동 (사용 안 함)
geometry_msgs/Vector3 angular
  float64 x    # 롤 (사용 안 함)
  float64 y    # 피치 (사용 안 함)
  float64 z    # 요 (회전 속도, rad/s)
```

## 차동 구동 (Differential Drive) 원리
```
left_speed = linear - angular
right_speed = linear + angular

예시:
- 전진: linear=1.0, angular=0.0 → left=1.0, right=1.0
- 좌회전: linear=0.0, angular=0.5 → left=-0.5, right=0.5
- 우회전: linear=0.0, angular=-0.5 → left=0.5, right=-0.5
- 전진+좌회전: linear=0.5, angular=0.3 → left=0.2, right=0.8
```

## 키보드 제어 키 매핑
```
teleop_twist_keyboard:
   i: 전진
   ,: 후진
   j: 좌회전
   l: 우회전
   k: 정지
   q/z: 속도 증가/감소
```

## 테스트 절차
1. rosserial 연결 확인
2. cmd_vel 토픽 publish 테스트
3. 모터 동작 확인
4. teleop으로 실시간 제어
5. 인코더 값 확인 (해당하는 경우)

## 문제 해결
- **모터가 반대로 회전**: IN1/IN2 또는 IN3/IN4 로직 반전
- **한쪽 모터만 동작**: 배선 및 PWM 핀 확인
- **반응 없음**: rosserial 연결 및 토픽 이름 확인

## 다음 단계
Lab 6에서는 PID 제어를 구현합니다.
