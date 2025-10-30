# Lab 6: PID 제어

## 목표
PID 제어를 사용하여 모터 속도를 정확하게 제어하는 방법을 학습합니다.

## PID 제어란?
PID(Proportional-Integral-Derivative)는 피드백 제어 시스템으로, 목표값과 현재값의 차이를 최소화합니다.

### PID 수식
```
output = Kp × error + Ki × ∫error dt + Kd × d(error)/dt

여기서:
- error = 목표값 - 현재값
- Kp: 비례 게인 (현재 오차에 비례)
- Ki: 적분 게인 (누적 오차 보정)
- Kd: 미분 게인 (오차 변화율 대응)
```

## 필요 하드웨어
- Arduino Uno
- L298N 모터 드라이버
- DC 모터 2개 (인코더 포함)
- 로터리 인코더 2개

## 인코더 연결
```
왼쪽 인코더 A상 -> Pin 2 (INT0)
오른쪽 인코더 A상 -> Pin 18/A4 (INT5)
```

## 예제 1: 기본 PID 모터 제어

### Arduino 업로드
`pid_motor_control.ino` 파일을 업로드

### ROS 실행
```bash
# rosserial 실행
rosrun rosserial_python serial_node.py /dev/ttyACM0

# cmd_vel로 속도 명령
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" -r 10

# 모터 속도 모니터링
rostopic echo /motor/left_speed
rostopic echo /motor/right_speed
```

### 속도 시각화
```bash
# rqt_plot으로 실시간 그래프
rqt_plot /motor/left_speed /motor/right_speed
```

## 예제 2: PID 튜닝 도구

### Arduino 업로드
`pid_tuning.ino` 파일을 업로드

### PID 파라미터 조정
```bash
# Kp=3.0, Ki=0.8, Kd=0.2로 설정
rostopic pub /pid_params std_msgs/Float32MultiArray "data: [3.0, 0.8, 0.2]"

# PID 상태 확인
rostopic echo /pid_status
```

## PID 튜닝 가이드

### 1단계: P 게인 조정 (Ki=0, Kd=0)
- Kp를 0에서 시작하여 점진적으로 증가
- 응답이 빨라지지만 진동 시작 전까지만
- **목표**: 빠른 응답, 약간의 오버슈트 허용

### 2단계: I 게인 추가
- Ki를 작은 값부터 증가
- 정상 상태 오차(steady-state error) 제거
- **주의**: 너무 크면 진동 발생

### 3단계: D 게인 추가
- Kd를 추가하여 오버슈트 감소
- 시스템 안정화
- **주의**: 노이즈에 민감할 수 있음

### Ziegler-Nichols 방법
1. Ki=0, Kd=0으로 설정
2. Kp를 증가시켜 시스템이 지속적으로 진동하는 값(Ku) 찾기
3. 진동 주기(Tu) 측정
4. 다음 공식 적용:
   ```
   Kp = 0.6 × Ku
   Ki = 2 × Kp / Tu
   Kd = Kp × Tu / 8
   ```

## 실전 튜닝 예시
```bash
# 초기값 (보수적)
rostopic pub /pid_params std_msgs/Float32MultiArray "data: [1.0, 0.0, 0.0]"

# P 게인 증가
rostopic pub /pid_params std_msgs/Float32MultiArray "data: [2.0, 0.0, 0.0]"

# I 게인 추가
rostopic pub /pid_params std_msgs/Float32MultiArray "data: [2.0, 0.5, 0.0]"

# D 게인 추가
rostopic pub /pid_params std_msgs/Float32MultiArray "data: [2.0, 0.5, 0.1]"

# 미세 조정
rostopic pub /pid_params std_msgs/Float32MultiArray "data: [2.5, 0.6, 0.15]"
```

## 성능 평가 지표
- **정착 시간** (Settling Time): 목표값 ±5% 이내 도달 시간
- **오버슈트** (Overshoot): 목표값 초과 정도
- **정상 상태 오차** (Steady-State Error): 최종 오차

## Anti-Windup
적분항이 무한정 증가하는 것을 방지:
```cpp
left_integral = constrain(left_integral, -100, 100);
```

## 문제 해결
- **진동이 심함**: Kp 또는 Kd 감소
- **응답이 느림**: Kp 증가
- **정상 상태 오차 존재**: Ki 증가
- **노이즈 발생**: Kd 감소 또는 필터 추가

## 응용 과제
1. 목표 속도 단계 변화에 대한 응답 테스트
2. 부하 변화에 대한 강인성 테스트
3. 다양한 튜닝 방법 비교

## 다음 단계
Lab 7에서는 모든 것을 통합하여 자율주행 예제를 만듭니다.
