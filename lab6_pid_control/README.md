# Lab 6: PID 제어 (PID Control)

## 목표
PID 제어 알고리즘을 이해하고 실제 하드웨어에 적용하여 폐루프 제어를 구현합니다.

## PID 제어란?

PID는 Proportional-Integral-Derivative의 약자로, 피드백 제어 시스템에서 가장 널리 사용되는 제어 알고리즘입니다.

### PID 수식
```
output = Kp * error + Ki * integral + Kd * derivative

error = setpoint - current_value
integral = sum(error * dt)
derivative = (error - last_error) / dt
```

### 각 항의 역할

#### P (Proportional) - 비례 항
- 현재 오차에 비례하여 제어
- 빠른 응답을 제공하지만 정상 상태 오차 발생 가능
- Kp가 너무 크면 진동(oscillation) 발생

#### I (Integral) - 적분 항
- 누적된 오차를 제거
- 정상 상태 오차를 제거하지만 느린 응답
- Ki가 너무 크면 오버슈트 발생

#### D (Derivative) - 미분 항
- 오차의 변화율에 반응
- 오버슈트를 줄이고 안정성 향상
- 노이즈에 민감할 수 있음

## 실습 내용

### 1. 기본 PID 제어기
- **파일**: `basic_pid.ino`
- **기능**: PID 알고리즘의 기본 구현
- **용도**: 학습 및 테스트용

#### 주요 파라미터
```cpp
float Kp = 2.0;  // 비례 게인
float Ki = 0.5;  // 적분 게인
float Kd = 0.1;  // 미분 게인
```

### 2. 모터 속도 PID 제어
- **파일**: `motor_speed_pid.ino`
- **기능**: 엔코더를 사용한 모터 속도 제어
- **센서**: 로터리 엔코더

#### 핀 연결
**Motor:**
- ENA → Pin 9 (PWM)
- IN1 → Pin 8
- IN2 → Pin 7

**Encoder:**
- Channel A → Pin 2 (Interrupt)
- Channel B → Pin 3 (Interrupt)

#### 사용법
1. 시리얼 모니터에서 목표 속도(RPM) 입력
2. PID 제어기가 자동으로 속도 조절
3. 실시간 속도와 PWM 값 확인

### 3. 라인 트레이싱 PID
- **파일**: `line_following_pid.ino`
- **기능**: PID를 사용한 라인 트레이싱
- **센서**: 5채널 IR 라인 센서 어레이

#### 핀 연결
**Line Sensors:**
- Sensor 0 (최좌) → A0
- Sensor 1 → A1
- Sensor 2 (중앙) → A2
- Sensor 3 → A3
- Sensor 4 (최우) → A4

**Motors:**
- Left Motor: ENA→Pin 9, IN1→Pin 8, IN2→Pin 7
- Right Motor: ENB→Pin 3, IN3→Pin 5, IN4→Pin 4

#### PID 튜닝
```cpp
float Kp = 0.8;  // 라인 추종 민감도
float Ki = 0.0;  // 보통 0으로 시작
float Kd = 0.5;  // 진동 감쇠
```

### 4. ROS 통합 PID 제어
- **파일**: `motor_speed_pid_ros.ino`
- **기능**: ROS로 목표 속도 설정 및 피드백
- **토픽**:
  - `/target_speed` (구독, Float32): 목표 RPM
  - `/current_speed` (발행, Float32): 현재 RPM

#### ROS 명령
```bash
# 목표 속도 설정
rostopic pub /target_speed std_msgs/Float32 "data: 100.0"

# 현재 속도 모니터링
rostopic echo /current_speed

# 속도 그래프
rqt_plot /target_speed/data /current_speed/data
```

## PID 튜닝 가이드

### Ziegler-Nichols 방법
1. Ki = 0, Kd = 0으로 설정
2. Kp를 천천히 증가시켜 지속적 진동이 발생하는 임계값 Ku 찾기
3. 진동 주기 Tu 측정
4. 다음 공식 사용:
   - Kp = 0.6 * Ku
   - Ki = 2 * Kp / Tu
   - Kd = Kp * Tu / 8

### 수동 튜닝 방법
1. **P 조정**: Kp부터 시작, 빠른 응답이 나올 때까지 증가
2. **D 조정**: 오버슈트가 있다면 Kd 증가
3. **I 조정**: 정상 상태 오차가 있다면 Ki 추가

### 튜닝 팁
- 한 번에 하나의 파라미터만 조정
- 작은 값부터 시작하여 점진적으로 증가
- 변화를 관찰하고 기록
- Anti-windup 적용 (적분 항 제한)

## 고급 주제

### Anti-windup
적분 항이 너무 커지는 것을 방지:
```cpp
integral = constrain(integral, -maxIntegral, maxIntegral);
```

### 샘플링 시간
일정한 샘플링 시간 유지가 중요:
```cpp
const unsigned long sampleTime = 100;  // 100ms
if (timeChange >= sampleTime) {
  // PID 계산
}
```

### 노이즈 필터링
미분 항에 로우패스 필터 적용:
```cpp
derivative = alpha * derivative + (1 - alpha) * newDerivative;
```

## 응용 과제
1. 온도 제어 시스템 구현
2. 로봇 팔 위치 제어
3. 균형 로봇 (Balancing Robot)
4. 크루즈 컨트롤 시스템

## 학습 내용
1. PID 제어 알고리즘 이해
2. 엔코더를 사용한 피드백 시스템
3. PID 파라미터 튜닝 방법
4. Anti-windup 및 노이즈 대책
5. 폐루프 제어의 장점

## 참고 자료
- [PID Control Theory](https://en.wikipedia.org/wiki/PID_controller)
- [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library)
- Ziegler-Nichols 튜닝 방법

## 다음 단계
이제 모든 기본 실습을 완료했습니다! 이제 자신만의 자율주행 로봇 프로젝트를 시작할 수 있습니다.
