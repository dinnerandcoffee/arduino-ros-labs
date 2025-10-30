# 문제 해결 가이드 (Troubleshooting Guide)

## 목차
1. [Arduino 관련 문제](#arduino-관련-문제)
2. [ROS/rosserial 문제](#rosrosserial-문제)
3. [하드웨어 문제](#하드웨어-문제)
4. [센서 문제](#센서-문제)
5. [모터 제어 문제](#모터-제어-문제)
6. [PID 제어 문제](#pid-제어-문제)
7. [일반적인 오류 메시지](#일반적인-오류-메시지)

---

## Arduino 관련 문제

### Arduino가 인식되지 않음

**증상**: `/dev/ttyACM0` 또는 `/dev/ttyUSB0` 포트가 보이지 않음

**해결 방법**:
```bash
# 1. USB 디바이스 확인
lsusb
# "Arduino" 또는 "USB Serial" 찾기

# 2. dmesg로 연결 확인
dmesg | tail -20

# 3. 포트 권한 설정
sudo chmod 666 /dev/ttyACM0

# 4. 사용자를 dialout 그룹에 추가 (영구적)
sudo usermod -a -G dialout $USER
# 로그아웃 후 재로그인
```

**추가 확인**:
- USB 케이블이 데이터 전송 가능한지 확인 (충전 전용 X)
- 다른 USB 포트 시도
- Arduino 보드의 LED 확인 (전원 LED가 켜져야 함)

### 업로드 실패: "avrdude: stk500_recv(): programmer is not responding"

**원인**: 포트가 다른 프로그램에 의해 사용 중

**해결 방법**:
```bash
# 1. rosserial 실행 중이면 종료
# Ctrl+C로 종료

# 2. 포트를 사용하는 프로세스 확인
lsof /dev/ttyACM0

# 3. 해당 프로세스 종료
kill <PID>

# 4. Arduino 리셋 버튼 누르고 바로 업로드
```

### 컴파일 오류: "ros.h: No such file or directory"

**원인**: ros_lib 라이브러리가 설치되지 않음

**해결 방법**:
```bash
# ros_lib 라이브러리 생성
cd ~/Arduino/libraries
rm -rf ros_lib  # 기존 것이 있다면 삭제
rosrun rosserial_arduino make_libraries.py .

# Arduino IDE 재시작
```

### 메모리 부족: "Low memory available"

**원인**: 프로그램이 Arduino 메모리 초과

**해결 방법**:
1. 불필요한 변수 제거
2. `F()` 매크로로 문자열을 프로그램 메모리에 저장:
```cpp
Serial.println(F("Hello"));  // RAM 대신 Flash 메모리 사용
```
3. 배열 크기 줄이기
4. Arduino Mega 사용 고려 (더 큰 메모리)

---

## ROS/rosserial 문제

### rosserial 연결 실패

**증상**:
```
Unable to sync with device
Lost sync with device
```

**해결 방법**:
```bash
# 1. Arduino 리셋
# 리셋 버튼 누르기

# 2. Baud rate 확인 (기본: 57600)
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=57600

# 3. rosserial 재시작
# Ctrl+C로 종료 후 재실행

# 4. USB 케이블 교체
```

**Arduino 코드 확인**:
```cpp
void setup() {
  nh.initNode();  // 이 줄이 있는지 확인
}

void loop() {
  nh.spinOnce();  // 이 줄이 있는지 확인
  delay(10);      // 짧은 delay 추가
}
```

### 토픽이 나타나지 않음

**확인 방법**:
```bash
# 토픽 목록 확인
rostopic list

# rosserial 노드 확인
rosnode list

# rosserial 로그 확인
rosnode info serial_node
```

**해결 방법**:
1. `nh.advertise()` 또는 `nh.subscribe()` 호출 확인
2. Arduino 재업로드
3. rosserial 재시작

### 메시지 전송이 느리거나 끊김

**원인**: Baud rate가 낮거나 메시지가 너무 큼

**해결 방법**:
```cpp
// Arduino 코드에서 Baud rate 증가
nh.getHardware()->setBaud(115200);

// ROS에서도 동일하게 설정
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
```

---

## 하드웨어 문제

### GND 연결 누락

**증상**: 센서 값이 불안정하거나 모터가 제대로 동작하지 않음

**해결 방법**:
- Arduino GND와 L298N GND 반드시 연결
- 모든 센서의 GND도 Arduino GND에 연결
- 배터리 GND도 공통 접지

```
[배터리 -] --- [L298N GND] --- [Arduino GND] --- [센서 GND]
```

### 전원 부족

**증상**:
- Arduino가 재부팅됨
- 모터가 약하게 돌거나 멈춤
- 브라운아웃 (brownout)

**해결 방법**:
1. 모터 전원과 Arduino 전원 분리
2. 배터리 전압 확인 (7.4V 이상)
3. 충분한 용량의 배터리 사용 (2200mAh 이상)
4. 커패시터 추가 (전원 안정화)

### 배선 접촉 불량

**확인 방법**:
```bash
# 멀티미터로 전압 측정
# VCC: 5V
# GND: 0V
# 신호선: 0-5V 변화
```

**해결 방법**:
- 점퍼선 재연결
- 헐거운 연결 납땜
- 품질 좋은 듀폰 선 사용

---

## 센서 문제

### 초음파 센서 값이 이상함

**증상**:
- 항상 0 또는 최대값 반환
- 값이 심하게 흔들림
- 측정 안 됨

**해결 방법**:

1. **배선 확인**:
```
VCC  -> 5V
TRIG -> Pin 9 (출력)
ECHO -> Pin 10 (입력)
GND  -> GND
```

2. **센서 방향 확인**:
- 장애물과 수직
- 부드러운 표면은 반사 약함
- 측정 거리: 2cm ~ 400cm

3. **코드 확인**:
```cpp
// 타임아웃 추가
duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms 타임아웃
if (duration == 0) {
  return 4.0;  // 최대 범위 반환
}
```

4. **노이즈 필터링**:
```cpp
// 이동 평균 필터
float readUltrasonicFiltered() {
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += readUltrasonic();
    delay(10);
  }
  return sum / 5.0;
}
```

### IR 센서가 반응하지 않음

**확인**:
```cpp
void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
}

void loop() {
  int value = digitalRead(IR_PIN);
  Serial.println(value);  // 0 또는 1 출력 확인
  delay(100);
}
```

**조정**:
- 센서의 가변저항으로 감도 조정
- 장애물과의 거리 조정 (보통 2-30cm)

---

## 모터 제어 문제

### 모터가 전혀 회전하지 않음

**체크리스트**:
- [ ] L298N에 전원 공급 (7-12V)
- [ ] ENA, ENB 점퍼 캡 제거 (PWM 제어용)
- [ ] Arduino GND와 L298N GND 연결
- [ ] 모터 연결 확인
- [ ] 코드에서 PWM 값 0이 아닌지 확인

**테스트 코드**:
```cpp
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  analogWrite(ENA, 200);  // 속도 설정
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void loop() {
  // 모터가 계속 회전해야 함
}
```

### 모터가 한쪽만 회전함

**원인**: 해당 모터 또는 드라이버 채널 문제

**확인**:
1. 모터 교체해서 테스트
2. 배선 재확인
3. L298N 출력 전압 측정
4. 드라이버 IC 과열 확인

### 모터 방향이 반대

**해결 방법**:
```cpp
// 방법 1: 하드웨어 - 모터 전선 바꾸기
// OUT1 <-> OUT2

// 방법 2: 소프트웨어 - 코드 수정
void moveForward(int speed) {
  // IN1과 IN2를 반대로
  digitalWrite(IN1, LOW);   // 원래 HIGH
  digitalWrite(IN2, HIGH);  // 원래 LOW
}
```

### 모터가 약하게 회전

**원인**:
- PWM 값이 너무 낮음
- 전원 부족
- 모터 부하 과다

**해결 방법**:
```cpp
// PWM 값 증가
analogWrite(ENA, 255);  // 최대 속도

// 배터리 전압 확인
// 7.4V 이상 필요
```

---

## PID 제어 문제

### 진동이 심함 (Oscillation)

**원인**: Kp가 너무 크거나 Kd가 너무 작음

**해결 방법**:
```cpp
// Kp 감소
Kp = Kp * 0.8;

// Kd 증가
Kd = Kd * 1.2;
```

### 응답이 느림 (Slow Response)

**원인**: Kp가 너무 작음

**해결 방법**:
```cpp
// Kp 증가
Kp = Kp * 1.2;
```

### 정상 상태 오차 (Steady-State Error)

**원인**: Ki가 너무 작거나 0

**해결 방법**:
```cpp
// Ki 증가
Ki = 0.5;  // 0에서 시작했다면

// Integral windup 방지
left_integral = constrain(left_integral, -100, 100);
```

### PID 출력이 불안정

**확인 사항**:
```cpp
// dt (샘플링 시간) 확인
// 너무 크거나 작지 않아야 함
float dt = (millis() - last_time) / 1000.0;

// dt가 0이 되지 않도록
if (dt < 0.001) return;

// 미분항 노이즈 필터
// 로우패스 필터 추가
derivative = 0.8 * derivative + 0.2 * (error - last_error) / dt;
```

---

## 일반적인 오류 메시지

### "expected initializer before 'void'"

**원인**: 세미콜론 누락 또는 중괄호 불일치

**해결 방법**: 이전 줄에 `;` 추가 또는 `{`, `}` 확인

### "was not declared in this scope"

**원인**: 변수 선언 누락 또는 오타

**해결 방법**: 변수명 확인 및 선언 추가

### "error: 'sensor_msgs' does not name a type"

**원인**: ros_lib 헤더 파일 누락

**해결 방법**:
```cpp
#include <ros.h>
#include <sensor_msgs/Range.h>  // 이 줄 추가
```

### "Serial port busy"

**원인**: 다른 프로그램이 포트 사용 중

**해결 방법**:
```bash
# Arduino IDE의 Serial Monitor 닫기
# rosserial 종료
# 다른 터미널의 screen/minicom 종료
```

---

## 디버깅 팁

### 시리얼 모니터 활용
```cpp
void setup() {
  Serial.begin(9600);
  Serial.println("Setup started");
}

void loop() {
  int value = analogRead(A0);
  Serial.print("Value: ");
  Serial.println(value);
  delay(100);
}
```

### LED 디버깅
```cpp
const int DEBUG_LED = 13;

void setup() {
  pinMode(DEBUG_LED, OUTPUT);
}

// 오류 발생 시 LED 켜기
if (error_condition) {
  digitalWrite(DEBUG_LED, HIGH);
}
```

### ROS 디버깅
```bash
# 토픽 데이터 확인
rostopic echo /topic_name

# 토픽 주파수 확인
rostopic hz /topic_name

# 노드 정보
rosnode info node_name

# 파라미터 확인
rosparam list
rosparam get /param_name
```

---

## 도움 받기

### 커뮤니티
- **ROS Answers**: https://answers.ros.org
- **Arduino Forum**: https://forum.arduino.cc
- **Stack Overflow**: https://stackoverflow.com (태그: arduino, ros)

### 문제 보고 시 포함할 정보
1. 사용 중인 하드웨어 (Arduino 모델, 센서 등)
2. 소프트웨어 버전 (ROS 버전, Arduino IDE 버전)
3. 정확한 오류 메시지
4. 시도한 해결 방법
5. 관련 코드 (최소한의 재현 가능한 예제)

### 체계적 디버깅
1. **문제 격리**: 최소한의 코드로 문제 재현
2. **이분 탐색**: 코드를 절반씩 주석 처리하며 원인 찾기
3. **비교**: 동작하는 예제와 비교
4. **로깅**: 의심되는 부분에 Serial.print 추가
5. **휴식**: 잠시 쉬고 새로운 관점에서 접근

---

**문제가 해결되지 않으면 GitHub Issues에 질문해 주세요!**
