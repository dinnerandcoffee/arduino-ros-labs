# 빠른 참조 가이드 (Quick Reference)

## ROS 명령어

### roscore
```bash
roscore                    # ROS 마스터 시작
```

### rosserial
```bash
# 기본 연결
rosrun rosserial_python serial_node.py /dev/ttyACM0

# Baud rate 지정
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200

# 다른 포트
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

### rostopic
```bash
rostopic list              # 토픽 목록
rostopic echo /topic       # 토픽 데이터 보기
rostopic hz /topic         # 토픽 주파수
rostopic info /topic       # 토픽 정보
rostopic pub /topic        # 토픽 발행

# cmd_vel 발행 예시
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" -r 10
```

### rosnode
```bash
rosnode list               # 노드 목록
rosnode info /node_name    # 노드 정보
rosnode kill /node_name    # 노드 종료
```

### roslaunch
```bash
roslaunch package file.launch              # launch 파일 실행
roslaunch arduino_ros_labs basic_serial.launch
```

---

## Arduino 핀 매핑

### PWM 핀 (Uno)
- **3, 5, 6, 9, 10, 11**: analogWrite() 사용 가능

### 인터럽트 핀 (Uno)
- **2 (INT0)**: attachInterrupt(0, ...)
- **3 (INT1)**: attachInterrupt(1, ...)

### 아날로그 핀
- **A0-A5**: analogRead() 사용

### 시리얼 핀
- **0 (RX), 1 (TX)**: Serial 통신 (USB 업로드 시 사용 주의)

---

## 회로 연결 요약

### L298N 연결
```
Arduino  ->  L298N
Pin 3    ->  ENA (PWM)
Pin 5    ->  IN1
Pin 6    ->  IN2
Pin 9    ->  ENB (PWM)
Pin 10   ->  IN3
Pin 11   ->  IN4
GND      ->  GND
```

### 센서 연결
```
HC-SR04:
5V  -> VCC
9   -> TRIG
10  -> ECHO
GND -> GND

IR:
5V  -> VCC
7   -> OUT
GND -> GND
```

### 인코더
```
왼쪽: Pin 2 (INT0)
오른쪽: Pin 18/A4
```

---

## Arduino 코드 템플릿

### 기본 구조
```cpp
#include <ros.h>

ros::NodeHandle nh;

void setup() {
  nh.initNode();
  // 초기화 코드
}

void loop() {
  // 메인 코드
  nh.spinOnce();
  delay(10);
}
```

### Publisher
```cpp
#include <std_msgs/Float32.h>

std_msgs::Float32 msg;
ros::Publisher pub("topic", &msg);

void setup() {
  nh.initNode();
  nh.advertise(pub);
}

void loop() {
  msg.data = value;
  pub.publish(&msg);
  nh.spinOnce();
  delay(100);
}
```

### Subscriber
```cpp
void callback(const std_msgs::Float32& msg) {
  float value = msg.data;
  // 처리
}

ros::Subscriber<std_msgs::Float32> sub("topic", &callback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
```

---

## 모터 제어 함수

### 기본 모터 제어
```cpp
void setMotor(int en, int in1, int in2, int pwm) {
  if (pwm >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -pwm);
  }
}

// 사용
setMotor(ENA, IN1, IN2, 200);  // 전진 속도 200
setMotor(ENA, IN1, IN2, -200); // 후진 속도 200
```

### 이동 함수
```cpp
void moveForward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
```

---

## 센서 읽기

### 초음파 센서
```cpp
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 4.0;
  
  return duration * 0.034 / 2 / 100.0;  // 미터
}
```

### IR 센서
```cpp
bool detectObstacle() {
  return digitalRead(IR_PIN) == LOW;
}
```

---

## PID 제어 코드

### 기본 PID
```cpp
float Kp = 2.0, Ki = 0.5, Kd = 0.1;
float error, last_error = 0, integral = 0;

int pidControl(float target, float current, float dt) {
  error = target - current;
  integral += error * dt;
  integral = constrain(integral, -100, 100);
  float derivative = (error - last_error) / dt;
  
  int output = Kp * error + Ki * integral + Kd * derivative;
  last_error = error;
  
  return constrain(output, -255, 255);
}
```

---

## ROS 메시지 타입

### geometry_msgs/Twist
```cpp
geometry_msgs::Twist msg;
msg.linear.x = 0.5;   // 전진/후진 (m/s)
msg.angular.z = 0.3;  // 회전 (rad/s)
```

### sensor_msgs/Range
```cpp
sensor_msgs::Range msg;
msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
msg.min_range = 0.02;
msg.max_range = 4.0;
msg.range = distance;
```

### std_msgs
```cpp
std_msgs::Float32 f_msg;
f_msg.data = 3.14;

std_msgs::Int32 i_msg;
i_msg.data = 42;

std_msgs::Bool b_msg;
b_msg.data = true;

std_msgs::String s_msg;
s_msg.data = "hello";
```

---

## 타이머 패턴

### millis() 기반
```cpp
unsigned long last_time = 0;
unsigned long interval = 100;  // ms

void loop() {
  if (millis() - last_time > interval) {
    // 100ms마다 실행
    
    last_time = millis();
  }
}
```

### 다중 타이머
```cpp
unsigned long timer1 = 0, timer2 = 0;

void loop() {
  if (millis() - timer1 > 50) {  // 20Hz
    // 센서 읽기
    timer1 = millis();
  }
  
  if (millis() - timer2 > 100) {  // 10Hz
    // 데이터 전송
    timer2 = millis();
  }
}
```

---

## 디버깅

### 시리얼 출력
```cpp
Serial.begin(9600);
Serial.print("Value: ");
Serial.println(value);

// ROS와 함께 사용 시 주의
// ros::NodeHandle과 Serial은 동시 사용 불가
```

### LED 디버깅
```cpp
digitalWrite(13, HIGH);  // 내장 LED 켜기
delay(100);
digitalWrite(13, LOW);   // 끄기
```

---

## 유용한 함수

### constrain
```cpp
int value = constrain(input, 0, 255);  // 0-255 범위로 제한
```

### map
```cpp
int output = map(input, 0, 1023, 0, 255);  // 범위 변환
```

### abs
```cpp
int absolute = abs(-42);  // 42
```

---

## 포트 권한 (Ubuntu)

```bash
# 임시
sudo chmod 666 /dev/ttyACM0

# 영구
sudo usermod -a -G dialout $USER
# 로그아웃 후 재로그인
```

---

## 주파수 설정

| 작업 | 권장 주파수 |
|------|------------|
| 센서 읽기 | 10-20Hz |
| PID 제어 | 20-50Hz |
| ROS 퍼블리시 | 5-10Hz |
| 메인 루프 | 100Hz+ |

---

## 일반적인 값

### PWM
- 최소 동작: ~50-70 (모터에 따라 다름)
- 중간 속도: 150
- 최대 속도: 255

### cmd_vel
- linear.x: -1.0 ~ 1.0 (m/s)
- angular.z: -1.0 ~ 1.0 (rad/s)

### PID 초기값
- Kp: 1.0 - 3.0
- Ki: 0.1 - 1.0
- Kd: 0.01 - 0.3

---

## 체크리스트

### 시작 전
- [ ] 배선 확인
- [ ] 전원 확인
- [ ] USB 연결
- [ ] 코드 업로드

### ROS 실행
- [ ] roscore 실행
- [ ] rosserial 실행
- [ ] 토픽 확인 (rostopic list)
- [ ] 데이터 확인 (rostopic echo)

### 문제 발생 시
- [ ] 시리얼 모니터로 확인
- [ ] 배선 재확인
- [ ] Arduino 리셋
- [ ] rosserial 재시작

---

## 참고 링크

- ROS Wiki: http://wiki.ros.org
- Arduino Reference: https://www.arduino.cc/reference/en
- rosserial: http://wiki.ros.org/rosserial_arduino

---

**이 참조 가이드를 인쇄하여 책상 옆에 두고 사용하세요!**
