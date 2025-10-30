# Quick Reference Guide

빠르게 참조할 수 있는 핵심 정보 모음

## 📌 핀 배치 요약

### Lab 1 & 4: 센서
| 센서 | 핀 | 타입 |
|------|-----|------|
| 초음파 Trig | 9 | Digital |
| 초음파 Echo | 10 | Digital |
| IR 센서 | 7 | Digital |

### Lab 2 & 5: 모터
| 모터 | 핀 | 타입 |
|------|-----|------|
| 왼쪽 모터 PWM | 9 | PWM |
| 왼쪽 모터 IN1 | 8 | Digital |
| 왼쪽 모터 IN2 | 7 | Digital |
| 오른쪽 모터 PWM | 3 | PWM |
| 오른쪽 모터 IN3 | 5 | Digital |
| 오른쪽 모터 IN4 | 4 | Digital |
| 서보 모터 | 6 | PWM |

### Lab 6: PID 제어
| 장치 | 핀 | 타입 |
|------|-----|------|
| 엔코더 A | 2 | Interrupt |
| 엔코더 B | 3 | Interrupt |
| 라인센서 1-5 | A0-A4 | Analog |

## 🔌 전원 연결

### Arduino Uno
- **입력 전압**: 7-12V (권장 9V)
- **USB 전원**: 5V
- **5V 핀 출력**: 최대 500mA
- **각 I/O 핀**: 최대 40mA

### L298N 모터 드라이버
- **입력 전압**: 5-35V (권장 7-12V)
- **출력 전류**: 최대 2A (각 채널)
- **5V 레귤레이터**: 최대 1A (12V 입력시)

### 센서
- **HC-SR04**: 5V, 15mA
- **IR 센서**: 5V, 20mA
- **서보 SG90**: 5V, 100-300mA

## 📝 ROS 명령어 치트시트

### roscore
```bash
roscore  # ROS 마스터 시작
```

### rosserial
```bash
# 기본 연결
rosrun rosserial_python serial_node.py /dev/ttyUSB0

# baud rate 지정
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=57600
```

### rostopic
```bash
# 토픽 목록
rostopic list

# 토픽 내용 출력
rostopic echo /topic_name

# 토픽 발행
rostopic pub /motor_cmd std_msgs/Int16 "data: 255"

# 주파수 확인
rostopic hz /topic_name

# 토픽 정보
rostopic info /topic_name
```

### rqt 도구
```bash
rqt_graph    # 노드 그래프
rqt_plot     # 데이터 플롯
rqt_console  # 로그 확인
```

## 🔧 Arduino IDE 단축키

| 기능 | Windows/Linux | macOS |
|------|---------------|-------|
| 업로드 | Ctrl+U | Cmd+U |
| 컴파일 | Ctrl+R | Cmd+R |
| 시리얼 모니터 | Ctrl+Shift+M | Cmd+Shift+M |
| 저장 | Ctrl+S | Cmd+S |

## 💾 Arduino 메모리

### Arduino Uno
- **Flash**: 32KB (프로그램 저장)
- **SRAM**: 2KB (변수 저장)
- **EEPROM**: 1KB (영구 저장)

### 메모리 절약 팁
```cpp
// 문자열을 PROGMEM에 저장
Serial.println(F("Hello World"));

// 전역 변수 최소화
// Serial.print 줄이기
```

## 📊 PID 튜닝 가이드

### 초기값 (권장)
```cpp
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
```

### 단계별 튜닝
1. **P 조정**: Kp 증가 → 빠른 응답, 진동 관찰
2. **D 조정**: 진동 감소용, Kd 추가
3. **I 조정**: 정상 상태 오차 제거, Ki 천천히 추가

### 현상별 대처
| 현상 | 조치 |
|------|------|
| 느린 응답 | Kp 증가 |
| 진동 (Oscillation) | Kp 감소, Kd 증가 |
| 정상 상태 오차 | Ki 증가 |
| 오버슈트 | Kd 증가, Kp/Ki 감소 |

## 🐛 문제 해결

### Arduino 업로드 실패
```bash
# 포트 권한
sudo chmod 666 /dev/ttyUSB0

# 사용자 그룹 추가
sudo usermod -a -G dialout $USER
# 재로그인 필요
```

### ROS 연결 실패
```bash
# 포트 확인
ls /dev/ttyUSB* /dev/ttyACM*

# rosserial 재시작
rosrun rosserial_python serial_node.py /dev/ttyUSB0

# 시리얼 모니터 닫기 확인
```

### 모터 작동 안 함
- [ ] L298N 전원 LED 확인
- [ ] ENA/ENB 점퍼 제거 (PWM 사용시)
- [ ] 공통 접지(GND) 연결
- [ ] 모터 전원 전압 확인 (7-12V)

### 센서 값 이상
- [ ] VCC, GND 극성 확인
- [ ] 케이블 연결 상태
- [ ] 전압 측정 (멀티미터)

## 📚 주요 라이브러리

### Arduino 기본
```cpp
#include <Servo.h>        // 서보 모터
```

### rosserial
```cpp
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
```

### PID (선택)
```cpp
#include <PID_v1.h>  // Brett Beauregard's PID Library
```

## 🔗 유용한 링크

### 공식 문서
- [Arduino Reference](https://www.arduino.cc/reference/en/)
- [ROS Wiki](http://wiki.ros.org/)
- [rosserial Arduino](http://wiki.ros.org/rosserial_arduino)

### 도구
- [Arduino IDE](https://www.arduino.cc/en/software)
- [Fritzing](https://fritzing.org/) - 회로도
- [rqt](http://wiki.ros.org/rqt) - ROS 시각화

### 커뮤니티
- [Arduino Forum](https://forum.arduino.cc/)
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)

## 📐 공식 모음

### 초음파 센서
```cpp
distance_cm = duration_us * 0.034 / 2;
```

### 차동 구동
```cpp
left_speed = linear - angular * wheel_base / 2;
right_speed = linear + angular * wheel_base / 2;
```

### PID
```cpp
output = Kp * error + Ki * integral + Kd * derivative;
```

### RPM 계산
```cpp
rpm = (pulses / ppr) * (60000 / time_ms);
```

## 🎯 학습 체크리스트

### 초급
- [ ] LED 점멸
- [ ] 시리얼 통신
- [ ] 센서 읽기
- [ ] 모터 제어

### 중급
- [ ] rosserial 연결
- [ ] Publisher/Subscriber
- [ ] 다중 센서 통합
- [ ] ROS 모터 제어

### 고급
- [ ] PID 알고리즘
- [ ] 엔코더 사용
- [ ] 차동 구동
- [ ] 자율 주행

---

**이 가이드를 프린트하여 책상에 놓고 참고하세요!** 📖
