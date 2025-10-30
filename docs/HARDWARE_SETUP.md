# 하드웨어 연결 가이드

이 문서는 각 실습에 필요한 하드웨어 연결 방법을 상세히 설명합니다.

## 기본 준비사항

### 필수 도구
- 브레드보드
- 점퍼 와이어 (M-M, M-F)
- USB 케이블 (Arduino 연결용)
- 멀티미터 (선택, 디버깅용)

### 안전 주의사항
⚠️ **중요**: 다음 사항을 반드시 준수하세요!
- 회로 연결 전 모든 전원 차단
- 극성 확인 (VCC, GND)
- Arduino 5V 핀으로 모터 직접 구동 금지
- 공통 접지(GND) 반드시 연결

## Lab 1: 센서 연결

### HC-SR04 초음파 센서

```
HC-SR04        Arduino Uno
---------      -----------
VCC     --->   5V
GND     --->   GND
Trig    --->   Pin 9 (Digital)
Echo    --->   Pin 10 (Digital)
```

**주의사항**:
- Echo 핀은 5V 신호를 출력하므로 3.3V 보드에서는 전압 분배기 필요
- 센서 앞에 20cm 이상 공간 확보

### IR 장애물 감지 센서

```
IR Sensor      Arduino Uno
---------      -----------
VCC     --->   5V
GND     --->   GND
OUT     --->   Pin 7 (Digital)
```

**조정**:
- 가변 저항으로 감지 거리 조정 가능
- LED가 켜지면 장애물 감지됨

## Lab 2: 모터 연결

### DC 모터 + L298N 모터 드라이버

```
L298N          Arduino Uno
---------      -----------
ENA     --->   Pin 9 (PWM)
IN1     --->   Pin 8
IN2     --->   Pin 7
ENB     --->   Pin 3 (PWM) [2번째 모터용]
IN3     --->   Pin 5 [2번째 모터용]
IN4     --->   Pin 4 [2번째 모터용]
GND     --->   GND (공통 접지)

L298N          전원
---------      ----
+12V    --->   12V 배터리 양극
GND     --->   12V 배터리 음극

L298N          DC Motor
---------      --------
OUT1    --->   Motor A+
OUT2    --->   Motor A-
OUT3    --->   Motor B+ [2번째 모터]
OUT4    --->   Motor B- [2번째 모터]
```

**L298N 점퍼 설정**:
- ENA, ENB 점퍼: PWM 제어시 제거, 항상 켜기는 연결
- 5V 출력: 12V 입력시 5V 레귤레이터 사용 가능

**배선 색상 (일반적)**:
- 빨강: +12V
- 검정: GND
- 모터: 색상 무관, 방향이 반대면 선 바꾸기

### 서보 모터 (SG90)

```
SG90           Arduino Uno
---------      -----------
갈색/검정 --->  GND
빨강     --->   5V
주황/노랑 --->  Pin 6 (PWM)
```

**주의사항**:
- 다수의 서보 사용시 별도 5V 전원 권장
- 전류 소모: 100-300mA (부하에 따라)

## Lab 3-5: ROS 통합

기본적으로 Lab 1, 2와 동일한 연결을 유지하되, Arduino는 USB로 PC에 연결되어야 합니다.

```
Arduino Uno    PC
-----------    --
USB     --->   USB 포트 (/dev/ttyUSB0 또는 /dev/ttyACM0)
```

## Lab 6: PID 제어

### 엔코더가 있는 DC 모터

```
Motor Encoder  Arduino Uno
-------------  -----------
VCC     --->   5V
GND     --->   GND
Ch A    --->   Pin 2 (Interrupt 0)
Ch B    --->   Pin 3 (Interrupt 1)

모터 전원 연결은 Lab 2와 동일
```

**엔코더 타입**:
- **광학 엔코더**: 일반적으로 A, B, VCC, GND 4개 핀
- **홀 센서**: 자석과 센서 사용
- 사양 확인: PPR (Pulses Per Revolution)

### 5채널 라인 센서 어레이

```
Line Sensor    Arduino Uno
-----------    -----------
VCC     --->   5V
GND     --->   GND
S1      --->   A0
S2      --->   A1
S3      --->   A2
S4      --->   A3
S5      --->   A4
```

**센서 배치**:
```
[S1] [S2] [S3] [S4] [S5]
 좌              중앙              우
```

**라인 추종**:
- 검은색 라인: 센서 값 낮음 (0)
- 흰색 바닥: 센서 값 높음 (1)
- 또는 그 반대 (센서에 따라)

## 차동 구동 로봇 완전 조립

### 전체 연결도

```
=== 왼쪽 모터 ===
L298N (Ch A)   Arduino
ENA     --->   Pin 9 (PWM)
IN1     --->   Pin 8
IN2     --->   Pin 7

=== 오른쪽 모터 ===
L298N (Ch B)   Arduino
ENB     --->   Pin 3 (PWM)
IN3     --->   Pin 5
IN4     --->   Pin 4

=== 센서 (초음파) ===
HC-SR04        Arduino
Trig    --->   Pin 9 (기존과 충돌 방지: Pin 11로 변경)
Echo    --->   Pin 10

=== 전원 ===
12V Battery    L298N
+       --->   +12V
-       --->   GND

L298N 5V out   Arduino Vin (선택)
Arduino        USB 또는 9V 배터리

=== 공통 접지 ===
Arduino GND    L298N GND (필수!)
```

## 핀 배치 요약 (권장)

| 기능 | 핀 번호 | 타입 | 비고 |
|------|---------|------|------|
| Left Motor PWM | 9 | PWM | ENA |
| Left Motor Dir1 | 8 | Digital | IN1 |
| Left Motor Dir2 | 7 | Digital | IN2 |
| Right Motor PWM | 3 | PWM | ENB |
| Right Motor Dir1 | 5 | Digital | IN3 |
| Right Motor Dir2 | 4 | Digital | IN4 |
| Servo | 6 | PWM | - |
| IR Sensor | 7 | Digital | - |
| Ultrasonic Trig | 11 | Digital | - |
| Ultrasonic Echo | 10 | Digital | - |
| Encoder A | 2 | Interrupt | INT0 |
| Encoder B | 3 | Interrupt | INT1 |
| Line Sensor 1-5 | A0-A4 | Analog | - |

## 문제 해결

### 모터가 작동하지 않음
1. L298N 전원 LED 확인
2. ENA/ENB 점퍼 제거했는지 확인 (PWM 사용시)
3. 모터 전원(12V) 연결 확인
4. 공통 접지(GND) 연결 확인

### 센서 값이 이상함
1. VCC, GND 극성 확인
2. 케이블 연결 상태 확인
3. 멀티미터로 전압 측정 (VCC: 5V)

### Arduino가 재부팅됨
1. 전원 부족: USB 전원 + 별도 모터 전원 사용
2. 공통 접지 미연결
3. 모터 역기전력: 다이오드 추가

### 엔코더 카운트 오류
1. Pull-up 저항 확인 (내장 또는 외장)
2. 노이즈 필터링: 커패시터 추가
3. 인터럽트 핀 사용 확인 (Pin 2, 3)

## 회로도 (선택)

각 Lab 폴더에 Fritzing 회로도 파일(.fzz)이 포함되어 있습니다.
Fritzing 소프트웨어로 열어서 확인할 수 있습니다.

**Fritzing 다운로드**: https://fritzing.org/

## 추가 팁

### 케이블 정리
- 색상별 정리: 빨강(5V), 검정(GND), 기타(신호)
- 케이블 타이로 묶기
- 브레드보드를 로봇에 고정

### 전원 관리
- Arduino와 모터는 별도 전원 사용 권장
- 공통 접지는 필수
- 배터리 전압 정기 확인

### 디버깅
- LED로 신호 확인 (예: Pin 13 내장 LED)
- Serial.print로 값 확인
- 한 번에 하나씩 연결하여 테스트

---

연결에 문제가 있으시면 Issues 탭에서 사진과 함께 문의해 주세요!
