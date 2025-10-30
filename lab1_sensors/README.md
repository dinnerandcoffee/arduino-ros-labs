# Lab 1: 센서 읽기 (Sensor Reading)

## 목표
아두이노를 사용하여 다양한 센서로부터 데이터를 읽고 시리얼 모니터로 출력합니다.

## 실습 내용

### 1. 초음파 센서 (HC-SR04)
- **파일**: `ultrasonic_sensor.ino`
- **기능**: 초음파를 이용한 거리 측정
- **핀 연결**:
  - VCC → 5V
  - GND → GND
  - Trig → Pin 9
  - Echo → Pin 10

### 2. 적외선 센서 (IR Sensor)
- **파일**: `ir_sensor.ino`
- **기능**: 장애물 감지
- **핀 연결**:
  - VCC → 5V
  - GND → GND
  - OUT → Pin 7

## 학습 내용
1. 디지털/아날로그 입력 읽기
2. 센서 데이터 처리 및 변환
3. 시리얼 통신을 통한 데이터 출력

## 실행 방법
1. Arduino IDE에서 해당 `.ino` 파일 열기
2. 보드와 포트 선택 (Tools → Board, Port)
3. 업로드 (Upload)
4. 시리얼 모니터 열기 (Tools → Serial Monitor, 9600 baud)

## 다음 단계
Lab 2에서는 모터 제어를 학습합니다.
