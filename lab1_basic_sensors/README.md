# Lab 1: 기본 센서 제어

## 목표
Arduino를 사용하여 기본적인 센서들을 제어하는 방법을 학습합니다.

## 사용 센서
1. **HC-SR04 초음파 센서** - 거리 측정
2. **IR 적외선 센서** - 장애물 감지

## 학습 내용
- 디지털 입출력 제어
- 펄스 신호 처리
- 시리얼 통신을 통한 데이터 모니터링

## 실습 순서

### 1. 초음파 센서 (ultrasonic_sensor.ino)
```
연결:
HC-SR04 VCC  -> Arduino 5V
HC-SR04 GND  -> Arduino GND
HC-SR04 TRIG -> Arduino Pin 9
HC-SR04 ECHO -> Arduino Pin 10
```

**동작 원리:**
- 트리거 핀으로 10μs 펄스 전송
- 에코 핀에서 반사된 신호의 지속 시간 측정
- 거리 = (시간 × 음속) / 2

### 2. IR 센서 (ir_sensor.ino)
```
연결:
IR VCC -> Arduino 5V
IR GND -> Arduino GND
IR OUT -> Arduino Pin 7
```

**동작 원리:**
- 적외선을 발사하고 반사된 신호 감지
- 장애물 있을 때: LOW
- 장애물 없을 때: HIGH

## 실행 방법
1. Arduino IDE에서 해당 .ino 파일 열기
2. 올바른 보드와 포트 선택
3. 업로드 후 시리얼 모니터로 결과 확인

## 다음 단계
Lab 2에서는 모터 제어를 학습합니다.
