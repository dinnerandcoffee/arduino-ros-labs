# 회로 연결 가이드

## 전체 시스템 배선도

```
[배터리 7.4V-12V]
    |
    +---> [L298N 12V 입력]
    |         |
    |         +---> [5V 출력] ---> [Arduino 5V (옵션)]
    |         |
    |         +---> [모터 출력 A] ---> [왼쪽 DC 모터]
    |         +---> [모터 출력 B] ---> [오른쪽 DC 모터]
    |
    +---> [Arduino Vin (옵션)]

[Arduino]
    |
    +---> [5V, GND] ---> [HC-SR04 초음파 센서]
    +---> [5V, GND] ---> [IR 센서]
    +---> [디지털 핀] ---> [L298N 제어 신호]
    +---> [인터럽트 핀] ---> [로터리 인코더]
```

## 1. 전원 연결

### L298N 모터 드라이버
```
배터리 (+) ---> L298N [12V 입력]
배터리 (-) ---> L298N [GND]
                L298N [5V 출력] ---> Arduino [5V] (옵션)
                L298N [GND] ---> Arduino [GND] *** 필수 공통 접지 ***
```

⚠️ **중요**: Arduino GND와 L298N GND를 반드시 연결해야 합니다!

### Arduino 전원
**옵션 1**: USB로 전원 공급 (개발/테스트용)
```
PC USB ---> Arduino USB 포트
```

**옵션 2**: L298N 5V 출력 사용
```
L298N [5V] ---> Arduino [5V]
L298N [GND] ---> Arduino [GND]
```

**옵션 3**: 배터리 직접 연결 (7-12V)
```
배터리 (+) ---> Arduino [Vin]
배터리 (-) ---> Arduino [GND]
```

## 2. L298N 모터 드라이버 연결

### L298N 핀 설명
```
[전원 입력]
- 12V: 모터 전원 입력 (7-35V)
- GND: 전원 접지
- 5V: 5V 출력 (점퍼 캡 장착 시)

[모터 A (왼쪽)]
- ENA: 속도 제어 (PWM)
- IN1: 방향 제어 1
- IN2: 방향 제어 2
- OUT1, OUT2: 모터 A 출력

[모터 B (오른쪽)]
- ENB: 속도 제어 (PWM)
- IN3: 방향 제어 1
- IN4: 방향 제어 2
- OUT3, OUT4: 모터 B 출력
```

### Arduino 연결
```
Arduino Pin 3 (PWM)  ---> L298N [ENA]
Arduino Pin 5        ---> L298N [IN1]
Arduino Pin 6        ---> L298N [IN2]
Arduino Pin 9 (PWM)  ---> L298N [ENB]
Arduino Pin 10       ---> L298N [IN3]
Arduino Pin 11       ---> L298N [IN4]
Arduino GND          ---> L298N [GND]
```

### 모터 연결
```
L298N [OUT1] ---> 왼쪽 모터 (+)
L298N [OUT2] ---> 왼쪽 모터 (-)
L298N [OUT3] ---> 오른쪽 모터 (+)
L298N [OUT4] ---> 오른쪽 모터 (-)
```

**주의**: 모터가 반대로 회전하면 (+)와 (-) 전선을 바꾸세요.

## 3. HC-SR04 초음파 센서

```
HC-SR04 [VCC]  ---> Arduino [5V]
HC-SR04 [TRIG] ---> Arduino [Pin 9]
HC-SR04 [ECHO] ---> Arduino [Pin 10]
HC-SR04 [GND]  ---> Arduino [GND]
```

**센서 배치**:
- 로봇 전면 중앙에 설치
- 지면과 평행하게 설치
- 장애물과 수직으로 향하도록 설치

## 4. IR 적외선 센서 (옵션)

```
IR 센서 [VCC] ---> Arduino [5V]
IR 센서 [OUT] ---> Arduino [Pin 7]
IR 센서 [GND] ---> Arduino [GND]
```

## 5. 로터리 인코더 (Lab 6-7용)

### 왼쪽 인코더
```
인코더 [VCC]  ---> Arduino [5V]
인코더 [A상]  ---> Arduino [Pin 2] (INT0)
인코더 [B상]  ---> Arduino [Pin 4] (옵션)
인코더 [GND]  ---> Arduino [GND]
```

### 오른쪽 인코더
```
인코더 [VCC]  ---> Arduino [5V]
인코더 [A상]  ---> Arduino [Pin 18/A4]
인코더 [B상]  ---> Arduino [Pin 19/A5] (옵션)
인코더 [GND]  ---> Arduino [GND]
```

**주의**: 인코더는 인터럽트 가능 핀에 연결해야 합니다.

## 6. 전체 연결표

| 컴포넌트 | 핀 | Arduino 핀 |
|---------|-----|-----------|
| L298N ENA | PWM | 3 |
| L298N IN1 | Digital | 5 |
| L298N IN2 | Digital | 6 |
| L298N ENB | PWM | 9 |
| L298N IN3 | Digital | 10 |
| L298N IN4 | Digital | 11 |
| 초음파 TRIG | Digital | 9 |
| 초음파 ECHO | Digital | 10 |
| IR 센서 | Digital | 7 |
| 왼쪽 인코더 | Interrupt | 2 |
| 오른쪽 인코더 | Interrupt | 18 (A4) |

## 7. 배선 체크리스트

- [ ] 배터리 전압 확인 (7.4V~12V)
- [ ] L298N과 Arduino GND 공통 연결
- [ ] PWM 핀 연결 확인 (3, 9번)
- [ ] 모터 극성 확인 (테스트 후 필요시 변경)
- [ ] 센서 전원 연결 (5V, GND)
- [ ] USB 케이블 연결
- [ ] 점퍼선 접촉 불량 확인

## 8. 안전 수칙

⚠️ **배선 전 확인**:
1. 모든 전원을 차단하고 배선
2. 극성 확인 (특히 배터리)
3. 단락 방지 (전선 피복 확인)

⚠️ **테스트 전 확인**:
1. 배선 재확인
2. 모터를 바퀴에서 분리하고 테스트
3. 로봇을 들어올린 상태에서 테스트

⚠️ **운영 중 주의**:
1. 모터 과열 체크
2. 배터리 전압 모니터링
3. 비상 정지 준비 (USB 분리 또는 전원 차단)

## 9. 문제 해결

### 모터가 회전하지 않음
1. L298N 전원 확인
2. ENA, ENB 점퍼 캡 제거 확인 (PWM 제어용)
3. 배선 연결 확인

### 모터가 한쪽만 회전
1. 해당 모터 배선 확인
2. L298N 출력 전압 측정
3. Arduino 핀 출력 확인

### 센서 값이 이상함
1. 5V 전원 확인
2. GND 연결 확인
3. 센서 방향 및 위치 확인

### Arduino 리셋됨
1. 전원 공급 부족 → 별도 배터리 사용
2. 모터 노이즈 → 커패시터 추가
3. 접지 불량 → GND 재연결

## 10. 권장 부품 사양

- **Arduino**: Uno R3
- **모터 드라이버**: L298N (2A per channel)
- **DC 모터**: 6V, 200RPM, 기어비 1:48
- **배터리**: LiPo 2S (7.4V) 2200mAh 이상
- **초음파 센서**: HC-SR04
- **점퍼선**: 듀폰 선 (Male-Female, Male-Male)
- **로터리 인코더**: 600 P/R (옵션)

## 11. 배선 사진/다이어그램 참고

실제 배선 시 다음을 참고하세요:
- L298N 공식 데이터시트
- Arduino Uno 핀맵
- HC-SR04 데이터시트

**온라인 회로 시뮬레이터**:
- Tinkercad Circuits
- Fritzing
