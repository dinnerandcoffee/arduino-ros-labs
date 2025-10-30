# 자주 묻는 질문 (FAQ)

Arduino-ROS 실습 중 자주 발생하는 질문과 답변 모음입니다.

## 🔧 Arduino 관련

### Q1: Arduino IDE에서 업로드가 안 됩니다.
**A:** 다음을 확인하세요:
1. **올바른 보드 선택**: Tools → Board → Arduino Uno
2. **올바른 포트 선택**: Tools → Port → /dev/ttyUSB0 (또는 /dev/ttyACM0)
3. **포트 권한 확인**:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # 또는 영구적으로
   sudo usermod -a -G dialout $USER  # 재로그인 필요
   ```
4. **USB 케이블 확인**: 데이터 전송 가능한 케이블인지 확인
5. **다른 프로그램이 포트 사용 중인지 확인**: 시리얼 모니터 닫기

### Q2: "avrdude: stk500_recv(): programmer is not responding" 오류가 발생합니다.
**A:** 
1. Arduino를 재연결하세요.
2. 다른 USB 포트를 시도하세요.
3. 리셋 버튼을 눌러보세요.
4. 부트로더가 손상되었을 수 있습니다 (드물게).

### Q3: Arduino 메모리가 부족하다는 메시지가 나옵니다.
**A:** Arduino Uno는 SRAM이 2KB로 제한적입니다:
```cpp
// 문자열을 PROGMEM에 저장
Serial.println(F("Text"));  // F() 매크로 사용

// 불필요한 전역 변수 제거
// Serial.print 줄이기
// 큰 배열은 PROGMEM 사용
```

### Q4: 센서 값이 계속 변하거나 이상합니다.
**A:** 
1. **연결 확인**: VCC, GND, 신호선 확인
2. **전원 안정성**: 전원이 충분한지 확인
3. **노이즈 필터링**: 
   ```cpp
   // 이동 평균 필터
   float average = (val1 + val2 + val3) / 3.0;
   ```
4. **Pull-up/Pull-down 저항**: 필요한 경우 추가

## 🤖 ROS 관련

### Q5: rosserial이 연결되지 않습니다.
**A:** 순서대로 확인:
1. **roscore 실행 여부**:
   ```bash
   roscore
   ```
2. **올바른 포트 사용**:
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*  # 포트 확인
   rosrun rosserial_python serial_node.py /dev/ttyUSB0
   ```
3. **Arduino 시리얼 모니터 닫기**: Arduino IDE의 시리얼 모니터가 열려있으면 안 됨
4. **baud rate 확인**:
   ```bash
   rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=57600
   ```

### Q6: "Lost sync with device" 오류가 계속 발생합니다.
**A:** 
```cpp
// Arduino setup() 함수에 추가
void setup() {
  delay(1000);  // ROS 연결 전 1초 대기
  nh.initNode();
  // ...
}
```

### Q7: ROS 토픽이 보이지 않습니다.
**A:** 
```bash
# rosserial 노드 확인
rosnode list  # /serial_node 있는지 확인

# 토픽 목록 확인
rostopic list

# rosserial 재시작
# Ctrl+C로 종료 후 재실행
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

### Q8: "No module named serial" 오류가 발생합니다.
**A:** pyserial 설치:
```bash
sudo apt-get install python3-serial
# 또는
pip3 install pyserial
```

## ⚡ 모터 제어 관련

### Q9: 모터가 전혀 작동하지 않습니다.
**A:** 체크리스트:
- [ ] L298N 전원 LED가 켜져 있는가?
- [ ] 12V 배터리가 연결되어 있는가?
- [ ] ENA/ENB 점퍼를 제거했는가? (PWM 사용시)
- [ ] Arduino GND와 L298N GND가 연결되어 있는가? (공통 접지)
- [ ] 모터가 L298N OUT1~OUT4에 연결되어 있는가?

### Q10: 모터가 한 방향으로만 회전합니다.
**A:** 
1. **코드 확인**: IN1, IN2 제어 로직 확인
2. **배선 확인**: 모터 선이 OUT1, OUT2에 올바르게 연결되었는지
3. **모터 드라이버 고장**: 다른 채널 테스트

### Q11: 모터 속도 제어가 안 됩니다.
**A:** 
1. **PWM 핀 사용**: ENA를 PWM 핀(9, 10, 3, 5, 6, 11)에 연결
2. **점퍼 제거**: L298N의 ENA/ENB 점퍼 제거
3. **analogWrite 사용**:
   ```cpp
   analogWrite(enA, 128);  // 0-255
   ```

### Q12: 차동 구동 로봇이 직진하지 않습니다.
**A:** 
1. **모터 속도 캘리브레이션**: 두 모터의 속도가 다를 수 있음
   ```cpp
   int leftSpeed = speed * 1.0;   // 보정 계수
   int rightSpeed = speed * 0.95; // 보정 계수
   ```
2. **바퀴 직경 확인**: 두 바퀴가 동일한지 확인
3. **기계적 문제**: 마찰, 정렬 확인

## 🎯 PID 제어 관련

### Q13: PID 제어가 불안정하고 진동합니다.
**A:** 
1. **Kp 감소**: 비례 게인이 너무 높을 수 있음
2. **Kd 증가**: 미분 항으로 진동 감쇠
3. **샘플링 시간 확인**: 일정한 간격으로 PID 계산
   ```cpp
   if (millis() - lastTime >= 100) {  // 100ms마다
     // PID 계산
   }
   ```

### Q14: PID 제어기가 목표값에 도달하지 못합니다.
**A:** 
1. **Ki 추가**: 적분 항으로 정상 상태 오차 제거
2. **Anti-windup 구현**:
   ```cpp
   integral = constrain(integral, -maxIntegral, maxIntegral);
   ```
3. **물리적 한계 확인**: 모터/액추에이터가 물리적으로 도달 가능한지

### Q15: 엔코더 카운트가 부정확합니다.
**A:** 
1. **인터럽트 핀 사용**: Pin 2, 3 (Arduino Uno)
2. **Pull-up 저항 활성화**:
   ```cpp
   pinMode(encoderA, INPUT_PULLUP);
   pinMode(encoderB, INPUT_PULLUP);
   ```
3. **노이즈 필터**: 커패시터 추가 (100nF)
4. **채터링 방지**: 소프트웨어 디바운싱

## 📡 통신 관련

### Q16: ROS 메시지가 누락됩니다.
**A:** 
1. **발행 주기 확인**: 너무 빠르게 발행하지 않기 (10Hz 권장)
2. **버퍼 크기 조정**:
   ```cpp
   ros::NodeHandle_<ArduinoHardware, 5, 5, 512, 512> nh;
   ```
3. **spinOnce() 호출**: 충분히 자주 호출
   ```cpp
   nh.spinOnce();
   delay(10);
   ```

### Q17: Serial.print와 rosserial을 같이 사용할 수 있나요?
**A:** **아니오!** rosserial은 시리얼 포트를 독점 사용합니다.
- 디버깅이 필요하면 ROS 토픽으로 발행하세요.
- 또는 Software Serial 사용 (별도 핀)

## 🔌 하드웨어 관련

### Q18: 센서와 모터를 동시에 사용하면 Arduino가 재부팅됩니다.
**A:** **전원 부족** 문제:
1. **모터는 별도 전원 사용**: 12V 배터리
2. **공통 접지 연결**: Arduino GND ↔ 모터 드라이버 GND
3. **USB 전원 + 외부 전원** 조합 사용

### Q19: 서보 모터가 떨립니다.
**A:** 
1. **전원 부족**: 별도 5V 전원 사용
2. **노이즈**: 커패시터 추가 (100-220μF)
3. **PWM 신호**: 안정적인 PWM 신호 확인

### Q20: 라인 센서가 라인을 감지하지 못합니다.
**A:** 
1. **센서 높이 조정**: 바닥에서 2-10mm
2. **임계값 조정**:
   ```cpp
   int threshold = 512;  // 0-1023, 조정 필요
   if (sensorValue > threshold) {
     // 라인 감지
   }
   ```
3. **조명 확인**: 일정한 조명 환경
4. **라인 색상**: 흰 바닥에 검은 라인 (또는 반대)

## 💻 소프트웨어 관련

### Q21: ros_lib 폴더가 없습니다.
**A:** 라이브러리 생성:
```bash
cd ~/Arduino/libraries
rm -rf ros_lib  # 기존 제거
rosrun rosserial_arduino make_libraries.py .
```
Arduino IDE 재시작 필요

### Q22: "#include <ros.h>" 오류가 발생합니다.
**A:** 
1. ros_lib 설치 확인 (Q21 참조)
2. Arduino IDE에서:
   - Sketch → Include Library → Add .ZIP Library
   - ros_lib 폴더 선택
3. Arduino IDE 재시작

### Q23: 여러 개의 .ino 파일을 어떻게 관리하나요?
**A:** Arduino IDE는 폴더 이름과 메인 .ino 파일이 같아야 합니다:
```
my_robot/
  my_robot.ino     # 메인 파일
  sensors.ino      # 자동으로 포함됨
  motors.ino       # 자동으로 포함됨
```

## 🎓 학습 관련

### Q24: 어느 Lab부터 시작해야 하나요?
**A:** 순차적으로 진행하세요:
1. **Lab 1**: Arduino 처음이라면 여기서 시작
2. **Lab 3**: ROS를 모르면 Lab 1, 2 후 진행
3. **Lab 6**: 모든 기초를 마친 후 도전

### Q25: 실습에 필요한 하드웨어를 어디서 구입하나요?
**A:** 
- **국내**: 디바이스마트, 아이씨뱅큐, 엘레파츠
- **해외**: AliExpress, Amazon, SparkFun, Adafruit
- **키트**: "Arduino Robot Car Kit" 검색

### Q26: 더 배우고 싶은데 어떤 프로젝트를 하면 좋을까요?
**A:** 난이도별 추천:
- **초급**: 장애물 회피 로봇
- **중급**: 라인 트레이싱 + ROS 통합
- **고급**: SLAM + 자율 주행

## 🆘 기타

### Q27: 도움을 어디서 받을 수 있나요?
**A:** 
1. **이 저장소**: GitHub Issues 탭
2. **Arduino Forum**: https://forum.arduino.cc/
3. **ROS Discourse**: https://discourse.ros.org/
4. **ROS Answers**: https://answers.ros.org/

### Q28: 상업적 용도로 사용해도 되나요?
**A:** 이 교재는 교육 목적으로 자유롭게 사용 가능합니다.

### Q29: 기여하고 싶은데 어떻게 하나요?
**A:** 
1. Fork this repository
2. 개선 사항 구현
3. Pull Request 제출
4. Issues에 제안 사항 등록

---

**질문이 여기에 없나요?** [Issues](https://github.com/dinnerandcoffee/arduino-ros-labs/issues)에서 질문해 주세요!
