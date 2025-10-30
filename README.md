# Arduino x ROS 실습 교재

이 교재는 ROS/AI 자율주행 로봇 과정에서 아두이노를 활용해
- 센서/모터 등 하부 하드웨어를 제어하고
- ROS(rosserial 또는 micro-ROS)로 상위 시스템과 통신하며
- 간단한 폐루프 제어(PID)까지 구현
하도록 설계되었습니다.

## 학습 흐름
1. 환경 세팅(IDE, 드라이버)와 Blink
2. 디지털/아날로그 I/O, 시리얼 디버깅
3. 거리/조도 등 대표 센서 읽기
4. 모터 드라이버(PWM/방향) 구동
5. ROS 브리지(rosserial 또는 micro-ROS)로 토픽 송수신
6. 엔코더 기반 속도 제어(PID) 및 통합 미니 프로젝트

자세한 커리큘럼은 [syllabus.md](./syllabus.md), 부품 목록은 [hardware.md](./hardware.md), 환경설정은 [setup.md](./setup.md)를 참고하세요.

## 요구 환경(권장)
- OS: Ubuntu 22.04 또는 Windows 11, macOS 최신
- ROS: ROS1 Noetic(rosserial) 또는 ROS2 Humble(micro-ROS)
- 보드: Arduino UNO(클래식) 또는 Nano/MEGA, 대안으로 ESP32(Arduino IDE 지원)
- 로봇: 2WD 섀시, L298N/TB6612FNG 모터 드라이버, HC-SR04 초음파, 라인센서(선택), 엔코더(가능하면)

학습 중 생길 수 있는 전원/배선 이슈와 안전사항은 각 레슨의 “주의” 섹션을 꼭 확인하세요.
