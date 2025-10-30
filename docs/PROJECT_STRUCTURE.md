# 프로젝트 구조

```
arduino-ros-labs/
├── README.md                    # 프로젝트 개요 및 빠른 시작
├── .gitignore                   # Git 무시 파일
│
├── docs/                        # 문서 디렉토리
│   ├── GETTING_STARTED.md      # 시작 가이드
│   ├── HARDWARE_SETUP.md       # 하드웨어 연결 가이드
│   ├── ROS_SETUP.md            # ROS 설정 가이드
│   └── PROJECT_STRUCTURE.md    # 이 파일
│
├── lab1_sensors/                # Lab 1: 센서 읽기
│   ├── README.md               # Lab 1 설명서
│   ├── ultrasonic_sensor.ino   # 초음파 센서 예제
│   └── ir_sensor.ino           # IR 센서 예제
│
├── lab2_motors/                 # Lab 2: 모터 제어
│   ├── README.md               # Lab 2 설명서
│   ├── dc_motor_control.ino    # DC 모터 제어
│   └── servo_control.ino       # 서보 모터 제어
│
├── lab3_rosserial_setup/        # Lab 3: rosserial 설정
│   ├── README.md               # Lab 3 설명서
│   ├── hello_world.ino         # Publisher 예제
│   └── led_control.ino         # Subscriber 예제
│
├── lab4_ros_sensors/            # Lab 4: ROS 센서 통합
│   ├── README.md               # Lab 4 설명서
│   ├── ultrasonic_ros.ino      # 초음파 센서 ROS 통합
│   └── multi_sensors_ros.ino   # 다중 센서 ROS 통합
│
├── lab5_ros_motors/             # Lab 5: ROS 모터 제어
│   ├── README.md               # Lab 5 설명서
│   ├── motor_control_ros.ino   # DC 모터 ROS 제어
│   ├── servo_control_ros.ino   # 서보 모터 ROS 제어
│   └── differential_drive_ros.ino  # 차동 구동 로봇
│
└── lab6_pid_control/            # Lab 6: PID 제어
    ├── README.md               # Lab 6 설명서
    ├── basic_pid.ino           # 기본 PID 알고리즘
    ├── motor_speed_pid.ino     # 모터 속도 PID 제어
    ├── line_following_pid.ino  # 라인 트레이싱 PID
    └── motor_speed_pid_ros.ino # ROS + PID 통합
```

## 파일 설명

### 루트 디렉토리
- **README.md**: 프로젝트의 전체 개요, 빠른 시작 가이드, 학습 경로
- **.gitignore**: Git에서 추적하지 않을 파일 목록

### docs/ - 문서
종합적인 가이드와 참고 자료를 포함합니다.

- **GETTING_STARTED.md**: 전체 교재 개요, 필요한 하드웨어/소프트웨어, 학습 경로
- **HARDWARE_SETUP.md**: 각 Lab별 상세한 회로 연결 방법
- **ROS_SETUP.md**: Ubuntu에서 ROS 설치 및 rosserial 설정 가이드
- **PROJECT_STRUCTURE.md**: 프로젝트 구조 설명 (이 파일)

### lab1_sensors/ - 센서 읽기
Arduino의 기본 입력 처리를 학습합니다.

- **ultrasonic_sensor.ino**: HC-SR04 초음파 센서로 거리 측정
- **ir_sensor.ino**: IR 센서로 장애물 감지
- 학습 내용: 디지털/아날로그 입력, 센서 데이터 처리

### lab2_motors/ - 모터 제어
Arduino의 기본 출력 제어를 학습합니다.

- **dc_motor_control.ino**: L298N 드라이버로 DC 모터 제어 (속도, 방향)
- **servo_control.ino**: 서보 모터 각도 제어
- 학습 내용: PWM, H-브리지, 서보 라이브러리

### lab3_rosserial_setup/ - ROS 통신 기초
Arduino와 ROS 간의 기본 통신을 학습합니다.

- **hello_world.ino**: ROS로 메시지 발행 (Publisher)
- **led_control.ino**: ROS 메시지 구독 (Subscriber)
- 학습 내용: rosserial, NodeHandle, Publisher/Subscriber 패턴

### lab4_ros_sensors/ - ROS 센서 통합
센서 데이터를 ROS 시스템과 통합합니다.

- **ultrasonic_ros.ino**: 초음파 센서 데이터를 ROS 토픽으로 발행
- **multi_sensors_ros.ino**: 여러 센서 데이터를 동시에 발행
- 학습 내용: 센서 → ROS 통합, 다중 Publisher, rqt 도구

### lab5_ros_motors/ - ROS 모터 제어
ROS 명령을 받아 모터를 제어합니다.

- **motor_control_ros.ino**: ROS 토픽으로 DC 모터 제어
- **servo_control_ros.ino**: ROS 토픽으로 서보 모터 제어
- **differential_drive_ros.ino**: geometry_msgs/Twist로 차동 구동 로봇 제어
- 학습 내용: ROS → 모터 통합, 차동 구동 운동학

### lab6_pid_control/ - PID 폐루프 제어
PID 알고리즘으로 정밀한 제어를 구현합니다.

- **basic_pid.ino**: PID 알고리즘의 기본 구현
- **motor_speed_pid.ino**: 엔코더 피드백으로 모터 속도 제어
- **line_following_pid.ino**: PID로 라인 트레이싱 구현
- **motor_speed_pid_ros.ino**: ROS와 PID를 통합한 속도 제어
- 학습 내용: PID 이론, 엔코더 사용, 폐루프 제어

## 학습 순서

### 단계별 진행
1. **Lab 1 → Lab 2**: Arduino 기초 (센서 입력, 모터 출력)
2. **Lab 3**: ROS 통신 이해
3. **Lab 4 → Lab 5**: ROS 통합 (센서 → ROS → 모터)
4. **Lab 6**: 고급 제어 (PID)

### 선행 학습 필요
- **Lab 4**: Lab 1, Lab 3 완료 필요
- **Lab 5**: Lab 2, Lab 3 완료 필요
- **Lab 6**: Lab 2, Lab 5 완료 권장

## 코드 스타일

### Arduino 스케치 구조
```cpp
/*
 * 파일 설명
 * 기능 설명
 * 
 * Hardware:
 *   - 필요한 하드웨어 목록
 * 
 * Connections:
 *   - 핀 연결 정보
 */

// 전역 변수 및 상수
const int pin = 9;

void setup() {
  // 초기화 코드
}

void loop() {
  // 메인 로직
}

// 헬퍼 함수들
```

### README 구조
각 Lab의 README는 다음을 포함합니다:
- 목표
- 실습 내용
- 하드웨어 연결
- 실행 방법
- 학습 내용
- 다음 단계

## 확장 아이디어

### 추가 예정 Lab
- Lab 7: Encoder와 Odometry
- Lab 8: IMU 센서 통합
- Lab 9: Autonomous Navigation
- Lab 10: 통합 프로젝트

### 기타 자료
- Fritzing 회로도 (.fzz 파일)
- 동영상 튜토리얼 링크
- FAQ 문서
- 문제 해결 가이드

## 기여 가이드

새로운 Lab이나 예제를 추가하려면:
1. 해당 Lab 디렉토리 생성
2. .ino 파일 작성 (주석 포함)
3. README.md 작성
4. 메인 README.md 업데이트
5. Pull Request 제출

## 참고사항

- 모든 코드는 Arduino Uno 기준으로 작성됨
- ROS Noetic (Ubuntu 20.04) 기준
- 한국어 주석과 영어 변수명 사용
- 교육용으로 최적화된 간결한 코드

---

질문이나 제안이 있으시면 Issues를 열어주세요!
