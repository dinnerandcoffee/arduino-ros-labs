# Arduino-ROS Labs 설치 가이드

## 개요
이 가이드는 Arduino와 ROS를 사용한 자율주행 로봇 실습 환경을 설정하는 방법을 안내합니다.

## 시스템 요구사항

### 하드웨어
- **PC/노트북**: Ubuntu 18.04/20.04/22.04
- **Arduino**: Uno, Mega, 또는 호환 보드
- **USB 케이블**: Arduino 연결용
- **로봇 섀시**: 2륜 구동
- **모터**: DC 기어드 모터 2개
- **모터 드라이버**: L298N 또는 유사
- **센서**: 
  - HC-SR04 초음파 센서
  - IR 적외선 센서 (옵션)
  - 로터리 인코더 (옵션, Lab 6-7용)
- **전원**: 7.4V~12V 배터리 (LiPo 2S 또는 18650 배터리팩)

### 소프트웨어
- **Ubuntu**: 18.04 (Melodic) / 20.04 (Noetic) / 22.04 (Humble)
- **ROS**: ROS1 Noetic 또는 ROS2 Humble
- **Arduino IDE**: 1.8.x 또는 2.x

## ROS 설치

### ROS1 Noetic (Ubuntu 20.04)
```bash
# ROS 저장소 추가
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 키 추가
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# ROS 설치
sudo apt update
sudo apt install ros-noetic-desktop-full

# 환경 설정
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 의존성 도구
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# rosdep 초기화
sudo rosdep init
rosdep update
```

### ROS2 Humble (Ubuntu 22.04)
```bash
# UTF-8 설정
locale

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS 저장소 추가
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 설치
sudo apt update
sudo apt install ros-humble-desktop

# 환경 설정
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## rosserial 설치

### ROS1 Noetic
```bash
# rosserial 패키지 설치
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial

# Arduino 라이브러리 생성
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

### ROS2 Humble (micro-ROS 권장)
```bash
# micro-ROS 설치
sudo apt install ros-humble-micro-ros-setup

# 또는 rosserial for ROS2
sudo apt install ros-humble-rosserial
```

## Arduino IDE 설치

### 방법 1: 공식 웹사이트
1. https://www.arduino.cc/en/software 방문
2. Linux 64-bit 다운로드
3. 압축 해제 후 install.sh 실행

### 방법 2: 패키지 매니저 (권장)
```bash
sudo apt update
sudo apt install arduino
```

### Arduino IDE 2.x (최신)
```bash
# AppImage 다운로드
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.2.1_Linux_64bit.AppImage

# 실행 권한 부여
chmod +x arduino-ide_2.2.1_Linux_64bit.AppImage

# 실행
./arduino-ide_2.2.1_Linux_64bit.AppImage
```

## USB 포트 권한 설정

```bash
# 사용자를 dialout 그룹에 추가
sudo usermod -a -G dialout $USER

# 로그아웃 후 재로그인 또는 재부팅

# 임시로 권한 부여 (재부팅 시 초기화됨)
sudo chmod 666 /dev/ttyACM0
```

## ros_lib 라이브러리 설치 (Arduino)

### 자동 설치 (권장)
```bash
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```

### 수동 설치
1. Arduino IDE 실행
2. Sketch -> Include Library -> Manage Libraries
3. "Rosserial Arduino Library" 검색 및 설치

## 추가 패키지 설치

```bash
# Teleop 키보드 제어
sudo apt-get install ros-noetic-teleop-twist-keyboard

# rqt 도구
sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins

# rviz
sudo apt-get install ros-noetic-rviz
```

## 연결 테스트

### 1. Arduino 연결 확인
```bash
# USB 디바이스 확인
ls -l /dev/ttyACM*
# 또는
ls -l /dev/ttyUSB*

# Arduino 포트 확인
dmesg | grep tty
```

### 2. Arduino IDE 테스트
1. Arduino IDE 실행
2. File -> Examples -> 01.Basics -> Blink
3. Tools -> Board -> Arduino Uno (또는 사용 중인 보드)
4. Tools -> Port -> /dev/ttyACM0 (또는 확인된 포트)
5. Upload 클릭
6. LED 깜박임 확인

### 3. rosserial 테스트
```bash
# 터미널 1: roscore
roscore

# 터미널 2: Hello World 예제 업로드 후
rosrun rosserial_python serial_node.py /dev/ttyACM0

# 터미널 3: 메시지 확인
rostopic list
rostopic echo /chatter
```

## 문제 해결

### Arduino가 인식되지 않음
```bash
# USB 케이블 확인 (데이터 전송 가능한 케이블 사용)
# 다른 USB 포트 시도
# Arduino 보드 리셋 버튼 누르기
```

### 업로드 실패
```bash
# 포트 권한 확인
ls -l /dev/ttyACM0

# 다른 프로그램이 포트 사용 중인지 확인
lsof /dev/ttyACM0

# rosserial 실행 중이면 종료
```

### ros_lib 컴파일 오류
```bash
# ros_lib 재생성
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

# Arduino IDE 재시작
```

### rosserial 연결 오류
```bash
# Baud rate 확인 (기본: 57600)
# Arduino 리셋 후 재시도
# USB 케이블 교체
```

## 다음 단계

설치가 완료되면 Lab 1부터 순서대로 실습을 진행하세요:

1. **Lab 1**: 기본 센서 제어
2. **Lab 2**: 모터 제어
3. **Lab 3**: rosserial 설정
4. **Lab 4**: 센서 데이터 퍼블리시
5. **Lab 5**: 모터 Subscriber
6. **Lab 6**: PID 제어
7. **Lab 7**: 자율주행 통합

## 참고 자료

- ROS Wiki: http://wiki.ros.org
- rosserial: http://wiki.ros.org/rosserial_arduino
- Arduino: https://www.arduino.cc
- ROS Answers: https://answers.ros.org
