# ROS 설정 가이드

이 가이드는 Ubuntu에서 ROS를 설치하고 rosserial_arduino를 설정하는 방법을 단계별로 설명합니다.

## 시스템 요구사항

- Ubuntu 20.04 LTS (ROS Noetic) 또는
- Ubuntu 18.04 LTS (ROS Melodic)
- Arduino IDE 1.8.x 또는 2.x
- 최소 4GB RAM, 20GB 디스크 공간

## ROS Noetic 설치 (Ubuntu 20.04)

### 1. 소스 리스트 설정

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 2. 키 설정

```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### 3. 패키지 업데이트

```bash
sudo apt update
```

### 4. ROS Noetic 설치

**전체 설치 (권장)**:
```bash
sudo apt install ros-noetic-desktop-full
```

**기본 설치** (용량 절약):
```bash
sudo apt install ros-noetic-desktop
```

### 5. 환경 설정

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. 의존성 도구 설치

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 7. rosdep 초기화

```bash
sudo rosdep init
rosdep update
```

## rosserial 설치

### 1. rosserial 패키지 설치

```bash
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```

### 2. Arduino 라이브러리 생성

**방법 1: rosrun 사용 (권장)**
```bash
cd ~/Arduino/libraries
rm -rf ros_lib  # 기존 라이브러리 제거 (있다면)
rosrun rosserial_arduino make_libraries.py .
```

**방법 2: Arduino Library Manager**
1. Arduino IDE 실행
2. Sketch → Include Library → Manage Libraries
3. "Rosserial Arduino Library" 검색
4. 설치 클릭

### 3. 설치 확인

Arduino IDE에서:
1. File → Examples → ros_lib
2. HelloWorld 예제가 보이면 성공

## 워크스페이스 설정 (선택)

자신만의 ROS 패키지를 개발하려면 catkin 워크스페이스가 필요합니다.

### 1. 워크스페이스 생성

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

### 2. 환경 설정

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 패키지 생성 (예시)

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_robot_pkg rospy std_msgs sensor_msgs geometry_msgs
cd ~/catkin_ws
catkin_make
```

## Arduino 설정

### 1. Arduino IDE 설치

**방법 1: 웹사이트에서 다운로드**
```bash
# https://www.arduino.cc/en/software 에서 다운로드
cd ~/Downloads
tar -xvf arduino-*.tar.xz
sudo mv arduino-*/ /opt/arduino
cd /opt/arduino
sudo ./install.sh
```

**방법 2: apt 사용**
```bash
sudo apt install arduino
```

### 2. 사용자 권한 설정

```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
```

**재로그인 필요!** (또는 재부팅)

### 3. 포트 확인

```bash
# Arduino 연결 전
ls /dev/ttyUSB* /dev/ttyACM*

# Arduino 연결 후 (위 명령 다시 실행)
# 새로 나타난 포트가 Arduino 포트입니다
```

일반적으로:
- `/dev/ttyUSB0` - FTDI 칩 사용하는 Arduino
- `/dev/ttyACM0` - 네이티브 USB Arduino (Uno, Mega)

## 첫 번째 테스트

### 1. Arduino 업로드

Lab 3의 `hello_world.ino` 파일을:
1. Arduino IDE에서 열기
2. Tools → Board → Arduino Uno 선택
3. Tools → Port → /dev/ttyUSB0 (또는 해당 포트) 선택
4. Upload 버튼 클릭

### 2. ROS 실행

**Terminal 1: roscore**
```bash
roscore
```

**Terminal 2: rosserial node**
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

출력 예시:
```
[INFO] [1234567890.123456]: ROS Serial Python Node
[INFO] [1234567890.234567]: Connecting to /dev/ttyUSB0 at 57600 baud
[INFO] [1234567890.345678]: Note: publish buffer size is 512 bytes
[INFO] [1234567890.456789]: Setup publisher on chatter [std_msgs/String]
```

**Terminal 3: 토픽 확인**
```bash
rostopic echo /chatter
```

출력 예시:
```
data: "hello world!"
---
data: "hello world!"
---
```

성공! 🎉

## 유용한 ROS 도구

### 1. rqt 설치

```bash
sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins
```

사용법:
```bash
rqt_graph  # 노드 그래프
rqt_plot   # 데이터 플롯
rqt_console  # 로그 확인
```

### 2. teleop 키보드 제어

```bash
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

사용법:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 3. rviz (3D 시각화)

```bash
sudo apt-get install ros-noetic-rviz
```

사용법:
```bash
rosrun rviz rviz
```

## 문제 해결

### "rosrun: command not found"

```bash
source /opt/ros/noetic/setup.bash
# 또는 .bashrc에 추가했는지 확인
```

### 포트 권한 오류

```bash
sudo chmod 666 /dev/ttyUSB0
# 또는
sudo usermod -a -G dialout $USER
# 재로그인 필요
```

### rosserial 연결 실패

**원인 1: 잘못된 포트**
```bash
# 포트 찾기
ls /dev/ttyUSB* /dev/ttyACM*
# 올바른 포트 사용
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

**원인 2: baud rate 불일치**
```bash
# Arduino는 기본 57600 baud
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=57600
```

**원인 3: 시리얼 모니터가 열려있음**
- Arduino IDE의 시리얼 모니터 닫기
- 다른 프로그램이 포트를 사용 중인지 확인

### ros_lib 컴파일 오류

**해결 1: 라이브러리 재생성**
```bash
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

**해결 2: Arduino IDE 재시작**

**해결 3: 메모리 부족**
- 불필요한 헤더 제거
- 간단한 예제로 시작

### "Lost sync with device"

**원인**: Arduino가 예상보다 빨리 또는 늦게 응답

**해결**:
```cpp
// setup() 함수에 추가
delay(1000);  // ROS 연결 전 1초 대기
```

## ROS 명령어 치트시트

### roscore
```bash
roscore  # ROS 마스터 시작 (모든 ROS 작업의 기본)
```

### rosnode
```bash
rosnode list  # 실행 중인 노드 목록
rosnode info /node_name  # 노드 정보
rosnode kill /node_name  # 노드 종료
```

### rostopic
```bash
rostopic list  # 토픽 목록
rostopic echo /topic_name  # 토픽 내용 출력
rostopic hz /topic_name  # 토픽 주파수 확인
rostopic info /topic_name  # 토픽 정보
rostopic pub /topic_name msg_type "data"  # 토픽 발행
```

### rosservice
```bash
rosservice list  # 서비스 목록
rosservice call /service_name  # 서비스 호출
```

### rosmsg / rossrv
```bash
rosmsg show std_msgs/String  # 메시지 구조 확인
rossrv show std_srvs/SetBool  # 서비스 구조 확인
```

### catkin
```bash
catkin_make  # 워크스페이스 빌드
catkin_create_pkg  # 새 패키지 생성
```

## 추가 학습 자료

### 공식 튜토리얼
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [rosserial Arduino Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials)

### 유용한 링크
- [ROS Answers](https://answers.ros.org/) - Q&A
- [ROS Discourse](https://discourse.ros.org/) - 토론
- [ROS Wiki](http://wiki.ros.org/) - 문서

### 추천 과정
1. ROS 기본 개념 (노드, 토픽, 서비스)
2. Publisher/Subscriber 작성
3. 커스텀 메시지 생성
4. Launch 파일 사용
5. TF (Transform) 이해

## 다음 단계

ROS 설정이 완료되었습니다!
- [Lab 3: rosserial Setup](../lab3_rosserial_setup/)으로 이동
- [Getting Started Guide](GETTING_STARTED.md)로 돌아가기

---

질문이 있으시면 Issues 탭에서 문의해 주세요!
