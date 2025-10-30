/*
 * Lab 7: 자율주행 통합 예제 (Autonomous Navigation Integration)
 * 
 * 센서 + 모터 제어 + PID를 통합하여 간단한 자율주행을 구현합니다.
 * 
 * 기능:
 * - 초음파 센서로 장애물 감지
 * - 장애물 회피 알고리즘
 * - PID 기반 모터 제어
 * - ROS 통신
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

// 핀 정의
const int ENA = 3, IN1 = 5, IN2 = 6;
const int ENB = 9, IN3 = 10, IN4 = 11;
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
const int ENCODER_LEFT_A = 2;
const int ENCODER_RIGHT_A = 18;

// 센서 데이터
float obstacle_distance = 4.0;
const float SAFE_DISTANCE = 0.3;  // 30cm
const float WARNING_DISTANCE = 0.5;  // 50cm

// 모터 제어
volatile long left_enc = 0, right_enc = 0;
long last_left = 0, last_right = 0;

// PID 변수
float Kp = 2.0, Ki = 0.5, Kd = 0.1;
float left_target = 0, left_current = 0, left_error = 0, left_last_error = 0, left_integral = 0;
float right_target = 0, right_current = 0, right_error = 0, right_last_error = 0, right_integral = 0;

// 타이머
unsigned long last_control_time = 0;
unsigned long last_sensor_time = 0;
unsigned long last_pub_time = 0;

// 자율주행 모드
enum DriveMode { MANUAL, AUTO_FORWARD, AUTO_AVOID_LEFT, AUTO_AVOID_RIGHT, AUTO_STOP };
DriveMode current_mode = AUTO_FORWARD;

// ROS 퍼블리셔
sensor_msgs::Range range_msg;
std_msgs::String status_msg;
ros::Publisher pub_range("/sensors/ultrasonic", &range_msg);
ros::Publisher pub_status("/robot/status", &status_msg);

// ROS 서브스크라이버
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  // 수동 제어 모드
  current_mode = MANUAL;
  left_target = (msg.linear.x - msg.angular.z * 0.5) * 100;
  right_target = (msg.linear.x + msg.angular.z * 0.5) * 100;
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd("/cmd_vel", &cmdVelCallback);

void setup() {
  // 모터 핀 설정
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  // 센서 핀 설정
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // 인코더 설정
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), []() { left_enc++; }, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), []() { right_enc++; }, RISING);
  
  // ROS 초기화
  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.advertise(pub_range);
  nh.advertise(pub_status);
  
  // Range 메시지 설정
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id = "ultrasonic_link";
  range_msg.field_of_view = 0.26;
  range_msg.min_range = 0.02;
  range_msg.max_range = 4.0;
  
  last_control_time = millis();
  last_sensor_time = millis();
  last_pub_time = millis();
}

void loop() {
  unsigned long now = millis();
  
  // 센서 읽기 (10Hz)
  if (now - last_sensor_time > 100) {
    obstacle_distance = readUltrasonic();
    
    // Range 메시지 퍼블리시
    range_msg.range = obstacle_distance;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    
    last_sensor_time = now;
  }
  
  // 자율주행 모드 결정
  if (current_mode != MANUAL) {
    autonomousControl();
  }
  
  // PID 제어 (20Hz)
  if (now - last_control_time > 50) {
    float dt = (now - last_control_time) / 1000.0;
    
    // 속도 계산
    left_current = (left_enc - last_left) / dt;
    right_current = (right_enc - last_right) / dt;
    
    // PID 계산 및 모터 제어
    pidControl(dt);
    
    last_left = left_enc;
    last_right = right_enc;
    last_control_time = now;
  }
  
  // 상태 퍼블리시 (5Hz)
  if (now - last_pub_time > 200) {
    publishStatus();
    last_pub_time = now;
  }
  
  nh.spinOnce();
}

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 4.0;  // 타임아웃
  
  return duration * 0.034 / 2 / 100.0;  // 미터 단위
}

void autonomousControl() {
  if (obstacle_distance < SAFE_DISTANCE) {
    // 매우 가까움 - 정지 또는 후진
    current_mode = AUTO_STOP;
    left_target = 0;
    right_target = 0;
  } else if (obstacle_distance < WARNING_DISTANCE) {
    // 경고 거리 - 회피 (좌회전)
    current_mode = AUTO_AVOID_LEFT;
    left_target = -30;
    right_target = 30;
  } else {
    // 안전 - 전진
    current_mode = AUTO_FORWARD;
    left_target = 50;
    right_target = 50;
  }
}

void pidControl(float dt) {
  // 왼쪽 모터 PID
  left_error = left_target - left_current;
  left_integral += left_error * dt;
  left_integral = constrain(left_integral, -100, 100);
  float left_derivative = (left_error - left_last_error) / dt;
  int left_pwm = Kp * left_error + Ki * left_integral + Kd * left_derivative;
  
  // 오른쪽 모터 PID
  right_error = right_target - right_current;
  right_integral += right_error * dt;
  right_integral = constrain(right_integral, -100, 100);
  float right_derivative = (right_error - right_last_error) / dt;
  int right_pwm = Kp * right_error + Ki * right_integral + Kd * right_derivative;
  
  // 모터 적용
  setMotor(ENA, IN1, IN2, constrain(left_pwm, -255, 255));
  setMotor(ENB, IN3, IN4, constrain(right_pwm, -255, 255));
  
  left_last_error = left_error;
  right_last_error = right_error;
}

void setMotor(int en, int in1, int in2, int pwm) {
  if (pwm >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -pwm);
  }
}

void publishStatus() {
  char buffer[50];
  switch (current_mode) {
    case MANUAL:
      sprintf(buffer, "MANUAL");
      break;
    case AUTO_FORWARD:
      sprintf(buffer, "AUTO_FORWARD (%.2fm)", obstacle_distance);
      break;
    case AUTO_AVOID_LEFT:
      sprintf(buffer, "AUTO_AVOID (%.2fm)", obstacle_distance);
      break;
    case AUTO_STOP:
      sprintf(buffer, "AUTO_STOP (%.2fm)", obstacle_distance);
      break;
  }
  status_msg.data = buffer;
  pub_status.publish(&status_msg);
}
