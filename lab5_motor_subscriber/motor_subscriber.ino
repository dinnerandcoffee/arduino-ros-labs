/*
 * Lab 5: 모터 Subscriber (Motor Control Subscriber)
 * 
 * ROS 토픽을 구독하여 모터를 제어합니다.
 * 
 * 토픽: /cmd_vel
 * 메시지 타입: geometry_msgs/Twist
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// 모터 A (왼쪽)
const int ENA = 3;
const int IN1 = 5;
const int IN2 = 6;

// 모터 B (오른쪽)
const int ENB = 9;
const int IN3 = 10;
const int IN4 = 11;

// 모터 속도 (PWM 값)
int left_speed = 0;
int right_speed = 0;

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  // linear.x: 전진/후진 속도 (-1.0 ~ 1.0)
  // angular.z: 회전 속도 (-1.0 ~ 1.0)
  
  float linear = msg.linear.x;
  float angular = msg.angular.z;
  
  // 차동 구동 계산
  float left = linear - angular;
  float right = linear + angular;
  
  // 정규화 (-1.0 ~ 1.0)
  if (abs(left) > 1.0) left = (left > 0) ? 1.0 : -1.0;
  if (abs(right) > 1.0) right = (right > 0) ? 1.0 : -1.0;
  
  // PWM 값으로 변환 (0 ~ 255)
  left_speed = abs(left * 255);
  right_speed = abs(right * 255);
  
  // 방향 설정
  if (left >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  
  if (right >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  
  // 속도 설정
  analogWrite(ENA, left_speed);
  analogWrite(ENB, right_speed);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVelCallback);

void setup() {
  // 모터 핀 설정
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // 초기화
  stopMotors();
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
