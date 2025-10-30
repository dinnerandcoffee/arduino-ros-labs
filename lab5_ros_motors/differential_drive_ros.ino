/*
 * Lab 5: Differential Drive Robot
 * ROS 토픽을 통한 차동 구동 로봇 제어
 * 
 * Hardware:
 * - L298N Motor Driver (2개 또는 듀얼 채널)
 * - DC Motors (Left, Right)
 * - Arduino Uno
 * 
 * Connections:
 * Left Motor:
 *   - ENA -> Pin 9 (PWM)
 *   - IN1 -> Pin 8
 *   - IN2 -> Pin 7
 * Right Motor:
 *   - ENB -> Pin 3 (PWM)
 *   - IN3 -> Pin 5
 *   - IN4 -> Pin 4
 * 
 * ROS 실행:
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyUSB0
 * 
 * 명령 예시:
 * $ rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'   # Forward
 * $ rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 1.0}}'   # Rotate left
 * $ rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: -0.5}, angular: {z: 0.0}}'  # Backward
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// Motor pins
// Left motor
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// Right motor
const int enB = 3;
const int in3 = 5;
const int in4 = 4;

// Robot parameters
const float wheelBase = 0.15;  // meters
const float maxSpeed = 255;     // PWM value

void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
  float linear = cmd_msg.linear.x;   // Forward/backward (-1.0 to 1.0)
  float angular = cmd_msg.angular.z; // Rotation (-1.0 to 1.0)
  
  // Calculate wheel speeds (differential drive)
  float leftSpeed = linear - angular * wheelBase / 2.0;
  float rightSpeed = linear + angular * wheelBase / 2.0;
  
  // Convert to PWM values
  int leftPWM = constrain(leftSpeed * maxSpeed, -maxSpeed, maxSpeed);
  int rightPWM = constrain(rightSpeed * maxSpeed, -maxSpeed, maxSpeed);
  
  // Control left motor
  if (leftPWM > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, abs(leftPWM));
  } else if (leftPWM < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, abs(leftPWM));
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }
  
  // Control right motor
  if (rightPWM > 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, abs(rightPWM));
  } else if (rightPWM < 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, abs(rightPWM));
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);
  }
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmdVelCallback);

void setup() {
  // Left motor setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Right motor setup
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Initial state - stopped
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
