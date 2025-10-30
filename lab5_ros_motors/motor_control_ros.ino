/*
 * Lab 5: ROS Motor Control
 * ROS 토픽을 통한 DC 모터 제어
 * 
 * Hardware:
 * - L298N Motor Driver
 * - DC Motor
 * - Arduino Uno
 * 
 * Connections:
 * - ENA -> Pin 9 (PWM)
 * - IN1 -> Pin 8
 * - IN2 -> Pin 7
 * 
 * ROS 실행:
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyUSB0
 * $ rostopic pub /motor_cmd std_msgs/Int16 "data: 255"   # Full forward
 * $ rostopic pub /motor_cmd std_msgs/Int16 "data: 128"   # Half forward
 * $ rostopic pub /motor_cmd std_msgs/Int16 "data: 0"     # Stop
 * $ rostopic pub /motor_cmd std_msgs/Int16 "data: -255"  # Full backward
 */

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

// Motor A connections
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

void motorCallback(const std_msgs::Int16& cmd_msg) {
  int speed = cmd_msg.data;
  
  if (speed > 0) {
    // Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, constrain(speed, 0, 255));
  } else if (speed < 0) {
    // Backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, constrain(-speed, 0, 255));
  } else {
    // Stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }
}

ros::Subscriber<std_msgs::Int16> motor_sub("motor_cmd", &motorCallback);

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Initial state - stopped
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  nh.initNode();
  nh.subscribe(motor_sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
