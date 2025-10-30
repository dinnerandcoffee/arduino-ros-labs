/*
 * Lab 5: ROS Servo Control
 * ROS 토픽을 통한 서보 모터 제어
 * 
 * Hardware:
 * - SG90 Servo Motor
 * - Arduino Uno
 * 
 * Connections:
 * - Signal -> Pin 6
 * - VCC -> 5V
 * - GND -> GND
 * 
 * ROS 실행:
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyUSB0
 * $ rostopic pub /servo_cmd std_msgs/Int16 "data: 90"   # Center
 * $ rostopic pub /servo_cmd std_msgs/Int16 "data: 0"    # Min
 * $ rostopic pub /servo_cmd std_msgs/Int16 "data: 180"  # Max
 */

#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

ros::NodeHandle nh;

Servo myServo;
const int servoPin = 6;

void servoCallback(const std_msgs::Int16& cmd_msg) {
  int angle = constrain(cmd_msg.data, 0, 180);
  myServo.write(angle);
}

ros::Subscriber<std_msgs::Int16> servo_sub("servo_cmd", &servoCallback);

void setup() {
  myServo.attach(servoPin);
  myServo.write(90);  // Center position
  
  nh.initNode();
  nh.subscribe(servo_sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
