/*
 * Lab 3: rosserial LED Control
 * ROS 토픽 구독을 통한 LED 제어
 * 
 * 이 예제는 ROS 토픽을 구독하여 LED를 제어합니다.
 * 
 * Hardware:
 * - LED -> Pin 13 (내장 LED)
 * 
 * ROS 측 실행:
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyUSB0
 * $ rostopic pub /led std_msgs/Bool "data: true"  # LED ON
 * $ rostopic pub /led std_msgs/Bool "data: false" # LED OFF
 */

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

const int ledPin = 13;

void ledCallback(const std_msgs::Bool& msg) {
  if (msg.data) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

ros::Subscriber<std_msgs::Bool> sub("led", &ledCallback);

void setup() {
  pinMode(ledPin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
