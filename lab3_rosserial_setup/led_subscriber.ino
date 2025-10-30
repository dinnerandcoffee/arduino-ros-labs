/*
 * Lab 3: LED 제어 예제 (LED Control via ROS)
 * 
 * ROS 토픽을 구독하여 LED를 제어합니다.
 */

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

const int LED_PIN = 13;

void messageCb(const std_msgs::Bool& msg) {
  if (msg.data) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

ros::Subscriber<std_msgs::Bool> sub("led_control", &messageCb);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
