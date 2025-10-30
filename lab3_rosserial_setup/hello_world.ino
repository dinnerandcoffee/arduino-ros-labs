/*
 * Lab 3: rosserial 기본 설정 (rosserial Setup)
 * 
 * Arduino와 ROS 간의 기본 통신을 설정합니다.
 * 
 * 필요 라이브러리:
 * - ros_lib (rosserial_arduino)
 * 
 * 설치 방법:
 * 1. ROS에서: sudo apt-get install ros-<distro>-rosserial-arduino
 * 2. 라이브러리 생성: rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries/
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup() {
  nh.initNode();
  nh.advertise(chatter);
}

void loop() {
  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1000);
}
