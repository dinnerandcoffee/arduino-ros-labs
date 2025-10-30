/*
 * Lab 3: rosserial Hello World
 * rosserial을 사용한 ROS 통신 기본
 * 
 * 이 예제는 Arduino에서 ROS로 메시지를 발행합니다.
 * 
 * 필요 라이브러리:
 * - ros_lib (rosserial_arduino)
 * 
 * ROS 측 실행:
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyUSB0
 * $ rostopic echo /chatter
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
