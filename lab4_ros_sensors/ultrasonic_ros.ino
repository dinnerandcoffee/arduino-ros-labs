/*
 * Lab 4: Ultrasonic Sensor to ROS
 * 초음파 센서 데이터를 ROS로 전송
 * 
 * Hardware:
 * - HC-SR04 Ultrasonic Sensor
 * - Arduino Uno
 * 
 * Connections:
 * - Trig -> Pin 9
 * - Echo -> Pin 10
 * 
 * ROS 실행:
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyUSB0
 * $ rostopic echo /ultrasonic
 */

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 distance_msg;
ros::Publisher ultrasonic_pub("ultrasonic", &distance_msg);

const int trigPin = 9;
const int echoPin = 10;

long duration;
float distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  nh.initNode();
  nh.advertise(ultrasonic_pub);
}

void loop() {
  // Trigger ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read echo
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  
  // Publish to ROS
  distance_msg.data = distance;
  ultrasonic_pub.publish(&distance_msg);
  
  nh.spinOnce();
  delay(100);
}
