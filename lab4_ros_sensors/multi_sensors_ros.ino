/*
 * Lab 4: Multiple Sensors to ROS
 * 여러 센서 데이터를 ROS로 전송
 * 
 * Hardware:
 * - HC-SR04 Ultrasonic Sensor
 * - IR Sensor
 * - Arduino Uno
 * 
 * Connections:
 * - Ultrasonic Trig -> Pin 9
 * - Ultrasonic Echo -> Pin 10
 * - IR Sensor -> Pin 7
 * 
 * ROS 실행:
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyUSB0
 * $ rostopic echo /ultrasonic
 * $ rostopic echo /ir_sensor
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

// Publishers
std_msgs::Float32 distance_msg;
std_msgs::Bool ir_msg;
ros::Publisher ultrasonic_pub("ultrasonic", &distance_msg);
ros::Publisher ir_pub("ir_sensor", &ir_msg);

// Ultrasonic pins
const int trigPin = 9;
const int echoPin = 10;

// IR sensor pin
const int irPin = 7;

long duration;
float distance;
bool obstacle;

void setup() {
  // Ultrasonic setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // IR sensor setup
  pinMode(irPin, INPUT);
  
  // ROS setup
  nh.initNode();
  nh.advertise(ultrasonic_pub);
  nh.advertise(ir_pub);
}

void loop() {
  // Read Ultrasonic Sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  
  // Read IR Sensor (LOW = obstacle detected)
  obstacle = (digitalRead(irPin) == LOW);
  
  // Publish to ROS
  distance_msg.data = distance;
  ultrasonic_pub.publish(&distance_msg);
  
  ir_msg.data = obstacle;
  ir_pub.publish(&ir_msg);
  
  nh.spinOnce();
  delay(100);
}
