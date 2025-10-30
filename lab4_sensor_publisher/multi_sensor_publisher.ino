/*
 * Lab 4: 다중 센서 퍼블리셔 (Multi-Sensor Publisher)
 * 
 * 초음파 센서와 IR 센서 데이터를 ROS로 전송합니다.
 */

#include <ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

// 초음파 센서
sensor_msgs::Range ultrasonic_msg;
ros::Publisher pub_ultrasonic("/sensors/ultrasonic", &ultrasonic_msg);

// IR 센서
std_msgs::Bool ir_msg;
ros::Publisher pub_ir("/sensors/ir_obstacle", &ir_msg);

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
const int IR_PIN = 7;

unsigned long sensor_timer;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  
  nh.initNode();
  nh.advertise(pub_ultrasonic);
  nh.advertise(pub_ir);
  
  // 초음파 센서 메시지 설정
  ultrasonic_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  ultrasonic_msg.header.frame_id = "ultrasonic_link";
  ultrasonic_msg.field_of_view = 0.26;
  ultrasonic_msg.min_range = 0.02;
  ultrasonic_msg.max_range = 4.0;
  
  sensor_timer = millis();
}

void loop() {
  if (millis() - sensor_timer > 100) {  // 10Hz
    // 초음파 센서 읽기
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.034 / 2 / 100.0;
    
    ultrasonic_msg.range = distance;
    ultrasonic_msg.header.stamp = nh.now();
    pub_ultrasonic.publish(&ultrasonic_msg);
    
    // IR 센서 읽기
    int ir_state = digitalRead(IR_PIN);
    ir_msg.data = (ir_state == LOW);  // LOW일 때 장애물 감지
    pub_ir.publish(&ir_msg);
    
    sensor_timer = millis();
  }
  
  nh.spinOnce();
}
