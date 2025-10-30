/*
 * Lab 4: 센서 데이터 퍼블리셔 (Sensor Data Publisher)
 * 
 * 초음파 센서 데이터를 ROS 토픽으로 전송합니다.
 * 
 * 토픽: /ultrasonic
 * 메시지 타입: sensor_msgs/Range
 */

#include <ros.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range("/ultrasonic", &range_msg);

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

const float MIN_RANGE = 0.02;  // 2cm
const float MAX_RANGE = 4.0;   // 400cm

unsigned long range_timer;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  nh.initNode();
  nh.advertise(pub_range);
  
  // Range 메시지 헤더 설정
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id = "ultrasonic_sensor";
  range_msg.field_of_view = 0.26;  // 약 15도
  range_msg.min_range = MIN_RANGE;
  range_msg.max_range = MAX_RANGE;
  
  range_timer = millis();
}

void loop() {
  if (millis() - range_timer > 100) {  // 10Hz
    long duration;
    float distance;
    
    // 초음파 센서 측정
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2 / 100.0;  // 미터 단위로 변환
    
    // ROS 메시지 퍼블리시
    range_msg.range = distance;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    
    range_timer = millis();
  }
  
  nh.spinOnce();
}
