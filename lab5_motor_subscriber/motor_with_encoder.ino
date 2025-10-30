/*
 * Lab 5: 향상된 모터 제어 (Enhanced Motor Control)
 * 
 * cmd_vel 구독 + 오도메트리 퍼블리시
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

// 모터 핀
const int ENA = 3, IN1 = 5, IN2 = 6;
const int ENB = 9, IN3 = 10, IN4 = 11;

// 인코더 핀 (옵션)
const int ENCODER_LEFT_A = 2;
const int ENCODER_RIGHT_A = 3;

volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

std_msgs::Int32 left_enc_msg;
std_msgs::Int32 right_enc_msg;

ros::Publisher pub_left_enc("/encoder/left", &left_enc_msg);
ros::Publisher pub_right_enc("/encoder/right", &right_enc_msg);

unsigned long last_pub_time = 0;

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear = msg.linear.x;
  float angular = msg.angular.z;
  
  float left = linear - angular * 0.5;
  float right = linear + angular * 0.5;
  
  // 속도 제한
  left = constrain(left, -1.0, 1.0);
  right = constrain(right, -1.0, 1.0);
  
  setMotor(ENA, IN1, IN2, left);
  setMotor(ENB, IN3, IN4, right);
}

void setMotor(int en_pin, int in1_pin, int in2_pin, float speed) {
  int pwm = abs(speed * 255);
  
  if (speed >= 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  }
  
  analogWrite(en_pin, pwm);
}

void leftEncoderISR() {
  left_encoder_count++;
}

void rightEncoderISR() {
  right_encoder_count++;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVelCallback);

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // 인코더 설정 (옵션)
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_left_enc);
  nh.advertise(pub_right_enc);
  
  last_pub_time = millis();
}

void loop() {
  // 인코더 값 퍼블리시 (10Hz)
  if (millis() - last_pub_time > 100) {
    left_enc_msg.data = left_encoder_count;
    right_enc_msg.data = right_encoder_count;
    
    pub_left_enc.publish(&left_enc_msg);
    pub_right_enc.publish(&right_enc_msg);
    
    last_pub_time = millis();
  }
  
  nh.spinOnce();
  delay(10);
}
