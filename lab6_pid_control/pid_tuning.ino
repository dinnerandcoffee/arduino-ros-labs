/*
 * Lab 6: PID 튜닝 도구 (PID Tuning Tool)
 * 
 * 동적으로 PID 파라미터를 조정할 수 있습니다.
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

// 모터 및 인코더 핀 (동일)
const int ENA = 3, IN1 = 5, IN2 = 6;
const int ENB = 9, IN3 = 10, IN4 = 11;
const int ENCODER_LEFT_A = 2;
const int ENCODER_RIGHT_A = 18;

// PID 파라미터 (동적 조정 가능)
float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

// PID 변수
float left_target = 0, left_current = 0, left_error = 0, left_last_error = 0;
float left_integral = 0, left_derivative = 0;
float right_target = 0, right_current = 0, right_error = 0, right_last_error = 0;
float right_integral = 0, right_derivative = 0;

volatile long left_enc = 0, right_enc = 0;
long last_left = 0, last_right = 0;
unsigned long last_time = 0;

// PID 파라미터 수신 콜백
void pidParamsCallback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length >= 3) {
    Kp = msg.data[0];
    Ki = msg.data[1];
    Kd = msg.data[2];
    
    // 적분 항 리셋 (파라미터 변경 시)
    left_integral = 0;
    right_integral = 0;
  }
}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  left_target = (msg.linear.x - msg.angular.z * 0.5) * 100;
  right_target = (msg.linear.x + msg.angular.z * 0.5) * 100;
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd("/cmd_vel", &cmdVelCallback);
ros::Subscriber<std_msgs::Float32MultiArray> sub_pid("/pid_params", &pidParamsCallback);

std_msgs::Float32 left_speed_msg, right_speed_msg;
std_msgs::Float32MultiArray pid_status_msg;
ros::Publisher pub_left("/motor/left_speed", &left_speed_msg);
ros::Publisher pub_right("/motor/right_speed", &right_speed_msg);
ros::Publisher pub_pid("/pid_status", &pid_status_msg);

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), []() { left_enc++; }, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), []() { right_enc++; }, RISING);
  
  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.subscribe(sub_pid);
  nh.advertise(pub_left);
  nh.advertise(pub_right);
  nh.advertise(pub_pid);
  
  // PID 상태 메시지 초기화
  pid_status_msg.data_length = 6;
  pid_status_msg.data = (float*)malloc(sizeof(float) * 6);
  
  last_time = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  
  if (dt >= 0.05) {  // 20Hz
    // 속도 계산
    left_current = (left_enc - last_left) / dt;
    right_current = (right_enc - last_right) / dt;
    
    // PID 계산
    left_error = left_target - left_current;
    left_integral += left_error * dt;
    left_integral = constrain(left_integral, -100, 100);
    left_derivative = (left_error - left_last_error) / dt;
    int left_pwm = Kp * left_error + Ki * left_integral + Kd * left_derivative;
    
    right_error = right_target - right_current;
    right_integral += right_error * dt;
    right_integral = constrain(right_integral, -100, 100);
    right_derivative = (right_error - right_last_error) / dt;
    int right_pwm = Kp * right_error + Ki * right_integral + Kd * right_derivative;
    
    // 모터 제어
    setMotor(ENA, IN1, IN2, constrain(left_pwm, -255, 255));
    setMotor(ENB, IN3, IN4, constrain(right_pwm, -255, 255));
    
    // 상태 업데이트
    last_left = left_enc;
    last_right = right_enc;
    left_last_error = left_error;
    right_last_error = right_error;
    last_time = now;
    
    // 퍼블리시
    left_speed_msg.data = left_current;
    right_speed_msg.data = right_current;
    pub_left.publish(&left_speed_msg);
    pub_right.publish(&right_speed_msg);
    
    // PID 상태 퍼블리시
    pid_status_msg.data[0] = Kp;
    pid_status_msg.data[1] = Ki;
    pid_status_msg.data[2] = Kd;
    pid_status_msg.data[3] = left_error;
    pid_status_msg.data[4] = right_error;
    pid_status_msg.data[5] = dt;
    pub_pid.publish(&pid_status_msg);
  }
  
  nh.spinOnce();
}

void setMotor(int en, int in1, int in2, int pwm) {
  if (pwm >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -pwm);
  }
}
