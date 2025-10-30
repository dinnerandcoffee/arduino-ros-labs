/*
 * Lab 6: PID 제어 구현 (PID Control Implementation)
 * 
 * 모터 속도를 PID 제어로 정확하게 유지합니다.
 * 
 * 필요 사항:
 * - 모터 인코더
 * - PID 라이브러리 또는 직접 구현
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

// 모터 핀
const int ENA = 3, IN1 = 5, IN2 = 6;
const int ENB = 9, IN3 = 10, IN4 = 11;

// 인코더 핀
const int ENCODER_LEFT_A = 2;
const int ENCODER_RIGHT_A = 18;  // Pin 18 = A4

// PID 상수
float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

// PID 변수 - 왼쪽 모터
float left_target_speed = 0;
float left_current_speed = 0;
float left_error = 0;
float left_last_error = 0;
float left_integral = 0;
float left_derivative = 0;
int left_pwm = 0;

// PID 변수 - 오른쪽 모터
float right_target_speed = 0;
float right_current_speed = 0;
float right_error = 0;
float right_last_error = 0;
float right_integral = 0;
float right_derivative = 0;
int right_pwm = 0;

// 인코더 카운트
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;
long last_left_count = 0;
long last_right_count = 0;

// 타이머
unsigned long last_time = 0;
unsigned long last_pub_time = 0;

// 퍼블리셔
std_msgs::Float32 left_speed_msg;
std_msgs::Float32 right_speed_msg;
ros::Publisher pub_left_speed("/motor/left_speed", &left_speed_msg);
ros::Publisher pub_right_speed("/motor/right_speed", &right_speed_msg);

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear = msg.linear.x;
  float angular = msg.angular.z;
  
  // 목표 속도 설정 (RPM 또는 임의 단위)
  left_target_speed = (linear - angular * 0.5) * 100;
  right_target_speed = (linear + angular * 0.5) * 100;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVelCallback);

void setup() {
  // 모터 핀 설정
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // 인코더 설정
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_left_speed);
  nh.advertise(pub_right_speed);
  
  last_time = millis();
  last_pub_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;  // 초 단위
  
  if (dt >= 0.05) {  // 20Hz PID 업데이트
    // 현재 속도 계산 (인코더 변화량 / 시간)
    long left_delta = left_encoder_count - last_left_count;
    long right_delta = right_encoder_count - last_right_count;
    
    left_current_speed = left_delta / dt;
    right_current_speed = right_delta / dt;
    
    // 왼쪽 모터 PID 계산
    left_error = left_target_speed - left_current_speed;
    left_integral += left_error * dt;
    left_integral = constrain(left_integral, -100, 100);  // Anti-windup
    left_derivative = (left_error - left_last_error) / dt;
    
    left_pwm = Kp * left_error + Ki * left_integral + Kd * left_derivative;
    left_pwm = constrain(left_pwm, -255, 255);
    
    // 오른쪽 모터 PID 계산
    right_error = right_target_speed - right_current_speed;
    right_integral += right_error * dt;
    right_integral = constrain(right_integral, -100, 100);
    right_derivative = (right_error - right_last_error) / dt;
    
    right_pwm = Kp * right_error + Ki * right_integral + Kd * right_derivative;
    right_pwm = constrain(right_pwm, -255, 255);
    
    // 모터에 PWM 적용
    setMotor(ENA, IN1, IN2, left_pwm);
    setMotor(ENB, IN3, IN4, right_pwm);
    
    // 상태 저장
    last_left_count = left_encoder_count;
    last_right_count = right_encoder_count;
    left_last_error = left_error;
    right_last_error = right_error;
    last_time = current_time;
  }
  
  // 속도 데이터 퍼블리시 (10Hz)
  if (current_time - last_pub_time > 100) {
    left_speed_msg.data = left_current_speed;
    right_speed_msg.data = right_current_speed;
    pub_left_speed.publish(&left_speed_msg);
    pub_right_speed.publish(&right_speed_msg);
    last_pub_time = current_time;
  }
  
  nh.spinOnce();
}

void setMotor(int en_pin, int in1_pin, int in2_pin, int pwm) {
  if (pwm >= 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    analogWrite(en_pin, pwm);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    analogWrite(en_pin, -pwm);
  }
}

void leftEncoderISR() {
  left_encoder_count++;
}

void rightEncoderISR() {
  right_encoder_count++;
}
