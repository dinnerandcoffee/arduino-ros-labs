/*
 * Lab 6: ROS Motor Speed Control with PID
 * ROS와 PID를 결합한 모터 속도 제어
 * 
 * Hardware:
 * - DC Motor with Encoder
 * - L298N Motor Driver
 * - Arduino Uno
 * 
 * ROS 실행:
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyUSB0
 * $ rostopic pub /target_speed std_msgs/Float32 "data: 100.0"  # Set target RPM
 * $ rostopic echo /current_speed  # Monitor current speed
 */

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

// Motor pins
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// Encoder pins
const int encoderA = 2;
const int encoderB = 3;

// Encoder variables
volatile long encoderCount = 0;
long lastEncoderCount = 0;

// PID parameters
float Kp = 1.5;
float Ki = 0.8;
float Kd = 0.05;

// PID variables
float targetSpeed = 0.0;
float currentSpeed = 0.0;
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
int motorPWM = 0;

// Timing
unsigned long lastTime = 0;
const unsigned long sampleTime = 100;

// Encoder parameters
const int pulsesPerRev = 360;

// ROS Publishers
std_msgs::Float32 speed_msg;
ros::Publisher speed_pub("current_speed", &speed_msg);

// ROS Subscribers
void targetSpeedCallback(const std_msgs::Float32& msg) {
  targetSpeed = msg.data;
}

ros::Subscriber<std_msgs::Float32> target_sub("target_speed", &targetSpeedCallback);

void setup() {
  // Motor setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Encoder setup
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, RISING);
  
  // ROS setup
  nh.initNode();
  nh.advertise(speed_pub);
  nh.subscribe(target_sub);
  
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  unsigned long timeChange = now - lastTime;
  
  if (timeChange >= sampleTime) {
    // Calculate current speed (RPM)
    long pulses = encoderCount - lastEncoderCount;
    currentSpeed = (pulses / (float)pulsesPerRev) * (60000.0 / timeChange);
    
    // Calculate PID
    error = targetSpeed - currentSpeed;
    integral += error * (timeChange / 1000.0);
    integral = constrain(integral, -100, 100);  // Anti-windup
    
    float derivative = (error - lastError) / (timeChange / 1000.0);
    
    float output = Kp * error + Ki * integral + Kd * derivative;
    motorPWM = constrain(motorPWM + output, 0, 255);
    
    // Apply to motor
    setMotorSpeed(motorPWM);
    
    // Publish current speed to ROS
    speed_msg.data = currentSpeed;
    speed_pub.publish(&speed_msg);
    
    // Store values
    lastEncoderCount = encoderCount;
    lastError = error;
    lastTime = now;
  }
  
  nh.spinOnce();
  delay(10);
}

void encoderISR() {
  if (digitalRead(encoderB) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setMotorSpeed(int pwm) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, abs(pwm));
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }
}
