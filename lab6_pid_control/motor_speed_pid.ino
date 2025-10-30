/*
 * Lab 6: Motor Speed PID Control
 * 엔코더를 사용한 모터 속도 PID 제어
 * 
 * Hardware:
 * - DC Motor with Encoder
 * - L298N Motor Driver
 * - Arduino Uno
 * 
 * Connections:
 * Motor:
 *   - ENA -> Pin 9 (PWM)
 *   - IN1 -> Pin 8
 *   - IN2 -> Pin 7
 * Encoder:
 *   - Channel A -> Pin 2 (Interrupt)
 *   - Channel B -> Pin 3 (Interrupt)
 */

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
float targetSpeed = 100.0;  // Target RPM
float currentSpeed = 0.0;
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float derivative = 0.0;
int motorPWM = 0;

// Timing
unsigned long lastTime = 0;
const unsigned long sampleTime = 100;  // 100ms

// Encoder parameters
const int pulsesPerRev = 360;  // Adjust for your encoder

void setup() {
  Serial.begin(9600);
  
  // Motor setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Encoder setup
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, RISING);
  
  Serial.println("Motor Speed PID Controller Ready");
  Serial.print("Target Speed: ");
  Serial.print(targetSpeed);
  Serial.println(" RPM");
  
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
    
    // Anti-windup
    integral = constrain(integral, -100, 100);
    
    derivative = (error - lastError) / (timeChange / 1000.0);
    
    // Calculate output
    float output = Kp * error + Ki * integral + Kd * derivative;
    motorPWM = constrain(motorPWM + output, 0, 255);
    
    // Apply to motor
    setMotorSpeed(motorPWM);
    
    // Debug output
    Serial.print("Target: ");
    Serial.print(targetSpeed);
    Serial.print(" | Current: ");
    Serial.print(currentSpeed);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | PWM: ");
    Serial.println(motorPWM);
    
    // Store values
    lastEncoderCount = encoderCount;
    lastError = error;
    lastTime = now;
  }
  
  // Read serial commands for target speed
  if (Serial.available() > 0) {
    targetSpeed = Serial.parseFloat();
    Serial.print("New target speed: ");
    Serial.println(targetSpeed);
  }
  
  delay(10);
}

void encoderISR() {
  // Read encoder state
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
