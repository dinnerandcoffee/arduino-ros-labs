/*
 * Lab 6: Line Following Robot with PID
 * PID 제어를 사용한 라인 트레이싱 로봇
 * 
 * Hardware:
 * - 5-Channel IR Line Sensor Array
 * - L298N Motor Driver (Dual Channel)
 * - DC Motors (Left, Right)
 * - Arduino Uno
 * 
 * Connections:
 * Line Sensors (Analog):
 *   - Sensor 0 (Leftmost) -> A0
 *   - Sensor 1 -> A1
 *   - Sensor 2 (Center) -> A2
 *   - Sensor 3 -> A3
 *   - Sensor 4 (Rightmost) -> A4
 * 
 * Left Motor:
 *   - ENA -> Pin 9 (PWM)
 *   - IN1 -> Pin 8
 *   - IN2 -> Pin 7
 * 
 * Right Motor:
 *   - ENB -> Pin 3 (PWM)
 *   - IN3 -> Pin 5
 *   - IN4 -> Pin 4
 */

// Motor pins
const int leftMotorPWM = 9;
const int leftMotor1 = 8;
const int leftMotor2 = 7;
const int rightMotorPWM = 3;
const int rightMotor1 = 5;
const int rightMotor2 = 4;

// Line sensor pins
const int sensors[5] = {A0, A1, A2, A3, A4};
int sensorValues[5];

// PID parameters
float Kp = 0.8;
float Ki = 0.0;
float Kd = 0.5;

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

// Motor speeds
const int baseSpeed = 150;  // Base motor speed (0-255)
int leftSpeed = 0;
int rightSpeed = 0;

// Line position weights
const int weights[5] = {-2, -1, 0, 1, 2};

void setup() {
  Serial.begin(9600);
  
  // Motor setup
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  
  // Sensor setup
  for (int i = 0; i < 5; i++) {
    pinMode(sensors[i], INPUT);
  }
  
  Serial.println("Line Following Robot with PID Ready");
  delay(2000);  // Wait 2 seconds before starting
}

void loop() {
  // Read line sensors
  readSensors();
  
  // Calculate line position (error)
  error = calculateError();
  
  // Calculate PID
  integral += error;
  integral = constrain(integral, -100, 100);  // Anti-windup
  derivative = error - lastError;
  
  correction = Kp * error + Ki * integral + Kd * derivative;
  
  // Calculate motor speeds
  leftSpeed = baseSpeed + correction;
  rightSpeed = baseSpeed - correction;
  
  // Constrain speeds
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Set motor speeds
  setMotors(leftSpeed, rightSpeed);
  
  // Debug output
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | Correction: ");
  Serial.print(correction);
  Serial.print(" | L: ");
  Serial.print(leftSpeed);
  Serial.print(" | R: ");
  Serial.println(rightSpeed);
  
  lastError = error;
  delay(10);
}

void readSensors() {
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensors[i]);
    // Normalize to 0 or 1 (adjust threshold as needed)
    sensorValues[i] = (sensorValues[i] > 512) ? 1 : 0;
  }
}

float calculateError() {
  // Calculate weighted sum
  float weightedSum = 0;
  int activeCount = 0;
  
  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] == 1) {  // 1 = line detected
      weightedSum += weights[i];
      activeCount++;
    }
  }
  
  // Return average position
  if (activeCount > 0) {
    return weightedSum / activeCount;
  } else {
    return lastError;  // Keep last error if no line detected
  }
}

void setMotors(int left, int right) {
  // Left motor
  if (left > 0) {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, left);
  } else if (left < 0) {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    analogWrite(leftMotorPWM, abs(left));
  } else {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, 0);
  }
  
  // Right motor
  if (right > 0) {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, right);
  } else if (right < 0) {
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(rightMotorPWM, abs(right));
  } else {
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, 0);
  }
}
