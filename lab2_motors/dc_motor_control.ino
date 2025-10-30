/*
 * Lab 2: DC Motor Control with L298N
 * L298N 모터 드라이버를 사용한 DC 모터 제어
 * 
 * Hardware:
 * - L298N Motor Driver
 * - DC Motor
 * - Arduino Uno
 * 
 * Connections:
 * - ENA -> Pin 9 (PWM)
 * - IN1 -> Pin 8
 * - IN2 -> Pin 7
 * - Motor A -> Motor terminals
 * - 12V Power Supply -> Motor driver
 * - GND -> Common ground
 */

// Motor A connections
const int enA = 9;   // PWM pin for speed control
const int in1 = 8;
const int in2 = 7;

void setup() {
  Serial.begin(9600);
  
  // Set all motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  Serial.println("DC Motor Control Ready");
}

void loop() {
  // Forward at full speed
  Serial.println("Moving Forward - Full Speed");
  moveForward(255);
  delay(2000);
  
  // Forward at half speed
  Serial.println("Moving Forward - Half Speed");
  moveForward(128);
  delay(2000);
  
  // Stop
  Serial.println("Stopping");
  stopMotor();
  delay(1000);
  
  // Backward at full speed
  Serial.println("Moving Backward - Full Speed");
  moveBackward(255);
  delay(2000);
  
  // Stop
  Serial.println("Stopping");
  stopMotor();
  delay(1000);
}

void moveForward(int speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, speed);
}

void moveBackward(int speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, speed);
}

void stopMotor() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
}
