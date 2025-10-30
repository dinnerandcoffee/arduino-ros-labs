/*
 * Lab 2: Servo Motor Control
 * 서보 모터 위치 제어
 * 
 * Hardware:
 * - SG90 Servo Motor
 * - Arduino Uno
 * 
 * Connections:
 * - Brown/Black (GND) -> GND
 * - Red (VCC) -> 5V
 * - Orange/Yellow (Signal) -> Pin 6
 */

#include <Servo.h>

Servo myServo;
const int servoPin = 6;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  
  Serial.println("Servo Motor Control Ready");
}

void loop() {
  // Sweep from 0 to 180 degrees
  Serial.println("Sweeping 0 to 180 degrees");
  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);
    Serial.print("Position: ");
    Serial.println(pos);
    delay(15);
  }
  
  delay(500);
  
  // Sweep from 180 to 0 degrees
  Serial.println("Sweeping 180 to 0 degrees");
  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);
    Serial.print("Position: ");
    Serial.println(pos);
    delay(15);
  }
  
  delay(500);
}
