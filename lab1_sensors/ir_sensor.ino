/*
 * Lab 1: IR Sensor Reading
 * 적외선 센서를 사용하여 장애물 감지
 * 
 * Hardware:
 * - IR Obstacle Avoidance Sensor
 * - Arduino Uno
 * 
 * Connections:
 * - VCC -> 5V
 * - GND -> GND
 * - OUT -> Pin 7
 */

const int irPin = 7;
int sensorValue;

void setup() {
  Serial.begin(9600);
  pinMode(irPin, INPUT);
  
  Serial.println("IR Sensor Ready");
}

void loop() {
  sensorValue = digitalRead(irPin);
  
  if (sensorValue == LOW) {
    Serial.println("Obstacle Detected!");
  } else {
    Serial.println("No Obstacle");
  }
  
  delay(100);
}
