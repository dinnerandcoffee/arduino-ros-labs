/*
 * Lab 1: Ultrasonic Sensor Reading
 * 초음파 센서를 사용하여 거리 측정
 * 
 * Hardware:
 * - HC-SR04 Ultrasonic Sensor
 * - Arduino Uno
 * 
 * Connections:
 * - VCC -> 5V
 * - GND -> GND
 * - Trig -> Pin 9
 * - Echo -> Pin 10
 */

const int trigPin = 9;
const int echoPin = 10;

long duration;
float distance;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.println("Ultrasonic Sensor Ready");
}

void loop() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Trigger the sensor by setting trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance (Speed of sound = 340 m/s)
  distance = duration * 0.034 / 2;
  
  // Print the distance
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  delay(100);
}
