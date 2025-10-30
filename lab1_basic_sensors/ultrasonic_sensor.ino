/*
 * Lab 1: 초음파 센서 제어 (Ultrasonic Sensor Control)
 * 
 * HC-SR04 초음파 센서를 사용하여 거리를 측정합니다.
 * 
 * 연결:
 * - VCC -> 5V
 * - GND -> GND
 * - TRIG -> Pin 9
 * - ECHO -> Pin 10
 */

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("초음파 센서 테스트 시작");
}

void loop() {
  long duration, distance;
  
  // 트리거 펄스 생성
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 에코 핀에서 펄스 읽기
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // 거리 계산 (cm)
  distance = duration * 0.034 / 2;
  
  Serial.print("거리: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  delay(500);
}
