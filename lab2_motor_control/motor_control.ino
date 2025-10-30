/*
 * Lab 2: DC 모터 제어 (DC Motor Control with L298N)
 * 
 * L298N 모터 드라이버를 사용하여 DC 모터를 제어합니다.
 * 
 * 연결:
 * - IN1 -> Pin 5
 * - IN2 -> Pin 6
 * - IN3 -> Pin 10
 * - IN4 -> Pin 11
 * - ENA -> Pin 3 (PWM)
 * - ENB -> Pin 9 (PWM)
 */

// 모터 A (왼쪽)
const int ENA = 3;
const int IN1 = 5;
const int IN2 = 6;

// 모터 B (오른쪽)
const int ENB = 9;
const int IN3 = 10;
const int IN4 = 11;

void setup() {
  Serial.begin(9600);
  
  // 모터 핀 설정
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  Serial.println("모터 제어 테스트 시작");
}

void loop() {
  Serial.println("전진");
  moveForward(200);
  delay(2000);
  
  Serial.println("정지");
  stopMotors();
  delay(1000);
  
  Serial.println("후진");
  moveBackward(200);
  delay(2000);
  
  Serial.println("정지");
  stopMotors();
  delay(1000);
  
  Serial.println("좌회전");
  turnLeft(150);
  delay(1000);
  
  Serial.println("정지");
  stopMotors();
  delay(1000);
  
  Serial.println("우회전");
  turnRight(150);
  delay(1000);
  
  Serial.println("정지");
  stopMotors();
  delay(2000);
}

void moveForward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
