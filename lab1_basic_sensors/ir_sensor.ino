/*
 * Lab 1: 적외선 센서 제어 (IR Sensor Control)
 * 
 * IR 장애물 감지 센서를 사용하여 물체를 감지합니다.
 * 
 * 연결:
 * - VCC -> 5V
 * - GND -> GND
 * - OUT -> Pin 7
 */

const int IR_PIN = 7;

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
  
  Serial.println("IR 센서 테스트 시작");
}

void loop() {
  int ir_state = digitalRead(IR_PIN);
  
  if (ir_state == LOW) {
    Serial.println("장애물 감지!");
  } else {
    Serial.println("장애물 없음");
  }
  
  delay(200);
}
