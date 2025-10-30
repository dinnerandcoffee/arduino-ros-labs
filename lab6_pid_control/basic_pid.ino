/*
 * Lab 6: Basic PID Controller
 * 기본 PID 제어기 구현
 * 
 * 이 예제는 기본적인 PID 제어 알고리즘을 보여줍니다.
 * 위치나 속도 제어에 사용할 수 있습니다.
 */

// PID parameters
float Kp = 2.0;  // Proportional gain
float Ki = 0.5;  // Integral gain
float Kd = 0.1;  // Derivative gain

// PID variables
float setpoint = 100.0;  // Target value
float input = 0.0;       // Current value
float output = 0.0;      // Control output

float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float derivative = 0.0;

unsigned long lastTime = 0;
float timeChange;

void setup() {
  Serial.begin(9600);
  Serial.println("PID Controller Ready");
  Serial.println("Setpoint: 100.0");
  lastTime = millis();
}

void loop() {
  // Simulate sensor reading (replace with actual sensor)
  input = analogRead(A0) * (200.0 / 1023.0);  // Scale to 0-200
  
  // Calculate PID
  unsigned long now = millis();
  timeChange = (now - lastTime) / 1000.0;  // Convert to seconds
  
  if (timeChange >= 0.1) {  // Update every 100ms
    // Calculate error
    error = setpoint - input;
    
    // Proportional term
    float P = Kp * error;
    
    // Integral term
    integral += error * timeChange;
    float I = Ki * integral;
    
    // Derivative term
    derivative = (error - lastError) / timeChange;
    float D = Kd * derivative;
    
    // Calculate output
    output = P + I + D;
    
    // Constrain output (if needed)
    output = constrain(output, -255, 255);
    
    // Print debug information
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | Input: ");
    Serial.print(input);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Output: ");
    Serial.println(output);
    
    // Store values for next iteration
    lastError = error;
    lastTime = now;
  }
  
  delay(10);
}
