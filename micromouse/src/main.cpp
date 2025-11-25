#include <Arduino.h>
#include <ESP32Servo.h>

// ESP32 Pin Configuration - Adjust based on your wiring
const int LEFT_SENSOR = 34;    // GPIO34 (Analog capable)
const int RIGHT_SENSOR = 35;   // GPIO35 (Analog capable)
const int FRONT_LEFT_SENSOR = 32;  // Additional sensors for better tracking
const int FRONT_RIGHT_SENSOR = 33;

// Motor control pins (Using L298N driver example)
const int LEFT_MOTOR_IN1 = 25;
const int LEFT_MOTOR_IN2 = 26;
const int RIGHT_MOTOR_IN1 = 27;
const int RIGHT_MOTOR_IN2 = 14;
const int MOTOR_PWM_FREQ = 5000;
const int MOTOR_PWM_RESOLUTION = 8;

// PWM channels for ESP32
const int LEFT_MOTOR_PWM_CHANNEL = 0;
const int RIGHT_MOTOR_PWM_CHANNEL = 1;

// PID Constants
float Kp = 0.8;   // Proportional gain
float Ki = 0.01;  // Integral gain  
float Kd = 0.3;   // Derivative gain

// Global variables
float integral = 0;
float previous_error = 0;
unsigned long previous_time = 0;

// Sensor calibration values
int sensor_min[] = {4095, 4095, 4095, 4095};
int sensor_max[] = {0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  
  // Configure motor pins
  ledcSetup(LEFT_MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(RIGHT_MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  
  ledcAttachPin(LEFT_MOTOR_IN1, LEFT_MOTOR_PWM_CHANNEL);
  ledcAttachPin(LEFT_MOTOR_IN2, LEFT_MOTOR_PWM_CHANNEL);
  ledcAttachPin(RIGHT_MOTOR_IN1, RIGHT_MOTOR_PWM_CHANNEL);
  ledcAttachPin(RIGHT_MOTOR_IN2, RIGHT_MOTOR_PWM_CHANNEL);

  // Sensor pins are analog by default, no need to set mode
  
  Serial.println("ESP32 Micromouse Line Follower Started");
  Serial.println("Calibrating sensors...");
  
  // Quick calibration
  calibrateSensors();
  
  Serial.println("Calibration complete!");
  delay(2000);
}

void calibrateSensors() {
  unsigned long start_time = millis();
  while (millis() - start_time < 5000) { // 5 second calibration
    for (int i = 0; i < 4; i++) {
      int sensor_pins[] = {LEFT_SENSOR, RIGHT_SENSOR, FRONT_LEFT_SENSOR, FRONT_RIGHT_SENSOR};
      int value = analogRead(sensor_pins[i]);
      
      if (value < sensor_min[i]) sensor_min[i] = value;
      if (value > sensor_max[i]) sensor_max[i] = value;
    }
    delay(100);
  }
}

float readSensor(int sensor_pin, int sensor_index) {
  int raw_value = analogRead(sensor_pin);
  // Normalize to 0-1000 range
  float normalized = map(raw_value, sensor_min[sensor_index], sensor_max[sensor_index], 0, 1000);
  return constrain(normalized, 0, 1000);
}

float calculateError() {
  float left_value = readSensor(LEFT_SENSOR, 0);
  float right_value = readSensor(RIGHT_SENSOR, 1);
  float front_left_value = readSensor(FRONT_LEFT_SENSOR, 2);
  float front_right_value = readSensor(FRONT_RIGHT_SENSOR, 3);
  
  // Weighted error calculation
  float error = (front_left_value - front_right_value) * 1.5 + 
                (left_value - right_value) * 0.5;
  
  Serial.printf("Sensors: L:%.1f R:%.1f FL:%.1f FR:%.1f Error: %.2f\n", 
                left_value, right_value, front_left_value, front_right_value, error);
  
  return error;
}

void setMotorSpeed(int left_speed, int right_speed) {
  // Constrain speeds to -255 to 255 range
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  // Left motor control
  if (left_speed > 0) {
    ledcWrite(LEFT_MOTOR_PWM_CHANNEL, abs(left_speed));
    digitalWrite(LEFT_MOTOR_IN2, LOW);
  } else {
    ledcWrite(LEFT_MOTOR_PWM_CHANNEL, abs(left_speed));
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
  }
  
  // Right motor control
  if (right_speed > 0) {
    ledcWrite(RIGHT_MOTOR_PWM_CHANNEL, abs(right_speed));
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
  } else {
    ledcWrite(RIGHT_MOTOR_PWM_CHANNEL, abs(right_speed));
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  }
}

void pidControl() {
  unsigned long current_time = millis();
  float delta_time = (current_time - previous_time) / 1000.0; // Convert to seconds
  
  if (delta_time == 0) delta_time = 0.01; // Prevent division by zero
  
  float error = calculateError();
  
  // PID calculations
  float proportional = error;
  integral += error * delta_time;
  integral = constrain(integral, -100, 100); // Anti-windup
  float derivative = (error - previous_error) / delta_time;
  
  float output = Kp * proportional + Ki * integral + Kd * derivative;
  
  // Motor speed calculation
  int base_speed = 150; // Base speed (0-255)
  int left_speed = base_speed - output;
  int right_speed = base_speed + output;
  
  setMotorSpeed(left_speed, right_speed);
  
  Serial.printf("PID - P:%.2f I:%.2f D:%.2f Output:%.2f L:%d R:%d\n", 
                proportional, integral, derivative, output, left_speed, right_speed);
  
  previous_error = error;
  previous_time = current_time;
}

void loop() {
  pidControl();
  delay(20); // ~50Hz update rate
}