/*
 * Simple Micromouse Motor Test using SparkFun TB6612 Library
 * Basic forward, backward, left, right movements
 * Without using standby pin
 */

#include <SparkFun_TB6612.h>

// Motor pins
#define AIN1 5
#define AIN2 4
#define PWMA 3
#define BIN1 8
#define BIN2 9
#define PWMB 6

// Motor speed (0-255)
const int motorSpeed = 255;

// Create motor objects without standby pin
// Use 255 as dummy value for standby when not using it
Motor motorLeft = Motor(AIN1, AIN2, PWMA, 1, 255);   // Channel A, offset 1
Motor motorRight = Motor(BIN1, BIN2, PWMB, 1, 255);  // Channel B, offset 1

void setup() {
  // Stop motors initially
  brake();
  
  Serial.begin(9600);
  Serial.println("Simple Motor Test Ready - TB6612 Version (No Standby)");
  Serial.println("F=Forward, B=Backward, L=Left, R=Right, S=Stop");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case 'F': forward(); break;
      case 'B': backward(); break;
      case 'L': left(); break;
      case 'R': right(); break;
      case 'S': brake(); break;
    }
  }
}

void forward() {
  Serial.println("Moving FORWARD");
  motorLeft.drive(motorSpeed);
  motorRight.drive(motorSpeed);
}

void backward() {
  Serial.println("Moving BACKWARD");
  motorLeft.drive(-motorSpeed);
  motorRight.drive(-motorSpeed);
}

void left() {
  Serial.println("Turning LEFT");
  motorLeft.drive(-motorSpeed);
  motorRight.drive(motorSpeed);
}

void right() {
  Serial.println("Turning RIGHT");
  motorLeft.drive(motorSpeed);
  motorRight.drive(-motorSpeed);
}

void brake() {
  Serial.println("STOPPING motors");
  motorLeft.brake();
  motorRight.brake();
}