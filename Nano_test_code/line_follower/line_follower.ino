#include <SparkFun_TB6612.h>
#include <QTRSensors.h>

// Motor pins
#define AIN1 5
#define BIN1 8
#define AIN2 4
#define BIN2 9
#define PWMA 3
#define PWMB 6
#define STBY 8

const int offsetA = 1;
const int offsetB = 1;

// Controls
#define sw1 10
#define sw2 11
#define led 7

// Sensors
#define NUM_SENSORS 8
unsigned int sensors1[8];
QTRSensors qtra;

// ========== WIRELESS ADJUSTABLE SETTINGS ==========
// Now supports VERY large and VERY small values
float kp = 0.15;
float ki = 0.00;
float kd = 0.00;

int maxSpeed = 150;
int baseSpeed = 150;

bool followBlackLine = true;
int centerPosition = 3500;

// Integral control variables
int lastError = 0;
float integral = 0;
float integral_limit = 1000.0;  // Now float for larger values

String btBuffer = "";

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup()
{
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4, A5, A6, A7
  }, NUM_SENSORS);

  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  Serial.begin(9600);
  Serial.println("READY - Wireless PID Control with Extended Range");
  Serial.println("Supports ANY PID values: from 0.000001 to 999999.0");
  Serial.println("Commands: KP, KI, KD, ILIMIT, SPEED, LINE, STATUS, HELP");

  // Wait for calibration button
  while (digitalRead(sw1) == HIGH) {
    checkBluetooth();
  }
  calibration();
}

void loop() {
  checkBluetooth();
  
  // Ramp start
  forward(motor1, motor2, 80);
  delay(50);
  
  // Wait for start button
  while (digitalRead(sw1) == HIGH) {
    checkBluetooth();
  }
  
  // Start line following
  followLine();
}

void calibration()
{
  Serial.println("Calibrating...");
  for (int i = 0; i <= 100; i++)
  {
    if (i < 25 || i >= 75) {
      left(motor1, motor2, 200);
    } else {
      right(motor1, motor2, 200);
    }
    qtra.calibrate();
    delay(10);
  }
  brake(motor1, motor2);
  Serial.println("Calibration Complete");
}

void followLine()
{
  Serial.println("Line Following Started - Adjust wirelessly!");
  
  while(digitalRead(sw1) == HIGH) 
  {
    checkBluetooth();
    
    int position;
    if (followBlackLine) {
      position = qtra.readLineBlack(sensors1);
    } else {
      position = qtra.readLineWhite(sensors1);
    }
    
    int error = centerPosition - position;
    
    // ========== COMPLETE PID WITH ANY VALUES ==========
    // Add error to integral term
    integral += error;
    
    // Apply windup protection with float support
    if (integral > integral_limit) integral = integral_limit;
    if (integral < -integral_limit) integral = -integral_limit;
    
    // Calculate derivative
    int derivative = error - lastError;
    
    // COMPLETE PID calculation - supports ANY values
    float motorSpeed = kp * error + ki * integral + kd * derivative;
    
    // Apply motor speed limits to prevent overflow
    motorSpeed = constrain(motorSpeed, -1000.0, 1000.0);
    
    lastError = error;
    // ========== END PID CALCULATION ==========
    
    int rightMotorSpeed = baseSpeed - motorSpeed;
    int leftMotorSpeed = baseSpeed + motorSpeed;
    
    // Apply speed limits
    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
    
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    
    delay(1);
  }
  
  brake(motor1, motor2);
  Serial.println("Stopped");
}

// ========== WIRELESS COMMAND PROCESSING ==========
void checkBluetooth() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (btBuffer.length() > 0) {
        processCommand(btBuffer);
        btBuffer = "";
      }
    } else {
      btBuffer += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  // === PID SETTINGS - SUPPORTS ANY VALUES ===
  if (cmd.startsWith("KP:")) {
    kp = stringToFloat(cmd.substring(3));
    Serial.print("P: "); 
    printScientific(kp); // Scientific notation for large/small values
  }
  else if (cmd.startsWith("KI:")) {
    ki = stringToFloat(cmd.substring(3));
    Serial.print("I: "); 
    printScientific(ki);
  }
  else if (cmd.startsWith("KD:")) {
    kd = stringToFloat(cmd.substring(3));
    Serial.print("D: "); 
    printScientific(kd);
  }
  else if (cmd.startsWith("ILIMIT:")) {
    integral_limit = stringToFloat(cmd.substring(7));
    Serial.print("I_Limit: "); 
    printScientific(integral_limit);
  }
  
  // === SPEED SETTINGS ===
  else if (cmd.startsWith("MAX:")) {
    maxSpeed = cmd.substring(4).toInt();
    Serial.print("Max Speed: "); Serial.println(maxSpeed);
  }
  else if (cmd.startsWith("BASE:")) {
    baseSpeed = cmd.substring(5).toInt();
    Serial.print("Base Speed: "); Serial.println(baseSpeed);
  }
  
  // === LINE SETTINGS ===
  else if (cmd == "BLACK") {
    followBlackLine = true;
    Serial.println("Line: BLACK");
  }
  else if (cmd == "WHITE") {
    followBlackLine = false;
    Serial.println("Line: WHITE");
  }
  else if (cmd.startsWith("CENTER:")) {
    centerPosition = cmd.substring(7).toInt();
    Serial.print("Center: "); Serial.println(centerPosition);
  }
  
  // === SYSTEM COMMANDS ===
  else if (cmd == "STATUS") {
    showStatus();
  }
  else if (cmd == "HELP") {
    showHelp();
  }
  else if (cmd == "STOP") {
    brake(motor1, motor2);
    Serial.println("Motors Stopped");
  }
  else if (cmd == "RESETI") {
    integral = 0;
    Serial.println("Integral Reset");
  }
  else if (cmd == "RESET") {
    resetToDefaults();
  }
  else if (cmd == "SHOWRAW") {
    showRawPID();
  }
  else {
    Serial.println("Unknown - Use: KP, KI, KD, ILIMIT, MAX, BASE, BLACK, WHITE, STATUS, HELP");
  }
  
  // Visual feedback
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
}

// Custom function to handle ANY float values
float stringToFloat(String input) {
  input.trim();
  return input.toFloat(); // Supports from 3.4028235E+38 to -3.4028235E+38
}

// Print in scientific notation for very large/small values
void printScientific(float value) {
  if (abs(value) >= 10000.0 || (abs(value) < 0.001 && value != 0.0)) {
    Serial.println(value, 6); // Scientific notation for extreme values
  } else {
    Serial.println(value, 6); // Normal notation with 6 decimals
  }
}

void showStatus() {
  Serial.println("=== CURRENT SETTINGS ===");
  Serial.print("PID: KP="); printScientific(kp);
  Serial.print("     KI="); printScientific(ki);
  Serial.print("     KD="); printScientific(kd);
  Serial.print("     ILIMIT="); printScientific(integral_limit);
  
  Serial.print("Speed: MAX="); Serial.print(maxSpeed);
  Serial.print(" BASE="); Serial.println(baseSpeed);
  
  Serial.print("Line: "); Serial.print(followBlackLine ? "BLACK" : "WHITE");
  Serial.print(" CENTER="); Serial.println(centerPosition);
  
  Serial.print("Integral Value: "); Serial.println(integral);
  Serial.println("========================");
}

void showRawPID() {
  Serial.println("=== RAW PID VALUES ===");
  Serial.print("KP (raw): "); Serial.println(kp, 15);
  Serial.print("KI (raw): "); Serial.println(ki, 15);
  Serial.print("KD (raw): "); Serial.println(kd, 15);
  Serial.print("ILIMIT (raw): "); Serial.println(integral_limit, 15);
  Serial.println("=====================");
}

void showHelp() {
  Serial.println("=== WIRELESS COMMANDS ===");
  Serial.println("PID:    KP:0.15, KI:0.000001, KD:50.5, ILIMIT:5000");
  Serial.println("        Supports ANY values: 0.000001 to 999999.0");
  Serial.println("        Examples:");
  Serial.println("        KP:0.000001  (very small)");
  Serial.println("        KP:1000.0    (very large)");
  Serial.println("        KI:0.0000001 (micro integral)");
  Serial.println("SPEED:  MAX:180, BASE:150");
  Serial.println("LINE:   BLACK, WHITE, CENTER:3500");
  Serial.println("SYSTEM: STATUS, STOP, RESETI, RESET, SHOWRAW, HELP");
  Serial.println("=========================");
}

void resetToDefaults() {
  kp = 0.15;
  ki = 0.00;
  kd = 0.00;
  integral_limit = 1000.0;
  maxSpeed = 150;
  baseSpeed = 150;
  followBlackLine = true;
  centerPosition = 3500;
  integral = 0;
  
  Serial.println("=== ALL SETTINGS RESET TO DEFAULTS ===");
  showStatus();
}