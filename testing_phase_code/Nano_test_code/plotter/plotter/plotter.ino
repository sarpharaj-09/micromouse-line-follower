+#include <SparkFun_TB6612.h>
#include <QTRSensors.h>

// Motor pins
#define AIN1 8
#define BIN1 5
#define AIN2 9
#define BIN2 4
#define PWMA 6
#define PWMB 3
#define STBY 10

const int offsetA = 1;
const int offsetB = 1;

// Controls
#define SW_START 10      // SW1 - Start/Stop line following
#define SW_CALIBRATE 11  // SW2 - Calibration
#define LED 7

// Sensors
#define NUM_SENSORS 8
#define SENSOR_TIMEOUT 2500
unsigned int sensors1[NUM_SENSORS];
QTRSensors qtra;

// PID Parameters
float kp = 0.9;
float ki = 0.000001;
float kd = 11;

// Speed settings
int maxSpeed = 180;
int baseSpeed = 180;

// Line tracking
bool followBlackLine = false;
int centerPosition = 3500;

// PID variables
int lastError = 0;
float integral = 0.0;
float integral_limit = 3000.0;

// State management
enum State { IDLE, CALIBRATING, READY, RUNNING };
State currentState = IDLE;

// PLOTTING CONTROL
bool enablePlotting = false;
unsigned long lastPlotTime = 0;
const unsigned long plotInterval = 50;  // Plot every 50ms (20Hz)

String btBuffer = "";

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// ========== SETUP ==========
void setup() {
  btBuffer.reserve(32);
  
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);
  qtra.setTimeout(SENSOR_TIMEOUT);

  pinMode(SW_START, INPUT_PULLUP);
  pinMode(SW_CALIBRATE, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  Serial.begin(9600);
  printWelcome();
  
  currentState = IDLE;
  blinkLED(2, 200);
}

// ========== MAIN LOOP ==========
void loop() {
  checkBluetooth();
  
  static bool lastSW1 = HIGH;
  static bool lastSW2 = HIGH;
  bool sw1 = digitalRead(SW_START);
  bool sw2 = digitalRead(SW_CALIBRATE);
  
  // SW2 - Calibration (pressed)
  if (sw2 == LOW && lastSW2 == HIGH) {
    delay(50);
    if (digitalRead(SW_CALIBRATE) == LOW) {
      handleCalibration();
    }
  }
  
  // SW1 - Start/Stop (pressed)
  if (sw1 == LOW && lastSW1 == HIGH) {
    delay(50);
    if (digitalRead(SW_START) == LOW) {
      handleStartStop();
    }
  }
  
  // Execute current state
  if (currentState == RUNNING) {
    runLineFollowing();
  }
  
  lastSW1 = sw1;
  lastSW2 = sw2;
}

// ========== BUTTON HANDLERS ==========
void handleCalibration() {
  if (currentState == RUNNING) {
    if (!enablePlotting) Serial.println("Stop robot before calibration!");
    blinkLED(3, 100);
    return;
  }
  
  currentState = CALIBRATING;
  calibration();
  currentState = READY;
}

void handleStartStop() {
  if (currentState == IDLE) {
    if (!enablePlotting) Serial.println("Calibrate first (press SW2)!");
    blinkLED(3, 100);
    return;
  }
  
  if (currentState == RUNNING) {
    currentState = READY;
    brake(motor1, motor2);
    integral = 0;
    if (!enablePlotting) Serial.println("STOPPED");
    blinkLED(1, 500);
  } else if (currentState == READY) {
    currentState = RUNNING;
    if (!enablePlotting) Serial.println("STARTING...");
    blinkLED(2, 100);
    forward(motor1, motor2, 80);
    delay(100);
  }
}

// ========== CALIBRATION ==========
void calibration() {
  if (!enablePlotting) {
    Serial.println("=== CALIBRATING ===");
    Serial.println("Move robot left-right over line...");
  }
  digitalWrite(LED, HIGH);
  
  for (int i = 0; i <= 100; i++) {
    if (i < 25 || i >= 75) {
      left(motor1, motor2, 200);
    } else {
      right(motor1, motor2, 200);
    }
    qtra.calibrate();
    delay(10);
  }
  
  brake(motor1, motor2);
  digitalWrite(LED, LOW);
  if (!enablePlotting) {
    Serial.println("Calibration Complete!");
    Serial.println("Press SW1 to start");
  }
  blinkLED(2, 200);
}

// ========== LINE FOLLOWING ==========
void runLineFollowing() {
  checkBluetooth();
  
  // Read sensor position
  int position = followBlackLine ? qtra.readLineBlack(sensors1) : qtra.readLineWhite(sensors1);
  
  // Calculate error
  int error = centerPosition - position;
  
  // Integral term with anti-windup
  integral += error;
  integral = constrain(integral, -integral_limit, integral_limit);
  
  // Derivative term
  int derivative = error - lastError;
  lastError = error;
  
  // PID calculation
  float pTerm = kp * error;
  float iTerm = ki * integral;
  float dTerm = kd * derivative;
  float correction = pTerm + iTerm + dTerm;
  correction = constrain(correction, -1000.0, 1000.0);
  
  // Calculate motor speeds
  int rightSpeed = constrain(baseSpeed - correction, 0, maxSpeed);
  int leftSpeed = constrain(baseSpeed + correction, 0, maxSpeed);
  
  // Drive motors
  motor1.drive(rightSpeed);
  motor2.drive(leftSpeed);
  
  // PLOTTING OUTPUT
  if (enablePlotting) {
    unsigned long currentTime = millis();
    if (currentTime - lastPlotTime >= plotInterval) {
      lastPlotTime = currentTime;
      
      // Format: Label:Value Label:Value ...
      Serial.print("Error:");
      Serial.print(error);
      Serial.print(",P:");
      Serial.print(pTerm);
      Serial.print(",I:");
      Serial.print(iTerm);
      Serial.print(",D:");
      Serial.print(dTerm);
      Serial.print(",Correction:");
      Serial.print(correction);
      Serial.print(",LeftSpeed:");
      Serial.print(leftSpeed);
      Serial.print(",RightSpeed:");
      Serial.print(rightSpeed);
      Serial.print(",Position:");
      Serial.println(position);
    }
  }
}

// ========== BLUETOOTH PROCESSING ==========
void checkBluetooth() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (btBuffer.length() > 0) {
        processCommand(btBuffer);
        btBuffer = "";
      }
    } else if (btBuffer.length() < 31) {
      btBuffer += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  bool validCommand = true;
  
  // PLOTTING COMMANDS
  if (cmd == "PLOT" || cmd == "PLOT:ON") {
    enablePlotting = true;
    Serial.println("Plotting:ENABLED");
    delay(100);
  }
  else if (cmd == "PLOT:OFF") {
    enablePlotting = false;
    Serial.println("Plotting:DISABLED");
  }
  else if (cmd.startsWith("PLOTRATE:")) {
    int newRate = cmd.substring(9).toInt();
    if (newRate >= 10 && newRate <= 1000) {
      const_cast<unsigned long&>(plotInterval) = newRate;
      if (!enablePlotting) {
        Serial.print("PlotRate=");
        Serial.print(newRate);
        Serial.println("ms");
      }
    }
  }
  
  // PID parameters
  else if (cmd.startsWith("KP:")) {
    kp = cmd.substring(3).toFloat();
    if (!enablePlotting) {
      Serial.print("P="); Serial.println(kp, 6);
    }
  }
  else if (cmd.startsWith("KI:")) {
    ki = cmd.substring(3).toFloat();
    if (!enablePlotting) {
      Serial.print("I="); Serial.println(ki, 6);
    }
  }
  else if (cmd.startsWith("KD:")) {
    kd = cmd.substring(3).toFloat();
    if (!enablePlotting) {
      Serial.print("D="); Serial.println(kd, 6);
    }
  }
  else if (cmd.startsWith("ILIMIT:")) {
    integral_limit = cmd.substring(7).toFloat();
    if (!enablePlotting) {
      Serial.print("I_Limit="); Serial.println(integral_limit, 2);
    }
  }
  
  // Speed settings
  else if (cmd.startsWith("MAX:")) {
    maxSpeed = constrain(cmd.substring(4).toInt(), 0, 255);
    if (!enablePlotting) {
      Serial.print("MaxSpeed="); Serial.println(maxSpeed);
    }
  }
  else if (cmd.startsWith("BASE:")) {
    baseSpeed = constrain(cmd.substring(5).toInt(), 0, 255);
    if (!enablePlotting) {
      Serial.print("BaseSpeed="); Serial.println(baseSpeed);
    }
  }
  
  // Line type
  else if (cmd == "BLACK") {
    followBlackLine = true;
    if (!enablePlotting) Serial.println("Line=BLACK");
  }
  else if (cmd == "WHITE") {
    followBlackLine = false;
    if (!enablePlotting) Serial.println("Line=WHITE");
  }
  else if (cmd.startsWith("CENTER:")) {
    centerPosition = cmd.substring(7).toInt();
    if (!enablePlotting) {
      Serial.print("Center="); Serial.println(centerPosition);
    }
  }
  
  // System commands
  else if (cmd == "STATUS" || cmd == "S") {
    bool wasPlotting = enablePlotting;
    enablePlotting = false;
    showStatus();
    enablePlotting = wasPlotting;
  }
  else if (cmd == "HELP" || cmd == "H") {
    bool wasPlotting = enablePlotting;
    enablePlotting = false;
    showHelp();
    enablePlotting = wasPlotting;
  }
  else if (cmd == "STOP") {
    currentState = READY;
    brake(motor1, motor2);
    integral = 0;
    if (!enablePlotting) Serial.println("STOPPED");
  }
  else if (cmd == "START") {
    if (currentState == READY) {
      currentState = RUNNING;
      if (!enablePlotting) Serial.println("STARTED");
    } else {
      if (!enablePlotting) Serial.println("Not ready! Calibrate first");
    }
  }
  else if (cmd == "RESETI") {
    integral = 0;
    if (!enablePlotting) Serial.println("Integral=0");
  }
  else if (cmd == "RESET") {
    bool wasPlotting = enablePlotting;
    enablePlotting = false;
    resetToDefaults();
    enablePlotting = wasPlotting;
  }
  else if (cmd == "SENSORS") {
    bool wasPlotting = enablePlotting;
    enablePlotting = false;
    showSensors();
    enablePlotting = wasPlotting;
  }
  else {
    validCommand = false;
    if (!enablePlotting) Serial.println("Unknown command. Type HELP");
  }
  
  // Visual feedback
  if (validCommand) {
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
  }
}

// ========== DISPLAY FUNCTIONS ==========
void printWelcome() {
  Serial.println("\n=== LINE FOLLOWER READY ===");
  Serial.println("SW1: Start/Stop");
  Serial.println("SW2: Calibrate");
  Serial.println("Commands: HELP, STATUS, PLOT");
  Serial.println("===========================\n");
}

void showStatus() {
  Serial.println("\n=== STATUS ===");
  Serial.print("State: ");
  switch(currentState) {
    case IDLE: Serial.println("IDLE"); break;
    case CALIBRATING: Serial.println("CALIBRATING"); break;
    case READY: Serial.println("READY"); break;
    case RUNNING: Serial.println("RUNNING"); break;
  }
  
  Serial.print("Plotting: ");
  Serial.println(enablePlotting ? "ENABLED" : "DISABLED");
  
  Serial.print("PID: P="); Serial.print(kp, 4);
  Serial.print(" I="); Serial.print(ki, 6);
  Serial.print(" D="); Serial.println(kd, 4);
  
  Serial.print("Speed: Base="); Serial.print(baseSpeed);
  Serial.print(" Max="); Serial.println(maxSpeed);
  
  Serial.print("Line: "); Serial.print(followBlackLine ? "BLACK" : "WHITE");
  Serial.print(" | Center="); Serial.println(centerPosition);
  
  Serial.print("Integral: "); Serial.print(integral, 2);
  Serial.print(" | Limit: "); Serial.println(integral_limit, 2);
  Serial.println("==============\n");
}

void showHelp() {
  Serial.println("\n=== COMMANDS ===");
  Serial.println("PLOT:   PLOT (enable) | PLOT:OFF | PLOTRATE:50");
  Serial.println("PID:    KP:0.15  KI:0.001  KD:5.0  ILIMIT:1000");
  Serial.println("SPEED:  BASE:150  MAX:200");
  Serial.println("LINE:   BLACK | WHITE | CENTER:3500");
  Serial.println("SYSTEM: STATUS (S) | HELP (H) | START | STOP");
  Serial.println("        RESETI | RESET | SENSORS");
  Serial.println("================\n");
}

void showSensors() {
  int position = followBlackLine ? qtra.readLineBlack(sensors1) : qtra.readLineWhite(sensors1);
  
  Serial.print("Pos="); Serial.print(position);
  Serial.print(" | Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensors1[i]);
    if (i < NUM_SENSORS - 1) Serial.print(",");
  }
  Serial.println();
}

void resetToDefaults() {
  kp = 0.15;
  ki = 0.0;
  kd = 0.0;
  integral_limit = 1000.0;
  maxSpeed = 150;
  baseSpeed = 150;
  followBlackLine = true;
  centerPosition = 3500;
  integral = 0;
  
  Serial.println("\n=== RESET TO DEFAULTS ===");
  showStatus();
}

// ========== UTILITY FUNCTIONS ==========
void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, HIGH);
    delay(delayMs);
    digitalWrite(LED, LOW);
    if (i < times - 1) delay(delayMs);
  }
}