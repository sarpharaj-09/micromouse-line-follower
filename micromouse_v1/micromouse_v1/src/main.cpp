#include <Arduino.h>
#include <SparkFun_TB6612.h>  // Sparkfun motor driver
#include <QTRSensors.h>       // QTR sensor library

// ---- Function prototypes (REQUIRED for PlatformIO) ----
void calibration();
void follow_segment();
void maze();
char select_turnL(char found_left, char found_straight, char found_right);
char select_turnR(char found_right, char found_straight, char found_left);
void turn(char dir);
void follow_segment1();
void simplify_path();
void follow_segment2();
void follow_segment3();

// Motor pin definitions
#define AIN1 5
#define BIN1 8
#define AIN2 4
#define BIN2 9

#define PWMA 3
#define PWMB 6

// Controls
#define sw1 10
#define sw2 11
#define sw3 12
#define led 7
int s1, s3, s2;
char dir;
int chr = 1;

// Sensor setup
#define NUM_SENSORS 8
unsigned int sensors1[8];
int thr[8];

// PID
#define MaxSpeed 255
#define BaseSpeed 255
int lastError = 0;
float kp = 0.151;
float kd = 0.8;
int last_pos = 3500;

// Shortest path variables
int num = 0;
char path[100];
int path_length = 0;

// Motor objects
Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, 1);
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, 1);

// QTR object
QTRSensors qtra;

void setup() {
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){ A7, A6, A5, A4, A3, A2, A1, A0 }, NUM_SENSORS);

  pinMode(sw1, INPUT);
  pinMode(sw2, INPUT);
  pinMode(sw3, INPUT);
  pinMode(led, OUTPUT);

  Serial.begin(9600);

  s1 = digitalRead(sw1);
  while (s1 == HIGH) s1 = digitalRead(sw1);

  digitalWrite(led, HIGH);
  delay(800);
  calibration();

  while (1) {
    int s2 = digitalRead(sw2);
    int s3 = digitalRead(sw3);
    if (s2 == LOW) { chr = 1; break; }
    if (s3 == LOW) { chr = 2; break; }
  }

  Serial.println(chr);
  delay(900);
  s1 = digitalRead(sw1);
  while (s1 == HIGH) s1 = digitalRead(sw1);
  delay(800);
}

void loop() {
  forward(motor1, motor2, 40);
  delay(40);
  forward(motor1, motor2, 60);
  delay(40);
  forward(motor1, motor2, 80);
  delay(40);
  maze();
}

void calibration() {
  for (int i = 0; i <= 100; i++) {
    if (i < 25 || i >= 75)
      left(motor1, motor2, 200);
    else
      right(motor1, motor2, 200);

    qtra.calibrate();
    delay(10);
  }

  brake(motor1, motor2);

  for (int i = 0; i < NUM_SENSORS; i++) {
    thr[i] = (qtra.calibrationOn.minimum[i] + qtra.calibrationOn.maximum[i]) / 2;
  }
}

void follow_segment() {
  while (1) {
    int position = qtra.readLineWhite(sensors1);
    int error = 3500 - position;
    int motorSpeed = kp * error + kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;

    rightMotorSpeed = constrain(rightMotorSpeed, 0, MaxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, MaxSpeed);

    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
   
    if ((sensors1[0] < thr[0]) || (sensors1[7] < thr[7])) return; //intersection detected

    bool line_lost = true;                          //checks for dead end
    for (int i = 1; i <= 6; i++)
      if (sensors1[i] < thr[i]) line_lost = false;

    if (line_lost) return;

    delay(5);
  }
}

void maze() {
  while (1) {
    follow_segment();
    digitalWrite(led, HIGH);
    brake(motor1, motor2);
    forward(motor1, motor2, 20);  // to align sensor exactly on line (for left and right turns)
    delay(10);
    brake(motor1, motor2);

    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;

    qtra.readLineWhite(sensors1);
    if (sensors1[0] < thr[0]) found_left = 1;
    if (sensors1[7] < thr[7]) found_right = 1;

    forward(motor1, motor2, 20);  //to check straight path ahead
    delay(40);
    brake(motor1, motor2);
    delay(60);

    qtra.readLineWhite(sensors1);
    for (int i = 1; i <= 6; i++)
      if (sensors1[i] < thr[i]) found_straight = 1;

    bool end = true;                           //checks for maze end (all sensors on white)
    for (int i = 1; i <= 6; i++)
      if (sensors1[i] > thr[i]) end = false;

    if (end) break;

    if (chr == 1)
      dir = select_turnL(found_left, found_straight, found_right);
    else
      dir = select_turnR(found_right, found_straight, found_left);

    turn(dir);

    path[path_length++] = dir;
    simplify_path();
  }

  brake(motor1, motor2);  // After completing maze move little forward to align bot over white surface(end pad)
  forward(motor1, motor2, 50);
  delay(100);
  brake(motor1, motor2);

  digitalWrite(led, HIGH);
  delay(4000);
  digitalWrite(led, LOW);

  s2 = digitalRead(sw2);                   //press sw2 ta start actual run (fastest run)
  while (s2 == HIGH) s2 = digitalRead(sw2);

  delay(800);

  // run fastest path
  forward(motor1, motor2, 60);
  delay(40);
  forward(motor1, motor2, 80);
  delay(40);
  forward(motor1, motor2, 100);
  delay(40);

  while (1) {
    for (int k = 0; k < path_length; k++) {
      follow_segment();
      forward(motor1, motor2, 60);     //move a bit forward to align sensor exactly on line before turn
      delay(60);
      brake(motor1, motor2);
      delay(5);
      turn(path[k]);
    }
    follow_segment();
    brake(motor1, motor2);
    forward(motor1, motor2, 80);  //move forward to align bot under finish pad
    delay(400);
    brake(motor1, motor2);
    digitalWrite(led, HIGH);
    delay(4000);
    digitalWrite(led, LOW);
  }
}

char select_turnL(char found_left, char found_straight, char found_right) {
  if (found_left) return 'L';
  else if (found_straight) return 'S';
  else if (found_right) return 'R';
  else return 'B';
}

char select_turnR(char found_right, char found_straight, char found_left) {
  if (found_right) return 'R';
  else if (found_straight) return 'S';
  else if (found_left) return 'L';
  else return 'B';
}

void turn(char dir) {
  int line_position;

  switch (dir) {
  case 'L':
    left(motor1, motor2, 300);
    qtra.readLineWhite(sensors1);
    while (sensors1[0] > thr[0])
      line_position = qtra.readLineWhite(sensors1);
    qtra.readLineWhite(sensors1);
    left(motor1, motor2, 200);
    while (sensors1[0] < thr[0])
      line_position = qtra.readLineWhite(sensors1);
    follow_segment1();
    brake(motor1, motor2);
    break;

  case 'R':
    right(motor1, motor2, 300);
    qtra.readLineWhite(sensors1);
    while (sensors1[7] > thr[7])
      line_position = qtra.readLineWhite(sensors1);
    qtra.readLineWhite(sensors1);
    right(motor1, motor2, 200);
    while (sensors1[7] < thr[7])
      line_position = qtra.readLineWhite(sensors1);
    follow_segment1();
    brake(motor1, motor2);
    break;

  case 'B':
    left(motor1, motor2, 250);
    qtra.readLineWhite(sensors1);
    while (sensors1[0] > thr[0])
      line_position = qtra.readLineWhite(sensors1);
    qtra.readLineWhite(sensors1);
    left(motor1, motor2, 150);
    while (sensors1[0] < thr[0])
      line_position = qtra.readLineWhite(sensors1);
    follow_segment1();
    brake(motor1, motor2);
    break;
  }
}

void follow_segment1() {
  int Kp = 0.1;
  int Kd = 10;

  for (int j = 0; j < 30; j++) {
    int position = qtra.readLineWhite(sensors1);
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;

    rightMotorSpeed = constrain(rightMotorSpeed, 0, MaxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, MaxSpeed);

    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(1);
  }
}

void simplify_path() {
  if (path_length < 3 || path[path_length - 2] != 'B') return;

  int total_angle = 0;
  for (int m = 1; m <= 3; m++) {
    switch (path[path_length - m]) {
    case 'R': total_angle += 90; break;
    case 'L': total_angle += 270; break;
    case 'B': total_angle += 180; break;
    }
  }

  total_angle %= 360;

  switch (total_angle) {
  case 0: path[path_length - 3] = 'S'; break;
  case 90: path[path_length - 3] = 'R'; break;
  case 180: path[path_length - 3] = 'B'; break;
  case 270: path[path_length - 3] = 'L'; break;
  }

  path_length -= 2;
}

void follow_segment2() {
  int Kp = 1;
  int Kd = 200;
  int baseSpeed = 100;
  int maxSpeed = 100;

  for (int j = 0; j < 100; j++) {
    int position = qtra.readLineWhite(sensors1);
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = baseSpeed + motorSpeed;
    int leftMotorSpeed = baseSpeed - motorSpeed;

    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);

    motor1.drive(-rightMotorSpeed);
    motor2.drive(-leftMotorSpeed);
    delay(1);
  }
}

void follow_segment3() {
  int Kp = 1;
  int Kd = 50;

  for (int j = 0; j < 10; j++) {
    int position = qtra.readLineWhite(sensors1);
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;

    rightMotorSpeed = constrain(rightMotorSpeed, 0, MaxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, MaxSpeed);

    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(1);
  }
}
