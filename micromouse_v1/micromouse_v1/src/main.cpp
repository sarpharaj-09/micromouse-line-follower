#include <SparkFun_TB6612.h>  // This library is for SparkFun motor driver  
#include <QTRSensors.h>       // It is for QTR sensors 

// Motor pin definitions
#define AIN1 4
#define BIN1 8
#define AIN2 5
#define BIN2 9
#define PWMA 3
#define PWMB 6
// Control switches and LED
#define sw1 10
#define sw2 11
#define sw3 12 
#define led 7

int s1, s2, s3;
char dir;
int chr = 1;

// Sensor configuration
#define NUM_SENSORS 8
uint16_t sensors1[NUM_SENSORS];
uint16_t thr[NUM_SENSORS];

// PID parameters
#define MaxSpeed 130
#define BaseSpeed 130
int lastError = 0;
float kp = 0.161;    // Proportional gain - tune based on your bot
float kd = 0.8;      // Derivative gain - tune based on your bot
// Shortest path parameters
char path[100];
int path_length = 0;

// Creating motor and sensor instances
Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, 1); 
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, 1);
QTRSensors qtra;

void calibration();
void follow_segment();
void maze();
void turn(char dir);
void simplify_path();
char select_turnL(char found_left, char found_straight, char found_right);
char select_turnR(char found_right, char found_straight, char found_left);
void follow_segment1();
void follow_segment2();
void follow_segment3();

void setup()
{
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {   // QTR8A sensor setup
    A7, A6, A5, A4, A3, A2, A1, A0
  }, NUM_SENSORS);

  pinMode(sw1, INPUT);
  pinMode(sw2, INPUT);
  pinMode(sw3, INPUT);
  pinMode(led, OUTPUT);     // Control pin setup

  Serial.begin(9600);
  // while (!Serial)
  // {
  //   ;                           //Serial communication setup for bluetooth
  // }
  s1 = digitalRead(sw1);
  while (s1 == HIGH)
  {
    s1 = digitalRead(sw1);      // Calibration phase - bot gets calibrated after pressing sw1
  }
  digitalWrite(led, HIGH);
  delay(800);
  calibration();
  s2 = digitalRead(sw2);
  while (s2 == HIGH)
  {
    s2 = digitalRead(sw2);      // Press sw2 to start the bot and follow the path
  }
  delay(800);
}

void loop() {
  forward(motor1, motor2, 30);
  delay(40);
  forward(motor1, motor2, 40);
  delay(40);                              // Gradually increase speed to avoid sudden acceleration
  forward(motor1, motor2, 60);
  delay(40);
  maze();
}

void calibration()
{
  // Calibration cycle - swing left and right to read sensor values
  for (int i = 0; i <= 100; i++)
  {
    if (i < 25 || i >= 75)
    {
      left(motor1, motor2, 170);   // Left turn
    }
    else
    {
      right(motor1, motor2, 170);  // Right turn
    }
    qtra.calibrate();
    delay(10);
  }
  
  brake(motor1, motor2);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibrationOn.minimum[i]);
    Serial.print(' ');
    thr[i] = (qtra.calibrationOn.minimum[i] + qtra.calibrationOn.maximum[i]) / 2;
    // Calculating the threshold value for making the decision above thr black line and below white line
  }

  Serial.println();
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print("Threshold[0]: ");
  Serial.println(thr[0]);
  Serial.print("Threshold[7]: ");
  Serial.println(thr[7]);
}
void follow_segment()
{
  while (1)
  {
    digitalWrite(led, LOW);
    // For BLACK line, use readLineBlack() instead of readLineWhite()
    int position = qtra.readLineWhite(sensors1);   // Get current position on line
    int error = 3500 - position;  // 3500 is center position
    int motorSpeed = kp * error + kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    
    if ((sensors1[0] < thr[0]) || (sensors1[7] < thr[7]))
    {
      return;
      // Found an intersection.
    }
    else if (sensors1[1] > thr[1] && sensors1[2] > thr[2] && sensors1[3] > thr[3] && sensors1[4] > thr[4] && sensors1[5] > thr[5] && sensors1[6] > thr[6])
    {
      // There is no line visible ahead, and we didn't see any
      // intersection. Must be a dead end.
      return;
    }
    delay(5);
  }
}
void maze()
{
  while (1)
  {
    follow_segment(); // Follow path until an intersection is detected
    digitalWrite(led, HIGH);
    brake(motor1,motor2);
    forward(motor1, motor2, 50);
    delay(150);
    brake(motor1, motor2);
    // These variables record whether the robot detected a line to the
    // left, straight, or right while examining the current intersection
    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;
    
    // Read sensors and check intersection type
    qtra.readLineWhite(sensors1);
    
    if (sensors1[0] < thr[0])
    {
      found_left = 1;
    }
    if (sensors1[7] < thr[7])
    {
      found_right = 1;
    }
    // Drive straight a bit more - this is enough to line up our
    //wheels with the intersection.
    forward(motor1, motor2, 50);
    delay(50);
    brake(motor1, motor2);
    delay(100);

    qtra.readLineWhite(sensors1);
    if (sensors1[1] < thr[1] || sensors1[2] < thr[2] || sensors1[3] < thr[3] || sensors1[4] < thr[4] || sensors1[5] < thr[5] || sensors1[6] < thr[6] )
    {
      found_straight = 1;
    }
    // Check for the ending spot.
    if (sensors1[1] < thr[1] && sensors1[2] < thr[2] && sensors1[3] < thr[3] && 
        sensors1[4] < thr[4] && sensors1[5] < thr[5] && sensors1[6] < thr[6])
      break;  // End point detected - all middle sensors on black
      
    // Decide which turn to take based on the selected rule
    if (chr == 1)
      dir = select_turnL(found_left, found_straight, found_right);
    else if (chr == 2)
      dir = select_turnR(found_right, found_straight, found_left);
    // Serial.println(dir);
    // Take a turn according to that
    turn(dir);
    // Store the intersection in the path variable.
    path[path_length] = dir;
    path_length++;
    // Simplify the path for the final run
    simplify_path();
  }
  // Second run: execute the optimized shortest path
  brake(motor1, motor2);
  forward(motor1, motor2, 80);
  delay(400);                   // Move straight at end point and turn on LED
  brake(motor1, motor2);
  for (int w = 0; w < path_length; w++)
  {
    Serial.print(path[w]);
    Serial.print(' ');
  }
  digitalWrite(led, HIGH);
  delay(4000);
  digitalWrite(led, LOW);
  s2 = digitalRead(sw2);
  while (s2 == HIGH)
  {
    s2 = digitalRead(sw2);     // Wait for pressing a switch
  }
  delay(800);
  forward(motor1, motor2, 60);
  delay(40);
  forward(motor1, motor2, 80);
  delay(40);
  forward(motor1, motor2, 100);
  delay(40);
  while (1)
  {
    int k;
    for (k = 0; k < path_length; k++)
    {
      follow_segment();
      forward(motor1, motor2, 50);    // After reaching a intercetion follow the shortest path turn
      delay(50);
      forward(motor1, motor2, 60);
      delay(200);
      brake(motor1, motor2);
      delay(5);
      turn(path[k]);
    }
    follow_segment();
    brake(motor1, motor2);
    forward(motor1, motor2, 80);
    delay(400);
    brake(motor1, motor2);
    digitalWrite(led, HIGH);
    delay(4000);
    digitalWrite(led, LOW);
  }
  // Go back to a starting of the main loop
}
char select_turnL(char found_left, char found_straight, char found_right)
{
  // Make a decision about how to turn. The following code
  // implements a left-hand-on-the-wall strategy, where we always prefer left turns
  if (found_left)
    return 'L';
  else if (found_straight)
    return 'S';
  else if (found_right)
    return 'R';
  else
    return 'B';
}
char select_turnR(char found_right, char found_straight, char found_left)
{
  // Make a decision about how to turn. The following code
  // implements a right-hand-on-the-wall strategy, where we always prefer right turns
  if (found_right)
    return 'R';
  else if (found_straight)
    return 'S';
  else if (found_left)
    return 'L';
  else
    return 'B';
}
void turn(char dir)
{
  // Perform turn based on direction parameter
  switch (dir)
  {
    case 'L':
      left(motor1, motor2, 200);
      qtra.readLineWhite(sensors1);
      while (sensors1[0] < thr[0])
      {
        qtra.readLineWhite(sensors1);  // Move left until sensor goes from white to black
      }
      qtra.readLineWhite(sensors1);
      left(motor1, motor2, 140);
      while (sensors1[0] > thr[0])
      {
        qtra.readLineWhite(sensors1);  // Continue left until back on white line
      }
      follow_segment1();  // Fast PID to quickly align bot on line
      brake(motor1, motor2);
      break;
      
    case 'R':
      right(motor1, motor2, 200);
      qtra.readLineWhite(sensors1);
      while (sensors1[7] < thr[7])
      {
        qtra.readLineWhite(sensors1);  // Move right until sensor goes from white to black
      }
      qtra.readLineWhite(sensors1);
      right(motor1, motor2, 140);
      while (sensors1[7] > thr[7])
      {
        qtra.readLineWhite(sensors1);  // Continue right until back on white line
      }
      follow_segment1();  // Fast PID to quickly align bot on line
      brake(motor1, motor2);
      break;
      
    case 'B':
      right(motor1, motor2, 150);
      qtra.readLineWhite(sensors1);
      while (sensors1[7] > thr[7])
      {
        qtra.readLineWhite(sensors1);  // U-turn using right turn
      }
      qtra.readLineWhite(sensors1);
      right(motor1, motor2, 120);
      while (sensors1[7] < thr[7])
      {
        qtra.readLineWhite(sensors1);
      }
      break;
  }
}
void follow_segment1()
{
  // Fast PID after turn to quickly align bot on line
  float Kp = 0.2;
  float Kd = 10;
  for (int j = 0; j < 10; j++)
  {
    int position = qtra.readLineWhite(sensors1);  // For BLACK line, use readLineBlack()
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(1);
  }
}
void simplify_path()
{
  // Path simplification. The strategy is that whenever we encounter a
  // sequence xBx, we can simplify it by cutting out the dead end.

  if (path_length < 3 || path[path_length - 2] != 'B')      // simplify the path only if the second-to-last turn was a 'B'
    return;
  int total_angle = 0;
  int m;
  // Get the angle as a number between 0 and 360 degrees.
  for (m = 1; m <= 3; m++)
  {
    switch (path[path_length - m])
    {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }
  // Replace all of those turns with a single one.
  total_angle = total_angle % 360;
  switch (total_angle)
  {
    case 0:
      path[path_length - 3] = 'S';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }
  path_length -= 2;
}
void follow_segment2()
{
  // Backward moving PID
  int Kp = 1;
  int Kd = 200;
  int baseSpeed = 100;
  int maxSpeed = 100;
  for (int j = 0; j < 100; j++)
  {
    int position = qtra.readLineWhite(sensors1);  // For BLACK line, use readLineBlack()
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = baseSpeed + motorSpeed;
    int leftMotorSpeed = baseSpeed - motorSpeed;
    if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;
    motor1.drive(-rightMotorSpeed);
    motor2.drive(-leftMotorSpeed);
    delay(1);
  }
}

void follow_segment3()
{
  int Kp = 1;
  int Kd = 50;
  for (int j = 0; j < 10; j++)
  {
    int position = qtra.readLineWhite(sensors1);  // For BLACK line, use readLineBlack()
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(1);
  }
}