#include <SparkFun_TB6612.h>  // This libaray is for sparkfun motor driver  
#include <QTRSensors.h>       // It is for qtr sensors 

// Initialization of the motors
#define AIN1 5 //4
#define BIN1 8 //8
#define AIN2 4 //5
#define BIN2 9 //9

#define PWMA 3
#define PWMB 6

// Initialization of the controls
#define sw1 10  
#define sw2 11
#define sw3 12 
#define led 7
int s1,s3,s2;
char dir;
int chr = 1;

// Initialization of sensors
#define NUM_SENSORS 8
unsigned int sensors1[8];
int thr[8];

// Initialization of PID parameter
#define MaxSpeed 180 //170
#define BaseSpeed 150 //170
int lastError = 0;
float kp = 0.012;    // It fully depends on the bot system
float kd = 0;    // Please follow the method provided in instructables to get your values
int last_pos = 3500;

// shortheschalneyyyt path parameters
int num = 0;
char path[100];
int path_length = 0;


// Creating the instance of class for motor and senors
Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, 1); 
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, 1); // right
QTRSensors qtra;

void setup()
{
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {   // QTR8A sensor setup
    A0, A1, A2, A3, A4, A5, A6, A7
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
    s1 = digitalRead(sw1);      // Calibration phase where the bot get calibrated after pressing s1 switch
  }
  digitalWrite(led,HIGH);
  delay(800);
  calibration();
  while (1)
  {
    int s2 = digitalRead(sw2);
    int s3 = digitalRead(sw3);
    if (s2 == LOW)
    {
      chr = 1;
      break;                        // Here we have to choose the Rule which will follow by the bot
                                    // S1 switch for LHS and s2 switch for RHS
                                    // In LHS left, straight, right will be priority
    }                               // In RHS right, straight, left will be priority
    if (s3 == LOW)                  
    {
      chr = 2;   
      break;
    }
  }
  Serial.println(chr);
  delay(900);
  s1 = digitalRead(sw1);
  while (s1 == HIGH)
  {
    s1 = digitalRead(sw1);      // Press s2 switch to start a bot to follow the path
  }
  delay(800);
}
void loop() {
  forward(motor1, motor2, 40);
  delay(40);
  forward(motor1, motor2, 60);
  delay(40);                              // For graduly increasing speed so that bot dose not get direct high speed at the start
  forward(motor1, motor2, 80);
  delay(40);
  maze();
}
void calibration()
{
  for (int i = 0; i <= 100; i++)
  {

    if (i < 25 || i >= 75)
    {
      left(motor1, motor2, 150); //Left turn
    }
    else
    {

      right(motor1, motor2, 150); //Right turn
    }
    qtra.calibrate();
    delay(10);
  }  // end calibration cycle
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
  Serial.println("\nThreshold Values:");
for (int i = 0; i < NUM_SENSORS; i++)
{
  Serial.print("Sensor ");
  Serial.print(i);
  Serial.print(": ");
  Serial.println(thr[i]);
}
}
void follow_segment()
{
  while (1)
  {
    digitalWrite(led, LOW);
    // FOR BLACK LINE FOLLOWER JUST REPLACE White WITH Black
    int position = qtra.readLineWhite(sensors1);   // Getting the present position of the bot
    int error =  3500 - position;
    int motorSpeed = kp * error + kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0)rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    if ((sensors1[0] < thr[0]) || (sensors1[7] < thr[7]))
    {
      return ;
      // Found an intersection.
    }
    else if (sensors1[1] > thr[1] && sensors1[2] > thr[2] && sensors1[3] > thr[3] && sensors1[4] > thr[4] && sensors1[5] > thr[5] && sensors1[6] > thr[6])
    {
      // There is no line visible ahead, and we didn't see any
      // intersection. Must be a dead end.
      return ;
    }
    delay(5);
  }
}
void maze()
{
  while (1)
  {
    // First main loop body were a goal is to store all the dead end or we say dry run path
    //After reaching at end we use break to get out of this
    follow_segment(); // It will follow the path until it dosn't get any intersection
    // brake(motor1, motor2);
    // back(motor1,motor2,100);
    // delay(1000);
    // brake(motor1,motor2);
    // delay(200);
    digitalWrite(led, HIGH);
    brake(motor1, motor2);           // After getting intersetion fist thing that we have to do
                                     // Drive straight a bit. This helps us in case we entered the
                                     // intersection at an angle
    forward(motor1, motor2, 40); //40
    delay(10); //30
    brake(motor1, motor2);
    // These variables record whether the robot has seen a line to the
    // straight ahead, and right, whil examining the current
    // intersection.
    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;
    // Now read the sensors and check the intersection type.
    qtra.readLineWhite(sensors1);
    // if (sensors1[7] < thr[7])
    // {
    //   found_left = 1;
    //   if (sensors1[1] < thr[1])
    //   {
    //     found_right = 1;
    //   }
    // }
    // if (sensors1[0] < thr[0])
    // {
    //   found_right = 1;
    //   if (sensors1[6] < thr[6])
    //   {
    //     found_left = 1;

    //   }
    // }
    if(sensors1[0] < thr[0]){
    found_left = 1;
    }
    if(sensors1[7] < thr[7]){

    found_right = 1;
    }
    // Drive straight a bit more - this is enough to line up our
    //wheels with the intersection.
    forward(motor1, motor2, 20);
    delay(40); //60 //100
    brake(motor1, motor2);
    delay(60); //60
    qtra.readLineWhite(sensors1);
    if (sensors1[1] < thr[1] || sensors1[2] < thr[2] || sensors1[3] < thr[3] || sensors1[4] < thr[4] || sensors1[5] < thr[5] || sensors1[6] < thr[6] )
    {
      found_straight = 1;
    }
    // Check for the ending spot.
    if (sensors1[1] < thr[1] && sensors1[2] < thr[2] && sensors1[3] < thr[3] && sensors1[4] < thr[4] && sensors1[5] < thr[5] && sensors1[6] < thr[6])
      break;
    // According to rule we have to decided which turn has to be taken
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
  // Now the second half is the short run which is obtained after sorting the dry run
  brake(motor1, motor2);
  forward(motor1, motor2, 50);
  delay(100);                   // Move straight a bit on end point and glow a led
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
      forward(motor1, motor2, 60);    // After reaching a intercetion follow the shortest path turn
      delay(60);
      // forward(motor1, motor2, 60);
      // delay(100);
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
  // Implements a left-hand-on-the-wall strategy, where we always preffer left turn
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
  // Implements a right-hand-on-the-wall strategy, Where we always preffer right turn
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
  int line_position;
  // According to dir perform a turn
  switch (dir)
  {
    case'L':
      left(motor1, motor2, 300);
      qtra.readLineWhite(sensors1);
      while (sensors1[0] > thr[0])
      {
        line_position = qtra.readLineWhite(sensors1);    //Move left until left extreme sensor goes out of the white line comes on black
      }
      qtra.readLineWhite(sensors1);
      left(motor1, motor2, 200);
      while (sensors1[0] < thr[0])
      {
        line_position = qtra.readLineWhite(sensors1);        // Again move left until it get's on white line
      }
      follow_segment1();    // Fast pid after turn to align bot quickly on line
      brake(motor1, motor2);
      break;
    case'R':
      right(motor1, motor2, 300); //250
      qtra.readLineWhite(sensors1);
      while (sensors1[7] > thr[7])
      {
        line_position = qtra.readLineWhite(sensors1);   //Move right until left extreme sensor goes out of the white line comes on black
      }
      qtra.readLineWhite(sensors1);
      right(motor1, motor2, 200); //150
      while (sensors1[7] < thr[7])
      {
        line_position = qtra.readLineWhite(sensors1);         // Again move right until it get's on white line
      }
      follow_segment1();      // Fast pid after turn to align a bot quickly on line
      brake(motor1, motor2);
      break;
    case'B':
      // right(motor1, motor2, 230);
      // qtra.readLineWhite(sensors1);
      // while (sensors1[7 ] > thr[7])
      // {
      //   line_position = qtra.readLineWhite(sensors1);          // Take u turn using right turn
      // }
      // qtra.readLineWhite(sensors1);
      // right(motor1, motor2, 150);

      // while (sensors1[7] < thr[7])
      // {
      //   line_position = qtra.readLineWhite(sensors1);
      // }
      // follow_segment3();
      // brake(motor1, motor2);
      // delay(50);
      // follow_segment2();    // Back moving pid
      // forward(motor1, motor2, 40);
      // delay(40);
      // break;
      left(motor1, motor2, 250);
      qtra.readLineWhite(sensors1);
      while (sensors1[0] > thr[0])
      {
        line_position = qtra.readLineWhite(sensors1);    //Move left until left extreme sensor goes out of the white line comes on black
      }
      qtra.readLineWhite(sensors1);
      left(motor1, motor2, 150);
      while (sensors1[0] < thr[0])
      {
        line_position = qtra.readLineWhite(sensors1);        // Again move left until it get's on white line
      }
      follow_segment1();    // Fast pid after turn to align bot quickly on line
      brake(motor1, motor2);
      // follow_segment2();    // Back moving pid
      // forward(motor1, motor2, 40);
      // delay(40);
      break;
  }
}
void follow_segment1()
{
  // Fast pid after turn to get quickly align a bot on line
  int Kp = 0.2;
  int Kd =10 ; //40
  for (int j = 0; j < 30; j++) {
    int position = qtra.readLineWhite(sensors1);       // FOR BLACK LINE FOLLOWER JUST REPLACE White WITH Black
    int error =  3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0)rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;
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
  //Back moving pid
  int Kp = 1;
  int Kd = 200; //80
  int baseSpeed = 100;
  int maxSpeed = 100;
  for (int j = 0; j < 100; j++) {
    int position = qtra.readLineWhite(sensors1);       // FOR BLACK LINE FOLLOWER JUST REPLACE White WITH Black
    int error =  3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = baseSpeed + motorSpeed;
    int leftMotorSpeed = baseSpeed - motorSpeed;
    if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed;
    if (rightMotorSpeed < 0)rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    motor1.drive(-rightMotorSpeed);
    motor2.drive(-leftMotorSpeed);
    delay(1);
  }
}

void follow_segment3()
{
  int Kp = 1;
  int Kd =50; //20
  for (int j = 0; j < 10; j++) {
    int position = qtra.readLineWhite(sensors1);       // FOR BLACK LINE FOLLOWER JUST REPLACE White WITH Black
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0)rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(1);
  }
}