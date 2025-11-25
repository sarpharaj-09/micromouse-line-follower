// Download below two library on your arduino IDE you will get this from google
#include <SparkFun_TB6612.h>  // This libaray is for sparkfun motor driver  
#include <QTRSensors.h>       // It is for qtr sensors 

// Initialization of the motors
#define AIN1 8
#define BIN1 5
#define AIN2 9
#define BIN2 4
#define PWMA 6
#define PWMB 3
#define STBY 10

const int offsetA = 1;
const int offsetB = 1;

// Initialization of the controls
#define sw1 10    // START switch
#define sw2 11    // CALIBRATE switch
#define led 7
int s1;
int s2;

// Initialization of sensors
#define NUM_SENSORS 8
unsigned int sensors1[8];
int thr[8];

// Initialization of PID parameter
#define MaxSpeed 250
#define BaseSpeed 250
int lastError = 0;
float kp = 0.1;
float kd = 0.3;
int last_pos = 3500;

// Creating the instance of class for motor and sensors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
QTRSensors qtra;

void setup()
{
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4, A5, A6, A7
  }, NUM_SENSORS);

  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  pinMode(13, OUTPUT);

  digitalWrite(STBY, HIGH);
  digitalWrite(13, LOW);

  Serial.begin(9600);

  // ---------- WAIT FOR SW2 TO CALIBRATE ----------
  Serial.println("Press SW2 to CALIBRATE.");
  while (digitalRead(sw2) == HIGH);  // Wait for press  
  calibration();                     // Run calibration
  delay(500);
  Serial.println("Calibration complete.");
}

void loop() {

  // ---------- WAIT FOR SW1 TO START ----------
  Serial.println("Press SW1 to START following line.");
  while (digitalRead(sw1) == HIGH);  // Wait for press

  // Soft Start
  forward(motor1, motor2, 60);  delay(40);
  forward(motor1, motor2, 80);  delay(40);
  forward(motor1, motor2, 100); delay(40);

  follow_segment1();
}



// ---------------------------------------------------------------
//                    CALIBRATION FUNCTION
// ---------------------------------------------------------------
void calibration()
{
  for (int i = 0; i <= 100; i++)
  {
    if (i < 25 || i >= 75)
      left(motor1, motor2, 255);
    else
      right(motor1, motor2, 255);

    qtra.calibrate();
    delay(10);
  }

  brake(motor1, motor2);

  Serial.println("Minimum values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtra.calibrationOn.minimum[i]);
    Serial.print(' ');
    thr[i] = (qtra.calibrationOn.minimum[i] + qtra.calibrationOn.maximum[i]) / 2;
  }

  Serial.println("\nMaximum values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtra.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  Serial.println("\nThresholds:");
  Serial.println(thr[0]);
  Serial.println(thr[7]);
}



// ---------------------------------------------------------------
//                 LINE FOLLOWING FUNCTION
// ---------------------------------------------------------------
void follow_segment1()
{
  while (1) {   // Keep following line

    int position = qtra.readLineWhite(sensors1);
    int error = 3500 - position;

    int motorSpeed = kp * error + kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed  = BaseSpeed + motorSpeed;

    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed  > MaxSpeed ) leftMotorSpeed  = MaxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed  < 0) leftMotorSpeed  = 0;

    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);

    delay(1);
  }
}
