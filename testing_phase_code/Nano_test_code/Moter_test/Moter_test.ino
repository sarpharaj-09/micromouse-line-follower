#include <SparkFun_TB6612.h>

// Motor pin definitions
#define AIN1 5
#define BIN1 8
#define AIN2 4
#define BIN2 9
#define PWMA 3
#define PWMB 6

// Control pins
#define sw1 10
#define led 7

// Creating motor instances
Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, 1); // Left motor
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, 1); // Right motor

int testSpeed = 150; // Default speed

void setup() {
  pinMode(sw1, INPUT);
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  
  delay(2000); // Give time for Serial Monitor to open
  
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Clear any remaining characters in buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
    
    executeCommand(command);
  }
}

void printMenu() {
  Serial.println("\n========================================");
  Serial.println("       MOTOR TEST MENU");
  Serial.println("========================================");
  Serial.println("F - Forward");
  Serial.println("B - Backward");
  Serial.println("L - Left Turn");
  Serial.println("R - Right Turn");
  Serial.println("S - Stop/Brake");
  Serial.println("1 - Test Left Motor Only");
  Serial.println("2 - Test Right Motor Only");
  Serial.println("3 - Test Both Motors (Individual Control)");
  Serial.println("+ - Increase Speed (+25)");
  Serial.println("- - Decrease Speed (-25)");
  Serial.println("V - View Current Speed");
  Serial.println("C - Continuous Forward (Press S to stop)");
  Serial.println("T - Run Full Test Sequence");
  Serial.println("M - Show this Menu");
  Serial.println("========================================");
  Serial.print("Current Speed: ");
  Serial.println(testSpeed);
  Serial.println("\nEnter command: ");
}

void executeCommand(char cmd) {
  switch (cmd) {
    case 'F':
    case 'f':
      Serial.println(">> Forward");
      digitalWrite(led, HIGH);
      forward(motor1, motor2, testSpeed);
      delay(2000);
      brake(motor1, motor2);
      digitalWrite(led, LOW);
      Serial.println("   Stopped");
      break;
      
    case 'B':
    case 'b':
      Serial.println(">> Backward");
      digitalWrite(led, HIGH);
      back(motor1, motor2, testSpeed);
      delay(2000);
      brake(motor1, motor2);
      digitalWrite(led, LOW);
      Serial.println("   Stopped");
      break;
      
    case 'L':
    case 'l':
      Serial.println(">> Left Turn");
      digitalWrite(led, HIGH);
      left(motor1, motor2, testSpeed);
      delay(1500);
      brake(motor1, motor2);
      digitalWrite(led, LOW);
      Serial.println("   Stopped");
      break;
      
    case 'R':
    case 'r':
      Serial.println(">> Right Turn");
      digitalWrite(led, HIGH);
      right(motor1, motor2, testSpeed);
      delay(1500);
      brake(motor1, motor2);
      digitalWrite(led, LOW);
      Serial.println("   Stopped");
      break;
      
    case 'S':
    case 's':
      Serial.println(">> Stop/Brake");
      brake(motor1, motor2);
      digitalWrite(led, LOW);
      Serial.println("   Motors stopped");
      break;
      
    case '1':
      Serial.println(">> Testing Left Motor Only");
      digitalWrite(led, HIGH);
      motor1.drive(testSpeed);
      motor2.drive(0);
      delay(2000);
      brake(motor1, motor2);
      digitalWrite(led, LOW);
      Serial.println("   Stopped");
      break;
      
    case '2':
      Serial.println(">> Testing Right Motor Only");
      digitalWrite(led, HIGH);
      motor1.drive(0);
      motor2.drive(testSpeed);
      delay(2000);
      brake(motor1, motor2);
      digitalWrite(led, LOW);
      Serial.println("   Stopped");
      break;
      
    case '3':
      Serial.println(">> Testing Both Motors Individually");
      digitalWrite(led, HIGH);
      Serial.println("   Left motor...");
      motor1.drive(testSpeed);
      motor2.drive(0);
      delay(1500);
      brake(motor1, motor2);
      delay(500);
      Serial.println("   Right motor...");
      motor1.drive(0);
      motor2.drive(testSpeed);
      delay(1500);
      brake(motor1, motor2);
      digitalWrite(led, LOW);
      Serial.println("   Stopped");
      break;
      
    case '+':
      testSpeed += 25;
      if (testSpeed > 255) testSpeed = 255;
      Serial.print(">> Speed increased to: ");
      Serial.println(testSpeed);
      break;
      
    case '-':
      testSpeed -= 25;
      if (testSpeed < 0) testSpeed = 0;
      Serial.print(">> Speed decreased to: ");
      Serial.println(testSpeed);
      break;
      
    case 'V':
    case 'v':
      Serial.print(">> Current Speed: ");
      Serial.println(testSpeed);
      break;
      
    case 'C':
    case 'c':
      Serial.println(">> Continuous Forward (Press 'S' to stop)");
      digitalWrite(led, HIGH);
      forward(motor1, motor2, testSpeed);
      Serial.println("   Running... Press 'S' to stop");
      break;
      
    case 'T':
    case 't':
      runFullTest();
      break;
      
    case 'M':
    case 'm':
      printMenu();
      break;
      
    case '\n':
    case '\r':
      // Ignore newline characters
      break;
      
    default:
      Serial.println(">> Invalid command! Press 'M' for menu");
      break;
  }
}

void runFullTest() {
  Serial.println("\n>> Running Full Test Sequence");
  Serial.println("========================================");
  
  // Test 1: Forward
  Serial.println("Test 1: Forward");
  digitalWrite(led, HIGH);
  forward(motor1, motor2, 100);
  delay(2000);
  brake(motor1, motor2);
  digitalWrite(led, LOW);
  delay(1000);
  
  // Test 2: Backward
  Serial.println("Test 2: Backward");
  digitalWrite(led, HIGH);
  back(motor1, motor2, 100);
  delay(2000);
  brake(motor1, motor2);
  digitalWrite(led, LOW);
  delay(1000);
  
  // Test 3: Left Turn
  Serial.println("Test 3: Left Turn");
  digitalWrite(led, HIGH);
  left(motor1, motor2, 150);
  delay(1500);
  brake(motor1, motor2);
  digitalWrite(led, LOW);
  delay(1000);
  
  // Test 4: Right Turn
  Serial.println("Test 4: Right Turn");
  digitalWrite(led, HIGH);
  right(motor1, motor2, 150);
  delay(1500);
  brake(motor1, motor2);
  digitalWrite(led, LOW);
  delay(1000);
  
  // Test 5: Speed Ramp
  Serial.println("Test 5: Speed Ramp (50-200)");
  digitalWrite(led, HIGH);
  for (int speed = 50; speed <= 200; speed += 50) {
    Serial.print("  Speed: ");
    Serial.println(speed);
    forward(motor1, motor2, speed);
    delay(1000);
  }
  brake(motor1, motor2);
  digitalWrite(led, LOW);
  delay(1000);
  
  // Test 6: Individual Motors
  Serial.println("Test 6: Individual Motors");
  digitalWrite(led, HIGH);
  Serial.println("  Left motor");
  motor1.drive(150);
  motor2.drive(0);
  delay(1500);
  brake(motor1, motor2);
  delay(500);
  Serial.println("  Right motor");
  motor1.drive(0);
  motor2.drive(150);
  delay(1500);
  brake(motor1, motor2);
  digitalWrite(led, LOW);
  
  Serial.println("========================================");
  Serial.println(">> Full Test Complete!");
  
  // Blink LED 3 times
  for (int i = 0; i < 3; i++) {
    digitalWrite(led, HIGH);
    delay(300);
    digitalWrite(led, LOW);
    delay(300);
  }
  
  Serial.println("\nPress 'M' for menu");
}