#include "NewPing.h"
#include "Servo.h"

// black -> red -> green

int leftMotorForwardPin = 7;
int leftMotorBackwardPin = 6;
int rightMotorForwardPin = 11;
int rightMotorBackwardPin = 10;

int ENA = 3; // enable right motor (A)
int ENB = 5; // enable left motor (B)

// for ultrasonic
int triggerPin = 1; // change
int echoPin = 0; // change
int MAX_DISTANCE = 200;

long duration;
long cm;

// colour sensor
int s0 = A0;
int s1 = A1;
int s2 = A3;
int s3 = A4;
int out = A2;
int data = 0;

int redValue;
int greenValue;
int blueValue;

int storePathColour = 0; // current path is green

// ===== PID VARIABLES =====
float Kp = 1.2;    // start here
float Ki = 0.0;
float Kd = 0.4;

float error = 0;
float lastError = 0;
float integral = 0;

int baseSpeed = 140;
int maxSpeed  = 255;

int rightSpeed = 0;
int leftSpeed = 0;

// CHANGE THIS AFTER CALIBRATION
int targetGreenG = 24;
int targetRedG = 28;
int targetGreenR = 35;
int targetRedR = 90;

NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);

Servo myServo;
int pos = 0;

// left motor is backwards
// forward pin spins CCW
// backward pin spins CW

void setup() {
  // put your setup code here, to run once:
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, HIGH); // enable right motor
  digitalWrite(ENB, HIGH); // enable left motor

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT); 


  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);

  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);

  myservo.attach(9); 

  Serial.begin(9600);
}

void followLinePID() {
  int greenValue = readGreen();
  int redValue = readRed();

  error = (targetGreenG - greenValue) + (targetRedG - redValue);
  integral += error;
  float derivative = error - lastError;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  leftSpeed  = baseSpeed + correction;
  rightSpeed = baseSpeed - correction;

  leftSpeed  = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // Forward motion
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);

  lastError = error;
}

void followLinePIDRed() {
  int greenValue = readGreen();
  int redValue = readRed();

  error = (targetGreenR - greenValue) + (targetRedR - redValue);
  integral += error;
  float derivative = error - lastError;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  leftSpeed  = baseSpeed + correction;
  rightSpeed = baseSpeed - correction;

  leftSpeed  = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // Forward motion
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);

  lastError = error;
}

int readGreen() {
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delay(5);
  return pulseIn(outPin, LOW);
}

int readRed() {
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delay(5);
  return pulseIn(outPin, LOW);
}

void debugOutput() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    int g = readGreen();
    int r = readRed();
    Serial.print("Green: ");
    Serial.print(g);
    Serial.print(" | Red: ");
    Serial.print(r);
    Serial.print(" | Left Speed: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right Speed: ");
    Serial.print(rightSpeed);
    Serial.print(" Error: ");
    Serial.println(error);
    lastPrint = millis();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  unsigned int distance = sonar.ping_cm();
  Serial.println(distance);

  int stateLeft = digitalRead(IR_PIN_LEFT);
  int stateRight = digitalRead(IR_PIN_RIGHT);

  //

  if (stateLeft == LOW && stateRight == HIGH) {
    turnRight(1000, 255, 255);
  } else if (stateRight == LOW && stateLeft == HIGH) {
    turnLeft(1000, 255, 255);
  driveForward(1000, 255, 255);

  //
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delay(5);
  int red = pulseIn(out, LOW);

  // Read GREEN
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delay(5);
  int green = pulseIn(out, LOW);

  // Read BLUE
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  delay(5);
  int blue = pulseIn(out, LOW);

  Serial.print("R: "); Serial.print(red);
  Serial.print(" G: "); Serial.print(green);
  Serial.print(" B: "); Serial.println(blue);

  // starting upload
  if (green < 50 && !(blue < 50 && red < 50) && storePathColour == 0) {
    followLinePID();
    debugOutput();
  } else if (red < 50 && !(blue < 50 && green < 50) && storePathColour == 0) {
    turnRight(3000);
    driveForward(2000);
    storePathColour = 1; // change to red
  } else if (red < 50 && !(blue < 50 && green < 50) && storePathColour == 1) {
    followLinePIDRed();
    debugOutput(); // PID
  } else if (blue < 50 && !(red < 50 && green < 50) && storePathColour == 1) {
    driveForward(3000);
  } else { // condition if gray
    stopDriving(10000);
  }

  delay(500);
}

int GetData() {
  data = pulseIn(out, LOW);
  delay(20);
  return data;
}

void driveForward(int delayTime) {

  analogWrite(leftMotorSpeedPin, leftSpeed);
  analogWrite(rightMotorSpeedPin, rightSpeed);

  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);
  delay(delayTime);
}

void driveBackward(int delayTime) {

  analogWrite(leftMotorSpeedPin, leftSpeed);
  analogWrite(rightMotorSpeedPin, rightSpeed);

  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, HIGH);
  delay(delayTime);
}

// fix because idk?
void turnLeft(int delayTime) {

  analogWrite(leftMotorSpeedPin, leftSpeed);
  analogWrite(rightMotorSpeedPin, rightSpeed);

  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);
  delay(delayTime);
}

// fix because idk?
void turnRight(int delayTime) {

  analogWrite(leftMotorSpeedPin, leftSpeed);
  analogWrite(rightMotorSpeedPin, rightSpeed);

  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, HIGH);
  delay(delayTime);
}

void stopDriving(int delayTime) {
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
  delay(delayTime);
}

void moveArmUp(int delayTime) {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(delayTime);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(delayTime);                       // waits 15ms for the servo to reach the position
  }
}

void moveArmDown(int delayTime) {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(delayTime);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(delayTime);                       // waits 15ms for the servo to reach the position
  }
}