#include "NewPing.h"

int leftMotorForwardPin = 7;
int leftMotorBackwardPin = 6;
int rightMotorForwardPin = 11;
int rightMotorBackwardPin = 10;
int leftMotorSpeedPin = 6;
int rightMotorSpeedPin = 5;

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

// IR
// int IR_PIN_LEFT = 7;
// int IR_PIN_RIGHT = 8; // change

NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);

// left motor is backwards
// forward pin spins CCW
// backward pin spins CW

void setup() {
  // put your setup code here, to run once:
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, HIGH); // enable right motor
  digitalWrite(ENB, HIGH); // enable left motor

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT); 

  // pinMode(IR_PIN, INPUT);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);

  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);

  Serial.begin(9600);

  

}

void loop() {
  // put your main code here, to run repeatedly:
  // driveForward(2000, 255, 255);
  // driveForward(1000, 128, 128); // slows down motor after 2 seconds

  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  unsigned int distance = sonar.ping_cm();
  Serial.println(distance);

  // int stateLeft = digitalRead(IR_PIN_LEFT);
  // int stateRight = digitalRead(IR_PIN_RIGHT);

  /*
  if both high, then nothing
  if stateleft = low and stateright = high, turn right
  if stateright = low and stateleft = high, turn left
  if both low, stop
  */

  // if (stateLeft == LOW && stateRight == HIGH) {
  //   turnRight(1000, 255, 255);
  // } else if (stateRight == LOW && stateLeft == HIGH) {
  //   turnLeft(1000, 255, 255);
  // }

  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  int red = pulseIn(out, LOW);

  // Read GREEN
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  int green = pulseIn(out, LOW);

  // Read BLUE
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  int blue = pulseIn(out, LOW);

  Serial.print("R: "); Serial.print(red);
  Serial.print(" G: "); Serial.print(green);
  Serial.print(" B: "); Serial.println(blue);

  delay(500);
  // red
  // digitalWrite(s2, LOW);        //S2/S3  levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH  is for Blue and HIGH/HIGH is for green
  // digitalWrite(s3, LOW);
  // int data1 = GetData();

  // // green
  // digitalWrite(s2, LOW);
  // digitalWrite(s3, HIGH);
  // int data2 = GetData();

  // // blue
  // digitalWrite(s2, HIGH);
  // digitalWrite(s3, HIGH);
  // int data3 = GetData();

  // Serial.println(data1, data2, data3);

  // delay(2000);
}

int GetData() {
  data = pulseIn(out, LOW);
  delay(20);
  return data;
}

void driveForward(int delayTime, int leftSpeed, int rightSpeed) {

  analogWrite(leftMotorSpeedPin, leftSpeed);
  analogWrite(rightMotorSpeedPin, rightSpeed);

  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);
  delay(delayTime);
}

void driveBackward(int delayTime, int leftSpeed, int rightSpeed) {

  analogWrite(leftMotorSpeedPin, leftSpeed);
  analogWrite(rightMotorSpeedPin, rightSpeed);

  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, HIGH);
  delay(delayTime);
}

// fix because idk?
void turnLeft(int delayTime, int leftSpeed, int rightSpeed) {

  analogWrite(leftMotorSpeedPin, leftSpeed);
  analogWrite(rightMotorSpeedPin, rightSpeed);

  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);
  delay(delayTime);
}

// fix because idk?
void turnRight(int delayTime, int leftSpeed, int rightSpeed) {

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

// // forward pin -> clockwise
// // backward pin -> counterclockwise

// // right
// int rightMotorForwardPin = 6; // forward 
// int rightMotorBackwardPin = 7; // backward

// int leftMotorForwardPin = 10; // forward (right)
// int leftMotorBackwardPin = 11; // backward
// int leftMotorForwardPin = 7;
// int leftMotorBackwardPin = 6;
// int rightMotorForwardPin = 11;
// int rightMotorBackwardPin = 10;

// int ENA = 3; // enable motor 1 (right)
// int ENB = 5; // enable motor 2 (left)

// // for ultrasonic distance sensor
// // const int triggerPin = 8; // change?
// // const int echoPin = 7; // change?

// // long duration;
// // long cm;

// void setup() {
//   // put your setup code here, to run once:
//   pinMode(rightMotorForwardPin, OUTPUT);
//   pinMode(rightMotorBackwardPin, OUTPUT);
//   pinMode(leftMotorForwardPin,  OUTPUT);
//   pinMode(leftMotorBackwardPin, OUTPUT);

//   pinMode(ENA, OUTPUT);
//   pinMode(ENB, OUTPUT);

//   digitalWrite(ENA, HIGH); // enable motor 1 (right)
//   digitalWrite(ENB, HIGH); // enable motor 2 (left)

//   // Serial.begin(9600);
//   // pinMode(triggerPin, OUTPUT);
//   // pinMode(echoPin, OUTPUT);

// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   // TEST WHICH MOTOR THIS IS
//   digitalWrite(rightMotorForwardPin,  HIGH); // forward
//   digitalWrite(rightMotorBackwardPin, LOW); // backward

//   digitalWrite(leftMotorForwardPin, LOW); // forward
//   digitalWrite(leftMotorBackwardPin, HIGH); // backward
//   delay(3000);

//   // digitalWrite(motor1pin1,  LOW);
//   // digitalWrite(motor1pin2, HIGH);

//   // digitalWrite(motor2pin1, LOW);
//   // digitalWrite(motor2pin2, HIGH);
//   // delay(3000);
// }
