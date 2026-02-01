#include "NewPing.h"
#include "Servo.h"

// black -> red -> green

int leftMotorForwardPin = 7;
int leftMotorBackwardPin = 6;
int rightMotorForwardPin = 11;
int rightMotorBackwardPin = 10;
// int leftMotorSpeedPin = 6;
// int rightMotorSpeedPin = 5;

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

// IR
// int IR_PIN_LEFT = 7;
// int IR_PIN_RIGHT = 8; // change

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
  // pinMode(leftMotorSpeedPin, OUTPUT);
  // pinMode(rightMotorSpeedPin, OUTPUT);

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

  myservo.attach(9); 

  Serial.begin(9600);

  

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
  if (!(red >= 50 && green >= 50 && blue >= 50)) { // if it's not black
    // turn right slightly and then move forward
    turnRight(3000); // like 90 to 100 degs
    driveForward(2000);
  } else if (blue < 50 && !(red < 50 && green < 50)) { // ensure not white
    turnRight(3000); // hopefully 90 degrees
    driveForward(2000);
    moveArmUp(2000);
    turnRight(6000);
    driveForward(4000);
    moveArmDown(2000);
    driveBackward(1000);
    turnRight(3000);
    driveForward(2000);
    turnLeft(3000);
    driveForward(2000);
  } else if (green < 50 && !(blue < 50 && green < 50)) {
    driveForward(2000); // PID
  } else { // condition for grey
    stopDriving(10000);
  }

    /*
    // turn right 90 degrees
    // move forward
    // pick up using arm
    // turn right 180 degrees
    // move forward * 2
    // drop block off
    // turn right 90 degrees
    // move forward
    // turn left 90 degrees
    // move forward so its green path
    */

  // reupload 2 (ball)
  // current path colour is green (0), change to black square (1)
  if (!(red >= 50 && green >= 50 && blue >= 50)) { //drive forward until the black square is detected
    driveForward(5000);
  } else if (red >= 50 && green >= 50 && blue >= 50) {
    storePathColour = 1; // change to black
    driveForward(2000);
  } else if (red < 50 && !(green < 50 && blue < 50) && storePathColour == 1) {
    stopDriving(3000); // lowkey just a test, who knows how much the ball will roll
    turnRight(4000); // 180 deg turn
    driveForward(3000); // im HOPING this is the grey zone
    storePathColour = 0; // store back to green (0)
  }
  //drive forward until black is detected
  //stop driving
  //drive forward, pushing the ball, until red is detected
  //stop driving, let the ball roll forward to the blue zone
  //turn 180 degrees


  // reupload 3 (back to green then red)
  // green is 0, red is 1
  if (green < 50 && !(blue < 50 && red < 50) && storePathColour == 0) {
    driveForward(2000);
  } else if (red < 50 && !(blue < 50 && green < 50) && storePathColour == 0) {
    turnRight(3000);
    driveForward(2000);
    storePathColour = 1; // change to red
  } else if (red < 50 && !(blue < 50 && green < 50) && storePathColour == 1) {
    driveForware(2000); // PID
  } else if (blue < 50 && !(red < 50 && green < 50) && storePathColour == 1) {
    turnRight(3000);
    driveForward(3000);
    moveArmUp(2000);
    turnRight(6000);
    driveForward(6000);
    turnRight(3000);
    driveForward(2000);
  } else { // condition if gray
    stopDriving(10000);
  }

  /*
  store path colour --> make as variable
  if green, move forward, store path colour as green 
  if red and the original colour was green, then rotate 90 degrees right, then move forward, store path colour as red
  if red and store path colour is red, then move forward
  if blue, and path colour is red, then:
    turn right 90 degrees, move forward, pick up box, turn 180 degrees, move forward, turn 90 degrees right, move forward
  if grey, then stop
  */

  // reupload 4 (last route, red)
  storePathColour = 1; // red is 1, black is 2
  if (distance <= 10) {
    turnLeft(2000);
    driveForward(2000);
    turnRight(2000);
    driveForward(2000);
    turnRight(2000);
    driveForward(2000);
    turnLeft(2000);
    driveForward(2000);
  } else if (blue < 50 && !(red < 50 && green < 50)) {
    turnLeft(2000);
    driveForward(2000);
    moveArmDown(2000);
    driveBackward(1000);
    turnLeft(4000);
    driveForward(2000);
    turnLeft(2000);
    driveForward(2000);
  } else if (red < 50 && !(blue < 50 && green < 50)) {
    driveForward(2000); // PID
  } else if (red >= 50 && green >= 50 && blue >= 50 && storePathColour == 1) {
    storePathColour == 2; // store as black, although it doesn't really matter
    turnRight(3000);
    driveForward(2000);
  } else if (red >= 50 && green >= 50 && blue >= 50) {
    driveForward(5000);
    stopDriving(10000);
  }
  /*
  store path colour --> make as variable
  if ultrasonic sensor shows <= 10
  turn left 90 degrees, move forward
    turn right 90 degrees, move forward
    turn right 90 degrees move forward
    turn left 90 degrees, move forward
  if blue, then turn left 90 deg, move forward, drop box off, turn 180 deg, move forward, turn 90 deg left move forward
  if red, move forward
  if black and original path colour was red, then store path colour as black, then turn right like 90-100 deg and move forward
  if black, move forward for like 10 seconds
  */

  // if red, red is below 50
  // if white, all below50
  // if black 

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
