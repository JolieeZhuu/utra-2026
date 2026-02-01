// ===== PID Line Follower using TCS230/TCS3200 + L298N (UNO) =====

// -------------------- TCS230 pins (your pinout) --------------------
const int S0  = A0;
const int S1  = A1;
const int OUT = A2;
const int S2  = A3;
const int S3  = A4;

// -------------------- Motor pins (your wiring) --------------------
// L298N direction pins
const int leftMotorForwardPin  = 6;   // IN1 (left forward)
const int leftMotorBackwardPin = 7;   // IN2 (left backward)
const int rightMotorForwardPin  = 11; // IN3 (right forward)
const int rightMotorBackwardPin = 10; // IN4 (right backward)

// L298N enable pins (PWM speed control)
const int ENA = 3; // right motor enable (ENA)  PWM
const int ENB = 5; // left motor enable (ENB)   PWM

// -------------------- PID tuning --------------------
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.9;

int baseSpeed = 110;     // adjust for your robot
int minSpeed  = 0;
int maxSpeed  = 255;

// Target: 0.5 means "midway between white and black".
// For edge-following, try 0.35 or 0.65 depending on sensor placement.
float targetDarkness = 0.5;

// -------------------- PID state --------------------
float err = 0, prevErr = 0, integ = 0;

// -------------------- Calibration --------------------
float whiteC = 0;  // clear pulse on white
float blackC = 0;  // clear pulse on black line

// Line lost handling
unsigned long lastOnLineMs = 0;
const unsigned long LOST_TIMEOUT_MS = 450; // time before recovery behavior

// -------------------- Sensor reading helpers --------------------
int readPulseAvg(int s2v, int s3v) {
  digitalWrite(S2, s2v);
  digitalWrite(S3, s3v);
  delay(2); // filter settle time

  long sum = 0;
  const int N = 8;
  for (int i = 0; i < N; i++) {
    long p = pulseIn(OUT, LOW, 30000); // timeout 30ms
    if (p == 0) p = 30000;
    sum += p;
  }
  return (int)(sum / N);
}

// Standard mapping for TCS230:
// RED:   S2=LOW  S3=LOW
// BLUE:  S2=LOW  S3=HIGH
// GREEN: S2=HIGH S3=HIGH
// CLEAR: S2=HIGH S3=LOW
int readRed()   { return readPulseAvg(LOW,  LOW);  }
int readBlue()  { return readPulseAvg(LOW,  HIGH); }
int readGreen() { return readPulseAvg(HIGH, HIGH); }
int readClear() { return readPulseAvg(HIGH, LOW);  }

// Convert clear pulse to normalized darkness [0..1]
// pulseIn gives larger numbers for darker (black) and smaller for bright (white)
float normalizedDarkness(float cPulse) {
  if (abs(blackC - whiteC) < 1.0) return 0.0;
  float x = (cPulse - whiteC) / (blackC - whiteC); // white->0, black->1
  if (x < 0) x = 0;
  if (x > 1) x = 1;
  return x;
}

// -------------------- Motor control (L298N) --------------------
void drive(int leftSpeed, int rightSpeed) {
  // leftSpeed/rightSpeed: -255..255
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // LEFT direction
  if (leftSpeed >= 0) {
    digitalWrite(leftMotorForwardPin, HIGH);
    digitalWrite(leftMotorBackwardPin, LOW);
  } else {
    digitalWrite(leftMotorForwardPin, LOW);
    digitalWrite(leftMotorBackwardPin, HIGH);
    leftSpeed = -leftSpeed;
  }

  // RIGHT direction
  if (rightSpeed >= 0) {
    digitalWrite(rightMotorForwardPin, HIGH);
    digitalWrite(rightMotorBackwardPin, LOW);
  } else {
    digitalWrite(rightMotorForwardPin, LOW);
    digitalWrite(rightMotorBackwardPin, HIGH);
    rightSpeed = -rightSpeed;
  }

  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // PWM on enables
  analogWrite(ENB, leftSpeed);   // left motor speed
  analogWrite(ENA, rightSpeed);  // right motor speed
}

void stopDriving() {
  analogWrite(ENB, 0);
  analogWrite(ENA, 0);
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
}

// -------------------- Calibration routine --------------------
void waitForKey(const char *msg) {
  Serial.println(msg);
  while (!Serial.available()) {}
  while (Serial.available()) Serial.read();
  delay(200);
}

float averageClear(int samples = 25) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readClear();
    delay(15);
  }
  return sum / (float)samples;
}

void calibrateBW() {
  stopDriving();

  waitForKey("Calibration: place sensor over WHITE and press any key...");
  whiteC = averageClear(30);
  Serial.print("whiteC(clear pulse) = ");
  Serial.println(whiteC);

  waitForKey("Calibration: place sensor over BLACK line and press any key...");
  blackC = averageClear(30);
  Serial.print("blackC(clear pulse) = ");
  Serial.println(blackC);

  Serial.print("Using targetDarkness = ");
  Serial.println(targetDarkness, 3);

  // Basic sanity
  if (blackC < whiteC) {
    Serial.println("Warning: blackC < whiteC (unexpected). Swapping them.");
    float tmp = blackC;
    blackC = whiteC;
    whiteC = tmp;
  }

  lastOnLineMs = millis();
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(9600);

  // TCS230 pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Frequency scaling: 20% (stable)
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Motor pins
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopDriving();
  delay(300);

  calibrateBW();

  waitForKey("Press any key to START driving...");
  Serial.println("Starting PID line follower...");
}

void loop() {
  // Read clear channel for black/white tracking
  float cPulse = (float)readClear();
  float darkness = normalizedDarkness(cPulse);  // 0..1 (white->0, black->1)

  // Consider "on line" if it's sufficiently dark
  // Tweak 0.6 depending on your line contrast
  bool onLine = (darkness > 0.60);
  if (onLine) lastOnLineMs = millis();

  // PID error: target - measurement
  err = targetDarkness - darkness;

  // PID terms
  integ += err;
  integ = constrain(integ, -1.2, 1.2); // anti-windup clamp
  float deriv = err - prevErr;
  prevErr = err;

  float correction = Kp * err + Ki * integ + Kd * deriv;

  // Convert correction -> turning amount
  // Larger multiplier = sharper steering
  int turn = (int)(correction * 140.0);

  int left  = baseSpeed + turn;
  int right = baseSpeed - turn;

  left  = constrain(left,  minSpeed, maxSpeed);
  right = constrain(right, minSpeed, maxSpeed);

//  // Line-lost recovery: if we haven't seen the line recently, spin to search
//  if (millis() - lastOnLineMs > LOST_TIMEOUT_MS) {
//    // Spin slowly to find line
//    // You can flip directions if it searches the wrong way
//    drive(80, -80);
//  } else {
//    drive(left, right);
//  }

  // Debug prints (comment out once working)
  Serial.print("cPulse="); Serial.print(cPulse);
  Serial.print(" dark="); Serial.print(darkness, 3);
  Serial.print(" err="); Serial.print(err, 3);
  Serial.print(" turn="); Serial.print(turn);
  Serial.print(" L="); Serial.print(left);
  Serial.print(" R="); Serial.print(right);
  Serial.print(" onLine="); Serial.println(onLine ? "Y" : "N");

  delay(10);
}
