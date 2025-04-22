#include <AFMotor.h>

unsigned int wait = 300;
unsigned int wait2 = 400;

// Motor Definitions
AF_DCMotor motorRight(4);
AF_DCMotor motorLeft(3);

// Define the analog pins for the IR sensors
const int sensorPins[5] = {A0, A1, A2, A3, A4};
int sensorValues[5];  // Store sensor readings

// Threshold values for white (no line) and black (line)
const int white_limit = 200;  // Adjust based on calibration
const int black_limit = 100;  // Adjust based on calibration

// PID Variables
float Kp = 30, Ki = 0, Kd = 14;
float error = 0, previousError = 0, totalError = 0;
int baseSpeed = 165;  // Base speed for both motors

// Path recording variables
char recentMoves[5];  // Store the last 10 moves
int moveIndex = 0;

void recordMove(char direction) {
  recentMoves[moveIndex] = direction;
  moveIndex = (moveIndex + 1) % 5;  // Wrap around after 10 moves
}

void analyzePath() {
  int leftMoves = 0;
  int rightMoves = 0;
  
  // Count the number of left and right turns in the recent moves
  for (int i = 0; i < 10; i++) {
    if (recentMoves[i] == 'L') leftMoves++;
    if (recentMoves[i] == 'R') rightMoves++;
  }
  
  // If there are more left turns, adjust the error to favor right movement
  if (leftMoves > rightMoves) {
    error -= 0.5;
  } 
  // If there are more right turns, adjust the error to favor left movement
  else if (rightMoves > leftMoves) {
    error += 0.5;
  }
}

void Forward() {
  applyPIDControl();
}

void STOP() {
  motorLeft.run(RELEASE);  // Stop the left motor
  motorRight.run(RELEASE); // Stop the right motor
}

void moveRight() {
  motorLeft.setSpeed(baseSpeed);
  motorRight.setSpeed(baseSpeed);
  motorLeft.run(FORWARD);               // Turn left motor forward
  motorRight.run(BACKWARD);             // Turn right motor backward
  recordMove('R');
}

void moveLeft() {
  motorLeft.setSpeed(baseSpeed);
  motorRight.setSpeed(baseSpeed);
  motorLeft.run(BACKWARD);             
  motorRight.run(FORWARD);              // Turn right motor forward
  recordMove('L');
}

void Backward() {
  motorLeft.setSpeed(baseSpeed);
  motorRight.setSpeed(baseSpeed);
  motorLeft.run(BACKWARD);           
  motorRight.run(FORWARD);
  recordMove('B');
}

void setup() {
  Serial.begin(9600);  // Initialize Serial communication
  motorLeft.setSpeed(baseSpeed);
  motorRight.setSpeed(baseSpeed);
  motorLeft.run(FORWARD);
  motorRight.run(FORWARD);
}

void readSensorValues() {
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

float computeError() {
  // Weights for each sensor: [leftFar, leftNear, Centre, rightNear, rightFar]
  float weights[5] = {-2, -1, 0, 1, 2};  
  float weightedSum = 0;
  float totalActiveSensors = 0;

  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] < black_limit) {  // Sensor on the line
      weightedSum += weights[i];
      totalActiveSensors++;
    }
  }

  if (totalActiveSensors == 0) {
    return (previousError > 0) ? 2 : -2;  // Turn in the direction of previous error
  }

  return weightedSum / totalActiveSensors;  // Average error
}

void applyPIDControl() {
  error = computeError();
  analyzePath();  // Modify error based on the path log

  float P = error;
  totalError += error;
  float I = totalError;
  float D = error - previousError;

  float pidOutput = (Kp * P) + (Ki * I) + (Kd * D);

  int leftMotorSpeed = baseSpeed + pidOutput;
  int rightMotorSpeed = baseSpeed - pidOutput;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  motorLeft.setSpeed(leftMotorSpeed);
  motorRight.setSpeed(rightMotorSpeed);

  motorLeft.run(FORWARD);
  motorRight.run(FORWARD);

  previousError = error;
}

void loop() {
  readSensorValues();
  uint16_t leftFar = sensorValues[0];
  uint16_t leftNear = sensorValues[1];
  uint16_t Centre = sensorValues[2];
  uint16_t rightNear = sensorValues[3];
  uint16_t rightFar = sensorValues[4];

  if (leftFar < black_limit && leftNear < black_limit) {
    moveLeft();
  }
  if (rightFar < black_limit && rightNear < black_limit && leftFar > white_limit && leftNear > white_limit) {
    moveRight();
  }

  applyPIDControl();
}
