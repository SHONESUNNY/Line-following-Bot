#include <AFMotor.h>

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
float Kp = 66.6, Ki = 0, Kd =19.5;
float error = 0, previousError = 0, totalError = 0;
int baseSpeed = 76;  // Base speed for both motors

void setup() {
  Serial.begin(9600);  // Initialize Serial communication
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

// Case-based handling for special scenarios
void handleSpecialCases() { 
  uint16_t leftFar = sensorValues[0];
  uint16_t leftNear = sensorValues[1];
  uint16_t Centre = sensorValues[2];
  uint16_t rightNear = sensorValues[3];
  uint16_t rightFar = sensorValues[4];

  // Case 1: Sharp turn left (both left sensors detect black)
  if (leftFar < black_limit && leftNear < black_limit) {
    Serial.println("Case: Sharp Left");
    motorLeft.run(BACKWARD);
    motorRight.run(FORWARD);
    delay(100);  // Adjust for smooth turning 
    return;
  }

  // Case 2: Sharp turn right (both right sensors detect black)
  if (rightFar < black_limit && rightNear < black_limit) {
    Serial.println("Case: Sharp Right");
    motorLeft.run(FORWARD);
    motorRight.run(BACKWARD);
    delay(100);  // Adjust for smooth turning
    return;
  }

  // Case 3: Robot is completely off the line (all sensors detect white)
  if (leftFar > white_limit && Centre > white_limit && rightFar > white_limit &&
      leftNear > white_limit && rightNear > white_limit) {
    Serial.println("Case: Lost Line Moving Backward");
    motorLeft.run(BACKWARD);
    motorRight.run(BACKWARD);
    delay(300);  // Reverse to reposition
    return;
  }

  // Default: Use PID control to follow the line smoothly
  applyPIDControl();
}

void loop() {
  readSensorValues();

  applyPIDControl();
  //handleSpecialCases();
  //delay(50);  // Small delay for readability
}