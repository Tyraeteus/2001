#include "PID_v1.h"
#include "L3G.h"
#include "Drive.h"

Drive::Drive(){
  
}

double Drive::readAngle(void) {
  return 0;
}

void Drive::doTurn(double degrees, double speed) {
  angleSetpoint = degrees;
  angleInput = readAngle();
  anglePID.Compute();
  drive(speed, angleOutput);  
}

void Drive::doDrive(double speed, double turn) {
  if(turn == 0) {
    driveMotors(speed, speed);
  } else if (turn < 0) {
    driveMotors((1 - turn) * speed, speed);
  } else {
    driveMotors(speed,  (1 - turn) * speed);
  }
}

void Drive::driveMotors(double left, double right) {
  moveLeftDrive(left);
  moveRightDrive(right);
}

void Drive::stopRobot() {
  driveMotors(0, 0);
}

void Drive::moveLeftDrive(double speed) {
  if(speed > 0) {
    analogWrite(leftDriveFow, 255 * speed);
    digitalWrite(leftDriveRev, LOW);
  } else {
    digitalWrite(leftDriveFow, LOW);
    analogWrite(leftDriveRev, 255*speed);
  }
}

void Drive::moveRightDrive(double speed) {
  if(speed > 0) {
    analogWrite(rightDriveFow, 255 * speed);
    digitalWrite(rightDriveRev, LOW);
  } else {
    digitalWrite(rightDriveFow, LOW);
    analogWrite(rightDriveRev, 255*speed);
  }
}

boolean Drive::isAtTube() {
  return digitalRead(tubeSwitchPin);
}

boolean Drive::isAtIntersection() {
  boolean reading = true;
  readLineSensor();
  for(int i = 0; i < numSensors; i++) {
    if(!(lineSensorReading[i] > threshold)) {
      reading = false;
    }
  }
  return reading;
}


void Drive::lineFollow(double speed) {
  lineSetpoint = 2500;
  lineInput = getLinePosition();
  linePID.Compute();
  drive(speed, lineOutput);
}


void Drive::readLineSensor() {
  linePosition = lineSensor.readLine(lineSensorReading);
}

double Drive::getLinePosition() {
  readLineSensor();
  return lineSensorReading[6];
}
