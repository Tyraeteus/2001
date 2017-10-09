#include "Arduino.h"
#include "Wire.h"
#include "Messages.h"
#include "QTRSensors.h"
#include "PID_v1.h"
#include "L3G.h"
#include "Encoder.h"
#include "Servo.h"

//Bluetooth communication
Messages msg;
unsigned long timeForHeartbeat;

//analog pins
const int armPotPin = 7;

//digital pins
const int armPin = 4;
const int leftDriveFow = 9;
const int leftDriveRev = 8;
const int rightDriveFow = 6;
const int rightDriveRev = 7;
const int leftEncPinA = 0;
const int leftEncPinB = 0;
const int rightEncPinA = 0;
const int rightEncPinB = 0;
const int tubeSwitchPin = 0;

//Servo
int gripperPin = 5;
Servo gripper;
Servo arm;

//Encoders
Encoder leftEnc(leftEncPinA, leftEncPinB);
Encoder rightEnc(rightEncPinA, rightEncPinB);;

//Gyro
//SDA is 20 SCL is 21
L3G gyro;

//Line Sensor
const int numSensors = 6;
const int emitterPin = 35;
unsigned int lineSensorReading[numSensors + 1];
unsigned int linePosition;
unsigned int threshold = 0;
const unsigned char lineSensorPins[numSensors] = {23, 25, 27, 29, 30, 33};
QTRSensorsRC lineSensor(lineSensorPins, numSensors, 2500, emitterPin);

//Line follwoing PID controller
double kPLine;
double kILine;
double kDLine;
double lineSetpoint;
double lineInput;
double lineOutput;
PID linePID(&lineInput, &lineOutput, &lineSetpoint, kPLine, kILine, kDLine, DIRECT);

//Turning PID controller
double kPAngle;
double kIAngle;
double kDAngle;
double angleSetpoint;
double angleInput;
double angleOutput;
PID anglePID(&angleInput, &angleOutput, &angleSetpoint, kPAngle, kIAngle, kDAngle, DIRECT);

//Arm PID controller
double kPArm;
double kIArm;
double kDArm;
double armSetpoint;
double armInput;
double armOutput;
PID armPID(&armInput, &armOutput, &armSetpoint, kPArm, kIArm, kDArm, DIRECT);

//Routing information
int intsFromReactor = 0;
int intsFromStorage = 0;
int turnFromStorage = 0;
int intsFromSupplyArea = 0;
int intsFromStorageArea = 0;
boolean routesCalculated = false;

//Operation variables
int intsCrossed = 0;
double globalSpeed = 1;
bool wasAtInt = false;
bool isLowRadiation = false;
bool isHighRadiation = false;
unsigned int armTargetPos = 0;
const unsigned int armUp = 580;
const unsigned int armHorizGrab = 345;
const unsigned int armDown = 30;
const unsigned int gripperOpen = 0;
const unsigned int gripperClosed = 0;

enum robotState {
  DRIVING_TO_INTERSECTION,
  DRIVING_TO_TUBE,
  DRIVING_TO_WALL,
  TURNING_LEFT_90,
  TURNING_RIGHT_90,
  TURNING_180,
  RAISING_GRIPPER,
  LOWERING_GRIPPER,
  OPENING_GRIPPER,
  CLOSING_GRIPPER,
  STOPPED
}robotState;

enum operationState {
  CALIBRATING_LINE_SENSOR,
  DRIVING_TO_REACTOR,
  REMOVING_VERTICAL_ROD,
  DRIVING_TO_STORAGE,
  INSERTING_HORIZONTAL_ROD,
  DRIVING_TO_SUPPLY,
  REMOVING_HORIZONTAL_ROD,
  INSERTING_VERTICAL_ROD,
  OPERATION_STOPPED
}operationState;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");
  Wire.begin();
  gyro.enableDefault();
  msg.setup();
  timeForHeartbeat = millis() + 1000;
  gripper.attach(gripperPin, 1000, 2000);
  arm.attach(armPin, 1000, 2000);
  robotState = STOPPED;
  operationState = CALIBRATING_LINE_SENSOR;

}

void loop() {
  if(!routesCalculated) {
    calculateRoutes();
  }

  gyro.read();
  sendHeartbeat();
  msg.printMessage();

  switch(robotState) {
    case DRIVING_TO_INTERSECTION:
      if(isAtIntersection()) {
        if(wasAtInt) {
          drive(globalSpeed, 0);
        } else {
          stopRobot();
          intsCrossed++;
          wasAtInt = true;
        }
      } else {
        wasAtInt = false;
        lineFollow(globalSpeed);
      }
      break;
    case DRIVING_TO_TUBE:
      if(isAtTube) {
        stopRobot();
      } else {
        lineFollow(globalSpeed);
      }
      break;
    case DRIVING_TO_WALL:
      if(isAtTube) {
        stopRobot();
      } else {
        lineFollow(globalSpeed);
      }
      break;
    case TURNING_LEFT_90:
      turn(90, globalSpeed);
      break;
    case TURNING_RIGHT_90:
      turn(-90, globalSpeed);
      break;
    case TURNING_180:
      turn(180, globalSpeed);
      break;
    case RAISING_GRIPPER:
      moveArm(armTargetPos);
      break;
    case LOWERING_GRIPPER:
      moveArm(armTargetPos);
      break;
    case OPENING_GRIPPER:
      openGripper();
      break;
    case CLOSING_GRIPPER:
      closeGripper();
      break;
  }

  switch(operationState) {
    
  }
}

void sendHeartbeat() {
  if (millis() > timeForHeartbeat) {
    timeForHeartbeat = millis() + 1000;
    msg.sendHeartbeat();
  }
}

void readLineSensor() {
  linePosition = lineSensor.readLine(lineSensorReading);
}

double getLinePosition() {
  readLineSensor();
  return lineSensorReading[6];
}

void sendStatus() {
  
}

void calculateRoutes() {
   intsFromReactor = msg.getFirstStorage();
   intsFromStorageArea = abs(msg.getFirstSupply() - msg.getFirstStorage());
   if(msg.getFirstSupply() - msg.getFirstStorage() > 0) {
    turnFromStorage = 1;
   } else if(msg.getFirstSupply() - msg.getFirstStorage() < 0) {
    turnFromStorage = -1;
   } else {
    turnFromStorage = 0;
   }
   intsFromSupplyArea = msg.getFirstSupply();
   routesCalculated = true;
}

void moveArmToPos(int position) {
  armSetpoint = position;
  armInput = getArmPos();
  armPID.Compute();
  moveArm(armOutput);
}

unsigned int getArmPos() {
  return analogRead(armPotPin);
}

void moveArm(double speed) {
 int dutyCycle = abs(speed) * 255;
  if(speed > 0) {
    analogWrite(armFow, dutyCycle);
    digitalWrite(armRev, LOW);
  } else {
    digitalWrite(armFow, LOW);
    analogWrite(armRev, dutyCycle);
  }
}

void lineFollow(double speed) {
  lineSetpoint = 2500;
  lineInput = getLinePosition();
  linePID.Compute();
  drive(speed, lineOutput);
}

boolean isAtIntersection() {
  boolean reading = true;
  readLineSensor();
  for(int i = 0; i < numSensors; i++) {
    if(!(lineSensorReading[i] > threshold)) {
      reading = false;
    }
  }
  return reading;
}

double readAngle() {
  return 0;
}

void turn(double degrees, double speed) {
  angleSetpoint = degrees;
  angleInput = readAngle();
  anglePID.Compute();
  drive(speed, angleOutput);  
}

void drive(double speed, double turn) {
  if(turn == 0) {
    driveMotors(speed, speed);
  } else if (turn < 0) {
    driveMotors((1 - turn) * speed, speed);
  } else {
    driveMotors(speed,  (1 - turn) * speed);
  }
}

void driveMotors(double left, double right) {
  moveLeftDrive(left);
  moveRightDrive(right);
}

void stopRobot() {
  driveMotors(0, 0);
}

void moveLeftDrive(double speed) {
  if(speed > 0) {
    analogWrite(leftDriveFow, 255 * speed);
    digitalWrite(leftDriveRev, LOW);
  } else {
    digitalWrite(leftDriveFow, LOW);
    analogWrite(leftDriveRev, 255*speed);
  }
}

void moveRightDrive(double speed) {
  if(speed > 0) {
    analogWrite(rightDriveFow, 255 * speed);
    digitalWrite(rightDriveRev, LOW);
  } else {
    digitalWrite(rightDriveFow, LOW);
    analogWrite(rightDriveRev, 255*speed);
  }
}

boolean isAtTube() {
  return digitalRead(tubeSwitchPin);
}

void openGripper() {
  gripper.write(gripperOpen);
}

void closeGripper() {
  gripper.write(gripperClosed);
}

