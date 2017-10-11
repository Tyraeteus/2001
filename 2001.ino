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
const int armPotPin = 0;

//digital pins
const int armPin = 5;
const int leftDriveFow = 8;
const int leftDriveRev = 9;
const int rightDriveFow = 7;
const int rightDriveRev = 6;
const int leftEncPinA = 3;
const int leftEncPinB = 2;
const int rightEncPinA = 19;
const int rightEncPinB = 18;
const int tubeSwitchPin = 21;

//Servo
int gripperPin = 10;
Servo gripper;
Servo arm;

//Encoders
Encoder leftEnc(leftEncPinA, leftEncPinB);
Encoder rightEnc(rightEncPinA, rightEncPinB);
const int countsPerRev = 3200;
const double wheelCircumfrence = 8.639;

//Gyro
//SDA is 20 SCL is 21
L3G gyro;

//Line Sensor
const int numSensors = 6;
const int emitterPin = 35;
unsigned int lineSensorReading[numSensors];
unsigned int threshold = 500;
const unsigned char lineSensorPins[numSensors] = {23, 25, 27, 29, 30, 33};
QTRSensorsRC lineSensor(lineSensorPins, numSensors, 2500, emitterPin);

//Line follwoing PID controller
double kPLine = .4;
double kILine = .01;
double kDLine = .07;
double lineSetpoint;
double lineInput;
double lineOutput;
PID linePID(&lineInput, &lineOutput, &lineSetpoint, kPLine, kILine, kDLine, DIRECT);

//Turning PID controller
double kPDist;
double kIDist;
double kDDist;
double distSetpoint;
double distInput;
double distOutput;
PID distPID(&distInput, &distOutput, &distSetpoint, kPDist, kIDist, kDDist, DIRECT);

//Arm PID controller
double kPArm = 1;
double kIArm = .01;
double kDArm = .05;
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
unsigned int armTargetPos = 0;
const unsigned int armUp = 320;
const unsigned int armHorizGrab = 190;
const unsigned int armDown = 10;
const unsigned int armMidGrab = 240;
const unsigned int gripperOpen = 0;
const unsigned int gripperClosed = 180;
boolean isFirstIteration = false;
boolean isSecondVisit = false;
const double backupDistance = 0;
boolean hasLeftLine = false;
boolean hasReachedLine = false;
boolean hasReachedTurn = false;
const double lineOvershoot = 0;
double distanceToDrive = 0;
boolean hasBackedUp = false;
boolean hasReachedTube = false;
boolean gripperInPosition = false;
boolean hasRod = false;
boolean hasTurned = false;
boolean isFirstTurn = false;
boolean isFromSupply = true;
boolean isLowRadiation = false;
boolean isHighRadiation = false;
boolean isSecondIteration = false;
boolean reGrip = false;

enum robotState {
  DRIVING_TO_INTERSECTION,
  DRIVING_TO_TUBE,
  DRIVING_TO_WALL,
  DRIVING_PAST_LINE,
  REVERSING_FROM_TUBE,
  STARTING_LEFT_TURN,
  STARTING_RIGHT_TURN,
  TURNING_LEFT,
  TURNING_RIGHT,
  TURNING_180,
  GRIPPER_TO_TOP,
  GRIPPER_TO_BOTTOM,
  GRIPPER_TO_MIDPOINT,
  OPENING_GRIPPER,
  CLOSING_GRIPPER,
  STOPPED
}robotState;

enum operationState {
  CALIBRATING_LINE_SENSOR,
  DRIVING_TO_REACTOR,
  TURNING_180_FROM_TUBE,
  REMOVING_VERTICAL_ROD,
  DRIVING_TO_STORAGE,
  TURNING_LEFT_90,
  INSERTING_HORIZONTAL_ROD,
  DRIVING_TO_SUPPLY,
  TURNING_RIGHT_90,
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
  linePID.SetOutputLimits(-255, 255);
  distPID.SetOutputLimits(-255, 255);
  armPID.SetOutputLimits(-255, 255);
  timeForHeartbeat = millis() + 1000;
  pinMode(leftDriveFow, OUTPUT);
  pinMode(leftDriveRev, OUTPUT);
  pinMode(rightDriveFow, OUTPUT);
  pinMode(rightDriveRev, OUTPUT);
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
        hasReachedTube = true;
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
    case DRIVING_PAST_LINE:
      driveDistance(globalSpeed, lineOvershoot);
      if(getDistance() == lineOvershoot) {
        hasReachedTurn = true;
      }
      break;
    case REVERSING_FROM_TUBE:
      driveDistance(-globalSpeed, backupDistance);
      if(getDistance() == backupDistance) {
        hasBackedUp = true;
      }
      break;
    case STARTING_LEFT_TURN:
      turnOffLine(globalSpeed, -1);
      break;
    case STARTING_RIGHT_TURN:
    turnOffLine(globalSpeed, 1);
      break;
    case TURNING_LEFT:
      turnToLine(globalSpeed, -1);
      break;
    case TURNING_RIGHT:
      turnToLine(globalSpeed, 1);
      break;
    case TURNING_180:
      turnToLine(globalSpeed, -1);
      break;
    case GRIPPER_TO_TOP:
      moveArm(armUp);
      if(getArmPos() == armUp) {
        gripperInPosition = true;
      }
      break;
    case GRIPPER_TO_BOTTOM:
      moveArm(armDown);
      if(getArmPos() == armDown) {
        gripperInPosition = true;
      }
      break;
    case GRIPPER_TO_MIDPOINT:
      moveArm(armMidGrab);
      if(getArmPos() == armMidGrab) {
        gripperInPosition = true;
      }
    case OPENING_GRIPPER:
      openGripper();
      break;
    case CLOSING_GRIPPER:
      closeGripper();
      break;
  }

  switch(operationState) {
    case CALIBRATING_LINE_SENSOR:
      break;
    case DRIVING_TO_REACTOR:
      if(isFirstIteration) {
        intsCrossed = 0;
        hasTurned = false;
        clearEncoders();
        isFirstIteration = false;
      }
      if(!isSecondVisit) {
        if(isAtTube() || hasReachedTube) {
          operationState = REMOVING_VERTICAL_ROD;
          isFirstIteration = true;
        } else {
          robotState = DRIVING_TO_TUBE;
        }
      } else {
        if(!hasTurned) {
          if(intsCrossed < 1) {
            robotState = DRIVING_TO_INTERSECTION;
          } else {
            operationState = TURNING_RIGHT_90;
            isFirstIteration = true;
          }
        } else {
          if(isAtTube || hasReachedTube) {
            operationState = INSERTING_VERTICAL_ROD;
            isFirstIteration = true;
          } else {
            robotState = DRIVING_TO_TUBE;
          }
        }
      }
      break;
    case TURNING_180_FROM_TUBE:
      if(isFirstIteration) {
        hasBackedUp = false;
        hasLeftLine = false;
        hasReachedLine = false;
        clearEncoders();
        isFirstIteration = false;
      }
      if(!hasBackedUp) {
        robotState = REVERSING_FROM_TUBE;
      } else {
        if(!hasLeftLine) {
          robotState = STARTING_LEFT_TURN;
        } else {
          if(!hasReachedLine) {
            robotState = TURNING_180;
          } else if(hasRod && !isSecondVisit){
            operationState = DRIVING_TO_STORAGE;
            isFirstIteration = true;
          } else if(isSecondVisit) {
            operationState = DRIVING_TO_REACTOR;
            isFirstIteration = true;
          } else {
            operationState = DRIVING_TO_SUPPLY;
            isFirstIteration = true;
            hasTurned = false;
          }
        }
      }
      break;
    case REMOVING_VERTICAL_ROD:
      if(isFirstIteration) {
       gripperInPosition = false; 
      }
      if(!gripperInPosition) {
        robotState = GRIPPER_TO_BOTTOM;
      } else {
        gripperInPosition = true;
        if(!hasRod) {
          robotState = CLOSING_GRIPPER;
          delay(500);
          hasRod = true;
          isLowRadiation = true;
        } else {
          robotState = GRIPPER_TO_TOP;
          if(getArmPos() == armUp) {
          operationState = TURNING_180_FROM_TUBE;
          isFirstIteration = true;
          }
        }
      }
      break;
    case DRIVING_TO_STORAGE:
      if(isFirstIteration) {
        intsCrossed = 0;
        hasReachedTube = false;
        isFirstIteration = false;
      }
      if(intsCrossed < intsFromReactor) {
        robotState = DRIVING_TO_INTERSECTION;
      } else if(!hasTurned) {
        operationState = TURNING_LEFT_90;
        isFirstTurn = true;
      } else {
        robotState = DRIVING_TO_TUBE;
        if(isAtTube() || hasReachedTube) {
          operationState = INSERTING_HORIZONTAL_ROD;
          isFirstIteration = true;
        }
      }
      break;
    case TURNING_LEFT_90:
      if(isFirstIteration) {
        hasLeftLine = false;
        hasReachedLine = false;
        hasReachedTurn = false;
        clearEncoders();
        isFirstIteration = false;
      }
      if(isSecondIteration) {
        hasTurned = false;
      }
      if(!hasReachedTurn) {
        robotState = DRIVING_PAST_LINE;
      } else if(!hasLeftLine) {
        robotState = STARTING_LEFT_TURN;
      } else if(!hasReachedLine) {
        robotState = TURNING_LEFT;
      } else if(isSecondIteration) {
        operationState = DRIVING_TO_SUPPLY;
        hasTurned = true;
      }
      if(!isFromSupply){
        operationState = DRIVING_TO_STORAGE;
        hasTurned = true;
      } else {
        operationState = DRIVING_TO_REACTOR;
        hasTurned = true;
      }
      break;
    case INSERTING_HORIZONTAL_ROD:
      if(isFirstIteration) {
        gripperInPosition = false;
        isFirstIteration = false;
      }
      if(!gripperInPosition) {
        robotState = GRIPPER_TO_MIDPOINT;
      } else if(hasRod){
        robotState = OPENING_GRIPPER;
        delay(500);
        hasRod = false;
        isLowRadiation = false;
      } else if(!(getArmPos() == armUp)) {
        robotState = GRIPPER_TO_TOP;
      } else {
        operationState = TURNING_180_FROM_TUBE;
        isFirstIteration = true;
      }
      break;
    case DRIVING_TO_SUPPLY:
      if(isFirstIteration) {
        intsCrossed = 0;
        hasReachedTube = false;
        isFirstIteration = false;
      }
      if(!hasTurned) {
        if(intsCrossed < 1) {
          robotState = DRIVING_TO_INTERSECTION;
        } else if(turnFromStorage == -1) {
          operationState = TURNING_LEFT_90;
          isFirstIteration = true;
        } else if(turnFromStorage == 1) {
          operationState = TURNING_RIGHT_90;
          isFirstIteration = true; 
        } else {
          hasTurned = true;
        }
      } else {
         if(intsCrossed < intsFromStorage) {
          robotState = DRIVING_TO_INTERSECTION;
         } else {
          if(turnFromStorage == -1) {
            operationState = TURNING_RIGHT_90;
            isFirstIteration = true;
            isSecondIteration = true;
          } else if(turnFromStorage = 1) {
            operationState = TURNING_LEFT_90;
            isFirstIteration = true;
            isSecondIteration = true;
          } else {
            if(isAtTube() || hasReachedTube) {
              operationState = REMOVING_HORIZONTAL_ROD;
              isFromSupply = true;
              isFirstIteration = true;
            } else {
              robotState = DRIVING_TO_TUBE;
            }
          }
         }
      }
      break;
    case TURNING_RIGHT_90:
      if(isFirstIteration) {
          hasLeftLine = false;
          hasReachedLine = false;
          hasReachedTurn = false;
          clearEncoders();
          isFirstIteration = false;
        }
      if(isSecondIteration) {
          hasTurned = false;
        }
      if(!hasReachedTurn) {
        robotState = DRIVING_PAST_LINE;
      } else if(!hasLeftLine) {
        robotState = STARTING_RIGHT_TURN;
      } else if(!hasReachedLine) {
        robotState = TURNING_RIGHT;
      } else if(isSecondIteration) {
        operationState = DRIVING_TO_SUPPLY;
        hasTurned = true;
      } else if(!hasRod){
        operationState = DRIVING_TO_SUPPLY;
        hasTurned = true;
      } else {
        operationState = DRIVING_TO_REACTOR;
        hasTurned = true;
      }
      break;
    case REMOVING_HORIZONTAL_ROD:
      if(isFirstIteration) {
       gripperInPosition = false; 
      }
      if(!gripperInPosition) {
        robotState = GRIPPER_TO_MIDPOINT;
      } else {
        gripperInPosition = true;
        if(!hasRod) {
          robotState = CLOSING_GRIPPER;
          delay(500);
          hasRod = true;
          isHighRadiation = true;
        } else {
          robotState = GRIPPER_TO_TOP;
          if(getArmPos() == armUp) {
            if(reGrip) {
              operationState = REMOVING_HORIZONTAL_ROD;
              reGrip = false;
            } else {
              operationState = TURNING_180_FROM_TUBE;
              isFirstIteration = true;
              isSecondVisit = true;
            }
          }
        }
      }
      break;
    case INSERTING_VERTICAL_ROD:
      if(isFirstIteration) {
       gripperInPosition = false; 
      }
      if(!gripperInPosition) {
        robotState = GRIPPER_TO_BOTTOM;
      } else {
        gripperInPosition = true;
        if(hasRod) {
          robotState = OPENING_GRIPPER;
          delay(500);
          hasRod = false;
          isHighRadiation = false;
        } else {
          robotState = GRIPPER_TO_TOP;
          if(getArmPos() == armUp) {
          operationState = OPERATION_STOPPED;
          }
        }
      }
      break;
    case OPERATION_STOPPED:
      robotState = STOPPED;
      break;
  }
}

void sendHeartbeat() {
  if (millis() > timeForHeartbeat) {
    timeForHeartbeat = millis() + 1000;
    msg.sendHeartbeat();
  }
}

void readLineSensor() {
  lineSensor.read(lineSensorReading);
}

unsigned int getLinePosition() {
  return lineSensor.readLine(lineSensorReading);
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
  double adjOutput = (armOutput + 255) * 180 / 512;
  moveArm(adjOutput);
}

unsigned int getArmPos() {
  return analogRead(armPotPin);
}

void moveArm(double speed) {
 arm.write((speed + 1) * 90);
}

void lineFollow(double speed) {
  lineSetpoint = 2500;
  lineInput = getLinePosition();
  linePID.Compute();
  double turnValue = lineOutput / 255;
  drive(speed, turnValue);
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

boolean isOnLine() {
  if(getLinePosition() != 0 || getLinePosition() != 5000) {
    return true;
  } else {
    return false;
  }
}

void turnOffLine(double speed, int direction) {
  if(isOnLine()) {
    drive(speed, direction);
  } else {
    stopRobot();
    hasLeftLine = true;
  }
}

void turnToLine(double speed, int direction) {
  if(isOnLine()) {
    stopRobot();
    hasReachedLine = true;
  } else {
    drive(speed, direction);
  }
}

void driveDistance(double speed, double distance) {
  distInput = getDistance();
  distSetpoint = distance;
  distPID.Compute();
  double rawSpeed = distOutput / 255;
  drive(speed * rawSpeed, 0);
}

double getDistance() {
  double avgCounts = (readLeftEnc() + readRightEnc()) / 2;
  return avgCounts * countsPerRev * wheelCircumfrence;
}

boolean hasDrivenDistance() {
  return getDistance() == distanceToDrive;
}

void clearEncoders() {
  leftEnc.write(0);
  rightEnc.write(0);
}

unsigned int readLeftEnc() {
  return leftEnc.read();
}

unsigned int readRightEnc() {
  return rightEnc.read();
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
  return !digitalRead(tubeSwitchPin);
}

void openGripper() {
  gripper.write(gripperOpen);
}

void closeGripper() {
  gripper.write(gripperClosed);
}

