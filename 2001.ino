#include "Arduino.h"
#include "Wire.h"
#include "Messages.h"
#include "QTRSensors.h"
#include "Encoder.h"
#include "Servo.h"
#include "Arm.h"
#include "Drive.h"

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


//line sensor pin of sorts?
const int emitterPin = 35;
const unsigned char lineSensorPins[numSensors] = {23, 25, 27, 29, 30, 33}; //why tf are these chars???

//Servo
int gripperPin = 5;
Servo gripper;
Servo arm;

//Encoders
Encoder leftEnc(leftEncPinA, leftEncPinB);
Encoder rightEnc(rightEncPinA, rightEncPinB);;


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
  /*
  Serial.println("Starting");
  Wire.begin();
  gyro.enableDefault();
  msg.setup();
  timeForHeartbeat = millis() + 1000;
  gripper.attach(gripperPin, 1000, 2000);
  arm.attach(armPin, 1000, 2000);
  robotState = STOPPED;
  operationState = CALIBRATING_LINE_SENSOR;
  */

  

  

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
          doDrive(globalSpeed, 0);
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
      doTurn(90, globalSpeed);
      break;
    case TURNING_RIGHT_90:
      doTurn(-90, globalSpeed);
      break;
    case TURNING_180:
      doTurn(180, globalSpeed);
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

