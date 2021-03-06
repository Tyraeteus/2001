/*
 * Messages.cpp
 *
 *  Created on: Sep 15, 2016
 *      Author: bradmiller
 */
#include "Arduino.h"
#include "Messages.h"
#include "BTComms.h"

BTComms comms;

/**
 * Constuctor
 * Initialize everything here when the class is created
 * Note: you cannot call methods that depend on other classes having already been created
 */
Messages::Messages() {
  stopped = false;
}

/**
 * Setup class code that is called from the Arduino sketch setup() function. This doesn't
 * get called until all the other classes have been created.
 */
void Messages::setup() {
  comms.setup();
}

/*
 * Method to get the number of the first available supply tube
 * In this case, we are looking for the first full tube
 */
int Messages::getFirstSupply() {
  int count = 0;
  boolean tubeAvailable = false;
  byte mask = B1000;
  byte availableTubes = supplyAvailability;
  while(count < 4 && !tubeAvailable) {
    //check if first tube is occupied
    if(availableTubes & mask) {
      //if so, set flag and return the current index, + 1 to help with route calculation
      tubeAvailable = true;
      return count + 1;
    //otherwise, perform a left shift to move the next index to checking position and advance the index
    } else {
      availableTubes <<= 1;
      count++;
    }
  }
  //if no tubes are availible, return an invalid tube
  if(count == 4 && tubeAvailable == false) {
    return -1;
  }
}

/*
 * Method to get the number of the first available storage tube
 * In this case, we are looking for the first empty tube
 */
int Messages::getFirstStorage() {
  int count = 0;
  boolean tubeAvailable = false;
  byte mask = B1000;
  byte availableTubes = storageAvailability;
  //check if first tube is occupied
  while(count < 4 && !tubeAvailable) {
    //if not, set flag and return the current index, + 1 to help with route calculation
    if(!(availableTubes & mask)) {
      tubeAvailable = true;
      return count + 1;
    //otherwise, perform a left shift to move the next index to checking position and advance the index
    } else {
      availableTubes <<= 1;
      count++;
    }
  }
  //if no tubes are available, return an invalid tube
  if(count == 4 && tubeAvailable == false) {
    return -1;
  }
}

/**
 * Check if the field is currently in the "stop" state
 * @returns bool value that is true if the robot should be stopped
 */
bool Messages::isStopped() {
  return stopped;
}

/**
 * Send a heartbeat message to the field to let it know that your code is alive
 * This should be called by your robot program periodically, say once per second. This
 * timing can easily be done in the loop() function of your program.
 */
void Messages::sendHeartbeat() {
  comms.writeMessage(kHeartbeat, 0x0d, 0x00);
}

/*
 * Method to send low level radation message
 */
void Messages::sendLowRadiation() {
  comms.writeMessage(kRadiationAlert, 0x0d, 0x2C);
}

/*
 * MEthod to send high level radation message
 */
void Messages::sendHighRadiation() {
  comms.writeMessage(kRadiationAlert, 0x0d, 0xFF);
}

/**
 * Print message for debugging
 * This method prints the message as a string of hex numbers strictly for debugging
 * purposes and is not required to be called for any other purpose.
 */
void Messages::printMessage() {
    for (int i = 0; i < comms.getMessageLength(); i++) {
      Serial.print(comms.getMessageByte(i), HEX);
      Serial.print(" ");
    }
    if(comms.getMessageByte(0) == 4) {
      Serial.println("Stop");
    } else if(comms.getMessageByte(0) == 5) {
      Serial.println("Resume");
    } else {
      Serial.println();
    }
    
}

/**
 * Read messages from the Bluetooth serial connection
 * This method should be called from the loop() function in your arduino code. It will check
 * to see if the lower level comms object has received a complete message, and run the appropriate
 * code to handle the message type. This should just save the state of the message inside this class
 * inside member variables. Then add getters/setters to retrieve the status from your program.
 */
bool Messages::read() {
  if (comms.read()) {
    switch (comms.getMessageByte(0)) {
    //read storage data
    case kStorageAvailability:
      storageAvailability = comms.getMessageByte(3);
      break;
    //read supply data
    case kSupplyAvailability:
      supplyAvailability = comms.getMessageByte(3);
      break;
    case kRadiationAlert:
      break;
    //set "stopped" flag so robot can stop
    case kStopMovement:
      stopped = true;
      break;
    //reset stopped flag
    case kResumeMovement:
      stopped = false;
      break;
    case kRobotStatus:
      break;
    case kHeartbeat:
      break;
    }
    return true;
  }
  return false;
}
