#include "BTComms.h"

#include "Arduino.h"

/**
 * Bluetooth communications constructor
 */
BTComms::BTComms() {
  messageIndex = 0;
  messageLength = 0;
  BTstate = kLookingForStart;
}

/**
 * Code that is called from the arduino setup() function
 * This initializes things that cannot be set up from the constructor.
 */
void BTComms::setup() {
  Serial3.begin(115200);
}

/**
 * Send a message to the RCS that has 3 values (opcode, source, dest)
 * This method sends messages via bluetooth to the field that have an opcode with
 * a source and destination address. It is used for the heartbeat message that has
 * no message data.
 *
 * You could add additional methods exactly like this one, that take a payload such
 * as a status message. You can create a new method that is exactly the same as this
 * one (also called writeMessage), but with an additional parameter that gets sent.
 * With C++ you can have multiple methods with the same name that are different by
 * the number of parameters they have. Be sure that the new function adjusts the length,
 * and writes the extra byte to the bluetooth interface and includes it in the checksum
 * calculation.
 */void BTComms::writeMessage(unsigned char opcode, unsigned char source, unsigned char dest) {
  Serial3.write(kMessageStart);
  Serial3.write(5);
  Serial3.write(opcode);
  Serial3.write(source);
  Serial3.write(dest);
  Serial3.write(0xff - (opcode + source + dest + 5));
}

/**
 * Get the length of the currently received message
 * @returns int The number of bytes in the received message
 */
int BTComms::getMessageLength() {
  return messageLength;
}

/**
 * Get a byte from the current message
 * Retrieve a byte from the currently received message. Only a single message is
 * remembered at any time, so you have to call read(), notice that there is a message,
 * and then do something with the message bytes.
 * @param index The offset (zero-based) to the byte to be returned
 * @returns unsigned char The byte that is at the specified index
 */
unsigned char BTComms::getMessageByte(unsigned index) {
  if (index >= messageLength) {
    Serial.print("request for message byte beyond end of message ");
    Serial.println(index);
    return 0;
  }
  return message[index];  
}

/**
 * Read a message from Bluetooth
 * This method reads messages from Bluetooth by looking for the message start byte, then
 * reading the message length and data.
 *
 * You should probably modify this to ignore messages with invalid checksums!
 */
bool BTComms::read() {
  int runningSum = 0;
  while (Serial3.available()) {
    unsigned inByte = Serial3.read();
    switch (BTstate) {
      case kLookingForStart:
        //Serial.println("Looking for Start");
        if (inByte != kMessageStart)
          break;
        BTstate = kReadingMessageLength;
        break;
      case kReadingMessageLength:
        //Serial.println("Reading Length");
        messageLength = inByte - 1;
        if (messageLength >= messageBufferLength) {
          Serial.println("Received message length greater than buffer size");
          BTstate = kLookingForStart;
          break;
        }
        messageIndex = 0;
        BTstate = kReadMessage;
        break;
      case kReadMessage:
        //Serial.println("Reading Message");
        message[messageIndex++] = inByte;
        if (messageIndex >= messageLength) {
          BTstate = kVerifyChecksum;
        }
        break;
       case kVerifyChecksum:
        //Serial.println("Verifying Checksum");
        unsigned char result;
        unsigned char verificationSum;
        for(int i = 0; i <= messageLength - 2; i++) {
          result = result + message[i];
        }
        verificationSum = 0xFF - result - messageLength - 1;
        if(verificationSum == message[messageLength - 1]) {
          BTstate = kVerifyDestination;
          //Serial.println("Checksum Pass");
        } else {
          BTstate = kLookingForStart;
          Serial.println("Checksum Fail");
          //Serial.println(verificationSum);
          //Serial.println(message[messageLength]);
        }
        break;
       case kVerifyDestination:
        //Serial.println("Verifying Destination");
        if(message[1] == 0x0D || message[1] == 0) {
          BTstate = kLookingForStart;
          //Serial.println("Destination Pass");
          return true;
        } else {
          BTstate = kLookingForStart;
          Serial.println("Destination Fail");
        }
        break;
       default:
        Serial.println("Invalid state");
    }
  }
  return false;
}


