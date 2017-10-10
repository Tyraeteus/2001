#include "Arm.h"

Arm::Arm(){
  
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


void openGripper() {
  gripper.write(gripperOpen);
}

void closeGripper() {
  gripper.write(gripperClosed);
}

