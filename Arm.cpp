#include "Arm.h"

Arm::Arm(){
  
}

void Arm::moveArmToPos(int position) {
  armSetpoint = position;
  armInput = getArmPos();
  armPID.Compute();
  moveArm(armOutput);
}

unsigned int Arm::getArmPos() {
  return analogRead(armPotPin);
}

void Arm::moveArm(double speed) {
 int dutyCycle = abs(speed) * 255;
  if(speed > 0) {
    analogWrite(armFow, dutyCycle);
    digitalWrite(armRev, LOW);
  } else {
    digitalWrite(armFow, LOW);
    analogWrite(armRev, dutyCycle);
  }
}


void Arm::openGripper() {
  gripper.write(gripperOpen);
}

void Arm::closeGripper() {
  gripper.write(gripperClosed);
}

