#ifndef _Arm
#define _Arm

class Arm {
  public:
    Arm();

  private:
    //Arm PID controller
    double kPArm;
    double kIArm;
    double kDArm;
    double armSetpoint;
    double armInput;
    double armOutput;
    PID armPID(&armInput, &armOutput, &armSetpoint, kPArm, kIArm, kDArm, DIRECT);
}

#endif
