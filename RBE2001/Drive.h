#ifndef _Drive
#define _Drive

class Drive {
  public:
    Drive();  
    void doTurn();
    void doDrive();
    
  private:
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

    //Gyro
    //SDA is 20 SCL is 21
    L3G gyro;
    
    //Line Sensor
    const int numSensors = 6;
    unsigned int lineSensorReading[numSensors + 1];
    unsigned int linePosition;
    unsigned int threshold = 0;
    QTRSensorsRC lineSensor(lineSensorPins, numSensors, 2500, emitterPin);

    double readAngle(void);
    
}

#endif
