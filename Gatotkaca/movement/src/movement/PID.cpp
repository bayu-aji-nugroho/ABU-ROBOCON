#include "PID.h"
#include <Arduino.h>


MyPID::MyPID(float kp, float ki, float kd, float outMin, float outMax)
    : kp(kp), ki(ki), kd(kd),
      outMin(outMin), outMax(outMax),
      input(0.0), output(0.0), setpoint(0.0),
      tuning(false)
{
    pid = new PID(&input, &output, &setpoint,
                  (double)kp, (double)ki, (double)kd, P_ON_M, DIRECT);

    pid->SetOutputLimits((double)outMin, (double)outMax);
    pid->SetSampleTime(10);

    pid->SetMode(AUTOMATIC); // ubah ke manual saat auto tuning

    aTune = new PID_ATune(&input, &output);

    aTune->SetOutputStep(10);     
    aTune->SetControlType(1);    
    aTune->SetNoiseBand(1.0);     
    aTune->SetLookbackSec(20);    
}



MyPID::~MyPID() {
    delete pid;
    delete aTune;
}


float MyPID::calculate(float target, float current) {
    setpoint = (double)target;
    input    = (double)current;

    if (tuning) { // jika ingin auto tuning def false
        byte val = aTune->Runtime();
        if (val != 0) {
            tuning = false;
            this->kp = aTune->GetKp();
            this->ki = aTune->GetKi();
            this->kd = aTune->GetKd();

            pid->SetTunings(this->kp, this->ki, this->kd);
            pid->SetMode(AUTOMATIC);
        }
    } else {
        pid->Compute();
    }

    return (float)output;
}


void MyPID::reset() {
    if (tuning) {
        tuning = false;
        aTune->Cancel();
    }
    output = setpoint = input = 0.0;
    pid->SetMode(MANUAL);
    pid->SetMode(AUTOMATIC);
}


void MyPID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    pid->SetTunings((double)kp, (double)ki, (double)kd);
}