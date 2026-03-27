#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <PID_v1.h>


class MyPID {
public:
    
    MyPID(float p, float i, float d, float outMin, float outMax);

    
    ~MyPID();

    float calculate(float target, float current);
    void reset();
    void setTunings(float kp, float ki, float kd);

private:
    PID* pid;
   

    float kp, ki, kd;
    float outMin, outMax;
    bool tuning;

    double input;
    double output;
    double setpoint;
};

#endif