#include "PID.h"
#include <Arduino.h>


MyPID::MyPID(float kp, float ki, float kd, float outMin, float outMax)
    : kp(kp), ki(ki), kd(kd),
      outMin(outMin), outMax(outMax),
      input(0.0), output(0.0), setpoint(0.0),
      tuning(false)
{
    pid = new PID(&input, &output, &setpoint,
                  (double)kp, (double)ki, (double)kd, P_ON_E, DIRECT);

    if (!pid) {
        Serial.printf("[FATAL] PID alloc gagal! Heap: %d\n", ESP.getFreeHeap());
        ESP.restart();
    }

    pid->SetOutputLimits((double)outMin, (double)outMax);
    pid->SetSampleTime(1);  
    pid->SetMode(AUTOMATIC);

}

    
MyPID::~MyPID() {
    delete pid;  
}


float MyPID::calculate(float target, float current) {
    
    setpoint = (double)target;
    input    = (double)current;

    pid->Compute();
    return (float)output;
}


void MyPID::reset() {
    output = setpoint = input = 0.0;
    if (!pid) return;
    pid->SetMode(MANUAL);
    pid->SetMode(AUTOMATIC);
}


void MyPID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    if (pid) pid->SetTunings((double)kp, (double)ki, (double)kd);
}