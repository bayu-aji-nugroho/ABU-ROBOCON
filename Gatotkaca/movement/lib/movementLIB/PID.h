#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>


class MyPID {
public:
    // Constructor
    MyPID(float p, float i, float d, float outMin, float outMax);

    // FIX #4: Tambah destructor untuk bebaskan memori
    ~MyPID();

    // Fungsi Utama
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