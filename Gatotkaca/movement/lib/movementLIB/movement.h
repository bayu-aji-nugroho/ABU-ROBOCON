#ifndef MOVEMENT_H
#define MOVEMENT_H

//menggabungkan movement

#include <Arduino.h>
class Encoder;
class MyPID;

class Movement{
    private:
        Encoder* encoder;
        MyPID* pid;
        int RPWM,LPWM;
        
        float nilai_PWM_ke_roda;
       

    public:
        Movement(
            float Kp, float Ki, float Kd, int chanelA, int ChanelB,
              float ppr,int RPWM, int LPWM);
         ~Movement() { delete encoder; delete pid;}
        void update(float target);
        void begin();
        void resetPID();

       
};


#endif