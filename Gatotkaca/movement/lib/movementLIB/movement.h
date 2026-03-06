#ifndef MOVEMENT_H
#define MOVEMENT_H

//menggabungkan movement

#include <Arduino.h>
class Encoder;
class MyPID;


struct LogStruct {
    String& name;
    float Target;
    float Actual;
};

class Movement{
    private:
        Encoder* encoder;
        MyPID* pid;
        int RPWM,LPWM;
        String name;
        unsigned long lastDebug = 0;
        
        float nilai_PWM_ke_roda;
        float _targetRPM = 0.0f;
        float _actualRPM = 0.0f;
        float _count     = 0.0f;
       

    public:
        Movement(
            String name,
            float Kp, float Ki, float Kd, int chanelA, int ChanelB,
              float ppr,int RPWM, int LPWM);
         ~Movement();
        void update(float target);
        void begin();
        void resetPID();
        float getActualRPM() const { return _actualRPM; }
        float getTargetRPM() const { return _targetRPM; }
        float getTargetCount() const {return _count;}
        float getPWM() const { return nilai_PWM_ke_roda; }
        const String& getName()   const { return name; } 
        
        

       
};


#endif