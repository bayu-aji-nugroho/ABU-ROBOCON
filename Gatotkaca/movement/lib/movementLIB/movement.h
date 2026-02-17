#ifndef MOVEMENT_H
#define MOVEMENT_H

//menggabungkan movement

#include <Arduino.h>
class Encoder;
class PID;

class Movement{
    private:
        Encoder* encoder;
        PID* pid;
        int RPWM,LPWM;
        int SDA_PIN, SCL_PIN;
        float nilai_PWM_ke_roda;

    public:
        Movement(
            float Kp, float Ki, float Kd, int chanelA, int ChanelB,
              float ppr,int RPWM, int LPWM);
        void update(float target);
        void begin();
        void resetPID();

        bool gyroUpdate();                    
        float getPitch();                     
        float getRoll();                      
        float getHeading();                   
        void setTargetHeading(float deg);     
        void resetHeading();                  
        bool isTilted(float threshold = 15.0f); 
        float getTiltCorrection(float targetPitch, float targetRoll, float corrLimit);
        float getHeadingCorrection(float corrLimit);
};


#endif