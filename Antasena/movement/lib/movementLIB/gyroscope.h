#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Arduino.h>
#include <Wire.h>
#include "PID.h"


#include <MPU6050_6Axis_MotionApps20.h>

class Mpu {
private:
    int         SDA_PIN, SCL_PIN;
    MPU6050     mpu;

    
    bool        dmpReady   = false;
    bool        Gyro       = false; 
    uint16_t    packetSize = 0;
    uint16_t    fifoCount  = 0;
    uint8_t     fifoBuffer[64];


    
    Quaternion    q;
    VectorFloat   gravity;
    float         ypr[3];   // [yaw, pitch, roll] dalam radian

    // dalam derajat
    float   yaw   = 0.0f;
    float   pitch = 0.0f;
    float   roll  = 0.0f;

    float   yawOffset = 0.0f;
    MyPID*  headingPID = nullptr;

   
    float         driftRate       = 0.0f;   
    unsigned long driftStartMs    = 0;
    float         driftStartYaw   = 0.0f;
    unsigned long initMs          = 0;     

public:
    Mpu(int sda, int scl, float Kp, float Ki, float Kd);
    ~Mpu();

    void  init();
    void  update();   

    float outputYaw()   const { return yaw;   }
    float outputPitch() const { return pitch; }
    float outputRoll()  const { return roll;  }


    float getTilt()     const { return max(fabsf(pitch), fabsf(roll)); }

    void  setheading(float h) { yawOffset = h; initMs    = millis();}
    float CorectionHeading();
    bool  isReady()     const { return dmpReady; }
    bool  isGyroReady() {return Gyro; }

    void  measureDrift(uint16_t durationMs = 3000);
};

#endif