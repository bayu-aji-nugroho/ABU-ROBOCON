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
        float nilai_PWM_ke_roda;

        static bool   _gyroReady;        // sudah begin() atau belum
        static float  _pitch;            // sudut maju-mundur (°)
        static float  _roll;             // sudut kiri-kanan (°)
        static float  _yaw;              // heading saat ini  (°)
        static float  _targetHeading;    // heading yang ingin dipertahankan

        // Kalman state (pitch & roll)
        static float  _kP_angle, _kP_bias, _kP_P00, _kP_P01, _kP_P10, _kP_P11;
        static float  _kR_angle, _kR_bias, _kR_P00, _kR_P01, _kR_P10, _kR_P11;

        // Offset kalibrasi
        static float  _offAx, _offAy, _offAz;
        static float  _offGx, _offGy, _offGz;

        // Waktu update terakhir gyro
        static unsigned long _gyroLastMicros;
        static bool          _gyroFirstUpdate;

        // ── Helper internal (static) ──
        static bool  _mpuWrite(uint8_t reg, uint8_t val);
        static bool  _mpuRead (uint8_t reg, uint8_t* buf, uint8_t len);
        static bool  _readImu (float& ax, float& ay, float& az,
                               float& gx, float& gy, float& gz);
        static float _kalmanUpdate(
                        float& angle, float& bias,
                        float& P00, float& P01, float& P10, float& P11,
                        float measAngle, float gyroRate, float dt);
    public:
        Movement(
            float Kp, float Ki, float Kd, int chanelA, int ChanelB,
              float ppr,int RPWM, int LPWM);
        void update(float target);
        void begin();
        void resetPID();

        static bool  gyroBegin();          // init I2C + MPU + kalibrasi
        static bool  gyroUpdate();         // baca sensor, update sudut — panggil tiap loop()

        static float getPitch()   { return _pitch; }
        static float getRoll()    { return _roll;  }
        static float getHeading() { return _yaw;   }

        // Heading lock
        static void  setTargetHeading(float deg) { _targetHeading = deg; }
        static void  resetHeading() { _yaw = 0; _targetHeading = 0; }
        static float getHeadingError();    // error ±180°, sudah handle wrap

        // Koreksi siap pakai — langsung tambahkan ke PWM / mixing motor
        static float getTiltCorrection(float corrLimit = 80.0f);
        static float getHeadingCorrection(float corrLimit = 80.0f);

        // Safety: apakah robot miring melebihi threshold?
        static bool  isTilted(float threshold = 15.0f);

        static bool  gyroConnected() { return _gyroReady; }

};


#endif