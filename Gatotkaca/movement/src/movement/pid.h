#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <Wire.h>

// Definisi Alamat MPU6050
#define MPU6050_ADDR         0x68
#define MPU_REG_WHO_AM_I     0x75
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CFG     0x1B
#define MPU_REG_ACCEL_CFG    0x1C
#define MPU_REG_ACCEL_XOUT   0x3B

// Konstanta Konversi & Kalibrasi
#define ACCEL_SCALE          16384.0f
#define GYRO_SCALE           131.0f
#define CALIB_SAMPLES        200

// Struktur data untuk Kalman Filter
struct KalmanState {
    float angle = 0.0f;
    float bias = 0.0f;
    float P[2][2] = {{0, 0}, {0, 0}};
    float Q_angle = 0.001f;
    float Q_bias = 0.003f;
    float R_measure = 0.03f;
};

class PID {
public:
    // Constructor
    PID(float p, float i, float d, float minVal, float maxVal);

    // Fungsi Utama PID Standar
    float calculate(float target, float current);
    void reset();

    // Fungsi Gyro & IMU
    bool gyroBegin();
    bool gyroUpdate();
    float getHeadingError() const;
    bool isTilted(float threshold) const;

    // Fungsi Koreksi Tingkat Lanjut
    float getTiltCorrection(float targetPitch, float targetRoll, float corrLimit);
    float getHeadingCorrection(float corrLimit);

    // Variabel Sudut (Public agar bisa dibaca di main sketch)
    float gyroPitch, gyroRoll, gyroYaw;
    float targetHeading = 0.0f;

private:
    // Parameter PID Utama
    float kp, ki, kd;
    float outMin, outMax;
    float alpha;
    float integralLimit;
    
    // State PID Utama
    float integral, prevError, prevComponent;
    float lastFilteredDerivative;
    unsigned long lastTime;

    // State Internal untuk Pitch, Roll, Yaw
    float _pitchIntegral, _pitchPrevErr, _pitchPrevComp, _pitchLastFD;
    unsigned long _pitchLastT = 0;
    
    float _rollIntegral, _rollPrevErr, _rollPrevComp, _rollLastFD;
    unsigned long _rollLastT = 0;
    
    float _yawIntegral, _yawPrevErr, _yawPrevComp, _yawLastFD;
    unsigned long _yawLastT = 0;

    // State Kalman & Sensor
    KalmanState _kPitch, _kRoll;
    bool _gyroConnected = false;
    bool _gyroFirstUpdate = true;
    unsigned long gyroLastMicros;

    // Offset Kalibrasi
    float offAx, offAy, offAz, offGx, offGy, offGz;

    // Helper Functions Internal
    float _pidCalc(float target, float current, float p, float i, float d, 
                  float limit, float& integ, float& prevErr, float& prevComp, 
                  float& lastFD, unsigned long& lastT);
    
    float kalmanUpdate(KalmanState& s, float measAngle, float gyroRate, float dt);
    bool mpuWrite(uint8_t reg, uint8_t val);
    bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t len);
    bool readImu(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
};

#endif