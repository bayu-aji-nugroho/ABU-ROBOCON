#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Arduino.h>
#include <Wire.h>
#include "PID.h"

// ── MPU6050 Register Map ─────────────────────────────────────────────────────
#define MPU6050_ADDR        0x68
#define MPU_REG_WHO_AM_I    0x75
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_REG_SMPLRT_DIV  0x19
#define MPU_REG_CONFIG      0x1A
#define MPU_REG_GYRO_CFG    0x1B
#define MPU_REG_ACCEL_CFG   0x1C
#define MPU_REG_ACCEL_XOUT  0x3B

// ── Skala & Kalibrasi ────────────────────────────────────────────────────────
#define ACCEL_SCALE         16384.0f   // ±2g
#define GYRO_SCALE          131.0f     // ±250°/s
#define CALIB_SAMPLES       200

// ── Limit koreksi default ────────────────────────────────────────────────────
#define TILT_CORR_LIMIT     30.0f
#define HEADING_CORR_LIMIT  40.0f

// ── Kalman Filter State ──────────────────────────────────────────────────────
struct KalmanState {
    float angle     = 0.0f;
    float bias      = 0.0f;
    float P[2][2]   = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    float Q_angle   = 0.001f;
    float Q_bias    = 0.003f;
    float R_measure = 0.03f;
};


class Gyroscope {
public:
    Gyroscope(int sdaPin = 21, int sclPin = 22);
    ~Gyroscope();

    bool begin();
    bool update();

    // ── Getter sudut ─────────────────────────────────────────────────────────
    float getPitch()   const { return _pitch; }
    float getRoll()    const { return _roll;  }
    float getHeading() const { return _yaw;   }

    void  resetHeading();
    void  setTargetHeading(float deg) { _targetHeading = deg; }
    float getTargetHeading()    const { return _targetHeading; }

    // ── Deteksi & Koreksi ────────────────────────────────────────────────────
    bool  isTilted(float threshold) const;
    float getTiltCorrection(float targetPitch, float targetRoll, float corrLimit);
    float getHeadingCorrection(float corrLimit);

    bool isConnected() const { return _connected; }

private:
    int _sdaPin, _sclPin;

    float _pitch = 0.0f;
    float _roll  = 0.0f;
    float _yaw   = 0.0f;

    float _targetHeading = 0.0f;

    bool          _connected   = false;
    bool          _firstUpdate = true;
    unsigned long _lastMicros  = 0;

    KalmanState _kPitch, _kRoll;

  
    float _offAx = 0.0f, _offAy = 0.0f, _offAz = 0.0f;
    float _offGx = 0.0f, _offGy = 0.0f, _offGz = 0.0f;

    // ── PID pakai MyPID ──────────────────────────────────────────────────────
    // Dibuat di begin() setelah limit diketahui, dihapus di destruktor
    MyPID* _pidPitch   = nullptr;
    MyPID* _pidRoll    = nullptr;
    MyPID* _pidYaw     = nullptr;

    // ── Helper ───────────────────────────────────────────────────────────────
    float _kalmanUpdate(KalmanState& s, float measAngle, float gyroRate, float dt);
    bool  _mpuWrite(uint8_t reg, uint8_t val);
    bool  _mpuRead(uint8_t reg, uint8_t* buf, uint8_t len);
    bool  _readImu(float& ax, float& ay, float& az,
                   float& gx, float& gy, float& gz);
};

#endif // GYROSCOPE_H