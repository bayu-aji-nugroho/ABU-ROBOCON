#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Arduino.h>
#include <Wire.h>

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
    // sdaPin / sclPin: pin I2C yang dipakai (default ESP32: 21/22)
    Gyroscope(int sdaPin = 21, int sclPin = 22);

    // Inisialisasi MPU6050 + kalibrasi offset
    // Return false jika sensor tidak terdeteksi
    bool begin();

    // Harus dipanggil setiap loop() agar sudut selalu update
    bool update();

    // ── Getter sudut ─────────────────────────────────────────────────────────
    float getPitch()   const { return _pitch; }
    float getRoll()    const { return _roll;  }
    float getHeading() const { return _yaw;   }

    // Reset yaw ke 0 (gunakan saat robot pertama kali dinyalakan)
    void resetHeading();

    // Set heading target untuk koreksi yaw
    void setTargetHeading(float deg) { _targetHeading = deg; }
    float getTargetHeading()   const { return _targetHeading; }

    // ── Deteksi & Koreksi ────────────────────────────────────────────────────

    // Return true jika pitch atau roll melebihi threshold (derajat)
    bool isTilted(float threshold) const;

    // Koreksi PWM untuk menjaga robot tetap datar
    // targetPitch / targetRoll dalam derajat, corrLimit = batas output PWM
    float getTiltCorrection(float targetPitch, float targetRoll, float corrLimit);

    // Koreksi PWM untuk menjaga heading
    float getHeadingCorrection(float corrLimit);

    bool isConnected() const { return _connected; }

private:
    int _sdaPin, _sclPin;

    // Sudut hasil filter
    float _pitch = 0.0f;
    float _roll  = 0.0f;
    float _yaw   = 0.0f;

    float _targetHeading = 0.0f;

    bool          _connected    = false;
    bool          _firstUpdate  = true;
    unsigned long _lastMicros   = 0;

    // Kalman filter untuk pitch dan roll
    KalmanState _kPitch, _kRoll;

    // Offset kalibrasi
    float _offAx, _offAy, _offAz;
    float _offGx, _offGy, _offGz;

    // State PID internal untuk tilt & heading correction
    float _alpha          = 0.25f;
    float _integralLimit  = 150.0f;

    // Pitch PID state
    float _pitchInteg = 0, _pitchPrevErr = 0, _pitchPrevComp = 0, _pitchLastFD = 0;
    unsigned long _pitchLastT = 0;

    // Roll PID state
    float _rollInteg = 0, _rollPrevErr = 0, _rollPrevComp = 0, _rollLastFD = 0;
    unsigned long _rollLastT = 0;

    // Yaw PID state
    float _yawInteg = 0, _yawPrevErr = 0, _yawPrevComp = 0, _yawLastFD = 0;
    unsigned long _yawLastT = 0;

    // ── Helper ───────────────────────────────────────────────────────────────
    float _pidCalc(float target, float current,
                   float p, float i, float d, float limit,
                   float& integ, float& prevErr, float& prevComp,
                   float& lastFD, unsigned long& lastT);

    float _kalmanUpdate(KalmanState& s, float measAngle, float gyroRate, float dt);

    bool  _mpuWrite(uint8_t reg, uint8_t val);
    bool  _mpuRead(uint8_t reg, uint8_t* buf, uint8_t len);
    bool  _readImu(float& ax, float& ay, float& az,
                   float& gx, float& gy, float& gz);
};

#endif // GYROSCOPE_H