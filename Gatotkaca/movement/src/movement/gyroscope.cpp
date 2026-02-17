#include "gyroscope.h"
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
Gyroscope::Gyroscope(int sdaPin, int sclPin)
    : _sdaPin(sdaPin), _sclPin(sclPin) {}

// ─────────────────────────────────────────────────────────────────────────────
bool Gyroscope::begin() {
    Wire.begin(_sdaPin, _sclPin);
    Wire.setClock(400000);

    // Cek WHO_AM_I
    uint8_t who;
    if (!_mpuRead(MPU_REG_WHO_AM_I, &who, 1)) {
        Serial.println(F("[Gyroscope] ERROR: Tidak bisa baca register WHO_AM_I"));
        return false;
    }
    if (who != 0x68 && who != 0x72) {
        Serial.print(F("[Gyroscope] ERROR: WHO_AM_I tidak cocok = 0x"));
        Serial.println(who, HEX);
        return false;
    }

    // Wake up, gunakan gyro X sebagai clock source
    if (!_mpuWrite(MPU_REG_PWR_MGMT_1, 0x01)) return false;
    delay(100); 

    _mpuWrite(MPU_REG_SMPLRT_DIV, 0x07); // Sample rate 125 Hz
    _mpuWrite(MPU_REG_CONFIG,     0x03); // DLPF ~44 Hz (kurangi noise)
    _mpuWrite(MPU_REG_GYRO_CFG,   0x00); // ±250°/s
    _mpuWrite(MPU_REG_ACCEL_CFG,  0x00); // ±2g
    delay(50);

    // ── Kalibrasi offset ─────────────────────────────────────────────────────
    Serial.println(F("[Gyroscope] Kalibrasi... jangan gerakkan robot!"));
    long sAx=0, sAy=0, sAz=0, sGx=0, sGy=0, sGz=0;
    float ax, ay, az, gx, gy, gz;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        _readImu(ax, ay, az, gx, gy, gz);
        sAx += (long)(ax * ACCEL_SCALE);
        sAy += (long)(ay * ACCEL_SCALE);
        sAz += (long)(az * ACCEL_SCALE);
        sGx += (long)(gx * GYRO_SCALE);
        sGy += (long)(gy * GYRO_SCALE);
        sGz += (long)(gz * GYRO_SCALE);
        delay(2);
    }

    _offAx =  (float)sAx / CALIB_SAMPLES / ACCEL_SCALE;
    _offAy =  (float)sAy / CALIB_SAMPLES / ACCEL_SCALE;
    _offAz =  (float)sAz / CALIB_SAMPLES / ACCEL_SCALE - 1.0f; // kurangi 1g
    _offGx =  (float)sGx / CALIB_SAMPLES / GYRO_SCALE;
    _offGy =  (float)sGy / CALIB_SAMPLES / GYRO_SCALE;
    _offGz =  (float)sGz / CALIB_SAMPLES / GYRO_SCALE;

    Serial.println(F("[Gyroscope] Kalibrasi selesai, siap dipakai!"));

    _connected   = true;
    _firstUpdate = true;
    _lastMicros  = micros();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
bool Gyroscope::update() {
    if (!_connected) return false;

    float ax, ay, az, gx, gy, gz;
    if (!_readImu(ax, ay, az, gx, gy, gz)) return false;

    unsigned long now = micros();
    float dt = (now - _lastMicros) / 1e6f;
    _lastMicros = now;

    if (dt <= 0 || dt > 0.5f) dt = 0.01f;

    // Sudut dari akselerometer
    float accelPitch = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG;
    float accelRoll  = atan2f(-ax, az)                  * RAD_TO_DEG;

    if (_firstUpdate) {
        _kPitch.angle = accelPitch;
        _kRoll.angle  = accelRoll;
        _pitch        = accelPitch;
        _roll         = accelRoll;
        _yaw          = 0.0f;
        _firstUpdate  = false;
        return true;
    }

    // Kalman filter untuk pitch dan roll
    _pitch = _kalmanUpdate(_kPitch, accelPitch, gy, dt);
    _roll  = _kalmanUpdate(_kRoll,  accelRoll,  gx, dt);

    // Yaw = integrasi gyro (akselerometer tidak bisa ukur yaw)
    _yaw += gz * dt;
    if (_yaw >  180.0f) _yaw -= 360.0f;
    if (_yaw < -180.0f) _yaw += 360.0f;

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
void Gyroscope::resetHeading() {
    _yaw           = 0.0f;
    _targetHeading = 0.0f;
    // Reset state PID yaw agar tidak ada lonjakan koreksi
    _yawInteg    = 0;
    _yawPrevErr  = 0;
    _yawPrevComp = 0;
    _yawLastFD   = 0;
    _yawLastT    = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
bool Gyroscope::isTilted(float threshold) const {
    return (fabsf(_pitch) > threshold || fabsf(_roll) > threshold);
}

// ─────────────────────────────────────────────────────────────────────────────
float Gyroscope::getTiltCorrection(float targetPitch, float targetRoll, float corrLimit) {
    float pc = _pidCalc(targetPitch, _pitch,
                        3.0f, 0.02f, 1.2f, corrLimit,
                        _pitchInteg, _pitchPrevErr, _pitchPrevComp, _pitchLastFD, _pitchLastT);

    float rc = _pidCalc(targetRoll, _roll,
                        3.0f, 0.02f, 1.2f, corrLimit,
                        _rollInteg, _rollPrevErr, _rollPrevComp, _rollLastFD, _rollLastT);

    return constrain(pc + rc, -corrLimit, corrLimit);
}

// ─────────────────────────────────────────────────────────────────────────────
float Gyroscope::getHeadingCorrection(float corrLimit) {
    // Hitung error heading dengan wrap-around ±180°
    float err = _targetHeading - _yaw;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    return _pidCalc(0.0f, -err,
                    2.5f, 0.05f, 0.8f, corrLimit,
                    _yawInteg, _yawPrevErr, _yawPrevComp, _yawLastFD, _yawLastT);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Internal PID helper (dipakai untuk tilt & heading correction)
// ─────────────────────────────────────────────────────────────────────────────
float Gyroscope::_pidCalc(float target, float current,
                           float p, float i, float d, float limit,
                           float& integ, float& prevErr, float& prevComp,
                           float& lastFD, unsigned long& lastT) {
    unsigned long now = micros();
    if (lastT == 0) {
        lastT    = now;
        prevErr  = target - current;
        prevComp = current;
        lastFD   = 0;
        return 0;
    }

    float dt = (now - lastT) / 1000.0f; // ms → lebih stabil untuk integral kecil
    if (dt <= 0) return 0;
    lastT = now;

    float error = target - current;
    integ += error * dt;
    integ  = constrain(integ, -_integralLimit, _integralLimit);

    float raw      = -(current - prevComp) / dt;
    float filtered = (_alpha * raw) + ((1.0f - _alpha) * lastFD);

    prevErr  = error;
    prevComp = current;
    lastFD   = filtered;

    float out = (error * p) + (integ * i) + (filtered * d);
    return constrain(out, -limit, limit);
}

// ─────────────────────────────────────────────────────────────────────────────
float Gyroscope::_kalmanUpdate(KalmanState& s, float measAngle,
                                float gyroRate, float dt) {
    // Predict
    float rate  = gyroRate - s.bias;
    s.angle    += dt * rate;
    s.P[0][0]  += dt * (dt*s.P[1][1] - s.P[0][1] - s.P[1][0] + s.Q_angle);
    s.P[0][1]  -= dt * s.P[1][1];
    s.P[1][0]  -= dt * s.P[1][1];
    s.P[1][1]  += s.Q_bias * dt;

    // Update
    float S  = s.P[0][0] + s.R_measure;
    float K0 = s.P[0][0] / S;
    float K1 = s.P[1][0] / S;
    float y  = measAngle - s.angle;
    s.angle += K0 * y;
    s.bias  += K1 * y;
    float P00 = s.P[0][0], P01 = s.P[0][1];
    s.P[0][0] -= K0 * P00;
    s.P[0][1] -= K0 * P01;
    s.P[1][0] -= K1 * P00;
    s.P[1][1] -= K1 * P01;

    return s.angle;
}

// ─────────────────────────────────────────────────────────────────────────────
bool Gyroscope::_mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool Gyroscope::_mpuRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU6050_ADDR, len);
    uint8_t i = 0;
    while (Wire.available() && i < len) buf[i++] = Wire.read();
    return (i == len);
}

bool Gyroscope::_readImu(float& ax, float& ay, float& az,
                          float& gx, float& gy, float& gz) {
    uint8_t buf[14];
    if (!_mpuRead(MPU_REG_ACCEL_XOUT, buf, 14)) return false;

    ax = (int16_t)(buf[0]<<8  | buf[1])  / ACCEL_SCALE - _offAx;
    ay = (int16_t)(buf[2]<<8  | buf[3])  / ACCEL_SCALE - _offAy;
    az = (int16_t)(buf[4]<<8  | buf[5])  / ACCEL_SCALE - _offAz;
    gx = (int16_t)(buf[8]<<8  | buf[9])  / GYRO_SCALE  - _offGx;
    gy = (int16_t)(buf[10]<<8 | buf[11]) / GYRO_SCALE  - _offGy;
    gz = (int16_t)(buf[12]<<8 | buf[13]) / GYRO_SCALE  - _offGz;
    return true;
}