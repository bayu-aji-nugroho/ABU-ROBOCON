#include "gyroscope.h"
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
Gyroscope::Gyroscope(int sdaPin, int sclPin)
    : _sdaPin(sdaPin), _sclPin(sclPin), _pidPitch(nullptr), _pidRoll(nullptr), _pidYaw(nullptr) {
    isInitialized = false;
    _connected = false;
}

// ─────────────────────────────────────────────────────────────────────────────
Gyroscope::~Gyroscope() {
    if (_pidPitch) delete _pidPitch;
    if (_pidRoll)  delete _pidRoll;
    if (_pidYaw)   delete _pidYaw;
}

// ─────────────────────────────────────────────────────────────────────────────
bool Gyroscope::begin() {
    // PENTING: Inisialisasi objek PID di awal agar pointer tidak NULL 
    // meskipun sensor gagal dideteksi. Ini mencegah "StoreProhibited" crash.
    if (!_pidPitch) _pidPitch = new MyPID(3.0f, 0.02f, 1.2f, -TILT_CORR_LIMIT,    TILT_CORR_LIMIT);
    if (!_pidRoll)  _pidRoll  = new MyPID(3.0f, 0.02f, 1.2f, -TILT_CORR_LIMIT,    TILT_CORR_LIMIT);
    if (!_pidYaw)   _pidYaw   = new MyPID(2.5f, 0.05f, 0.8f, -HEADING_CORR_LIMIT, HEADING_CORR_LIMIT);

    Wire.begin((int)_sdaPin, (int)_sclPin);
    Wire.setClock(400000);

    // Cek WHO_AM_I
    uint8_t who;
    if (!_mpuRead(MPU_REG_WHO_AM_I, &who, 1)) {
        Serial.println(F("[Gyroscope] ERROR: Tidak bisa baca register WHO_AM_I (Cek Kabel!)"));
        isInitialized = false;
        return false;
    }
    
    if (who != 0x68 && who != 0x72) {
        Serial.print(F("[Gyroscope] ERROR: WHO_AM_I tidak cocok = 0x"));
        Serial.println(who, HEX);
        isInitialized = false;
        return false;
    }

    // Konfigurasi MPU6050
    if (!_mpuWrite(MPU_REG_PWR_MGMT_1, 0x01)) return false;
    delay(100);

    _mpuWrite(MPU_REG_SMPLRT_DIV, 0x07); // Sample rate 125 Hz
    _mpuWrite(MPU_REG_CONFIG,     0x03); // DLPF ~44 Hz
    _mpuWrite(MPU_REG_GYRO_CFG,   0x00); // ±250°/s
    _mpuWrite(MPU_REG_ACCEL_CFG,  0x00); // ±2g
    delay(50);

    // ── Kalibrasi offset ─────────────────────────────────────────────────────
    Serial.println(F("[Gyroscope] Kalibrasi... jangan gerakkan robot!"));
    long sAx=0, sAy=0, sAz=0, sGx=0, sGy=0, sGz=0;
    float ax, ay, az, gx, gy, gz;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        if (!_readImu(ax, ay, az, gx, gy, gz)) {
             Serial.println(F("[Gyroscope] ERROR: Gagal baca data saat kalibrasi"));
             return false;
        }
        sAx += (long)(ax * ACCEL_SCALE);
        sAy += (long)(ay * ACCEL_SCALE);
        sAz += (long)(az * ACCEL_SCALE);
        sGx += (long)(gx * GYRO_SCALE);
        sGy += (long)(gy * GYRO_SCALE);
        sGz += (long)(gz * GYRO_SCALE);
        delay(2);
    }

    _offAx = (float)sAx / CALIB_SAMPLES / ACCEL_SCALE;
    _offAy = (float)sAy / CALIB_SAMPLES / ACCEL_SCALE;
    _offAz = (float)sAz / CALIB_SAMPLES / ACCEL_SCALE - 1.0f;
    _offGx = (float)sGx / CALIB_SAMPLES / GYRO_SCALE;
    _offGy = (float)sGy / CALIB_SAMPLES / GYRO_SCALE;
    _offGz = (float)sGz / CALIB_SAMPLES / GYRO_SCALE;

    Serial.println(F("[Gyroscope] Kalibrasi selesai, siap dipakai!"));

    isInitialized = true;
    _connected   = true;
    _firstUpdate = true;
    _lastMicros  = micros();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
bool Gyroscope::update() {
    if (!isInitialized || !_connected) return false;

    float ax, ay, az, gx, gy, gz;
    if (!_readImu(ax, ay, az, gx, gy, gz)) return false;

    unsigned long now = micros();
    float dt = (now - _lastMicros) / 1e6f;
    _lastMicros = now;

    if (dt <= 0 || dt > 0.5f) dt = 0.01f;

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

    _pitch = _kalmanUpdate(_kPitch, accelPitch, gy, dt);
    _roll  = _kalmanUpdate(_kRoll,  accelRoll,  gx, dt);

    _yaw += gz * dt;
    if (_yaw >  180.0f) _yaw -= 360.0f;
    if (_yaw < -180.0f) _yaw += 360.0f;

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
void Gyroscope::resetHeading() {
    _yaw           = 0.0f;
    _targetHeading = 0.0f;
    if (_pidYaw) _pidYaw->reset();
}

// ─────────────────────────────────────────────────────────────────────────────
bool Gyroscope::isTilted(float threshold) const {
    if (!isInitialized) return false;
    return (fabsf(_pitch) > threshold || fabsf(_roll) > threshold);
}

// ─────────────────────────────────────────────────────────────────────────────
float Gyroscope::getTiltCorrection(float targetPitch, float targetRoll, float corrLimit) {
    if (!isInitialized || !_pidPitch || !_pidRoll) { 
        return 0; 
    }
    float pc = _pidPitch->calculate(targetPitch, _pitch);
    float rc = _pidRoll->calculate(targetRoll,   _roll);
    return constrain(pc + rc, -corrLimit, corrLimit);
}

// ─────────────────────────────────────────────────────────────────────────────
float Gyroscope::getHeadingCorrection(float corrLimit) {
    // Proteksi Null Pointer
    if (!isInitialized || !_pidYaw) {
        return 0;
    }

    float err = _targetHeading - _yaw;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    float out = _pidYaw->calculate(err, 0.0f);
    return constrain(out, -corrLimit, corrLimit);
}

// ─────────────────────────────────────────────────────────────────────────────
float Gyroscope::_kalmanUpdate(KalmanState& s, float measAngle,
                                float gyroRate, float dt) {
    float rate  = gyroRate - s.bias;
    s.angle    += dt * rate;
    s.P[0][0]  += dt * (dt*s.P[1][1] - s.P[0][1] - s.P[1][0] + s.Q_angle);
    s.P[0][1]  -= dt * s.P[1][1];
    s.P[1][0]  -= dt * s.P[1][1];
    s.P[1][1]  += s.Q_bias * dt;

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