#include <Arduino.h>
#include "../lib/movementLIB/movement.h"
#include "../lib/movementLIB/encoder.h"
#include "../lib/movementLIB/PID.h"

#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_SMPLRT_DIV  0x19
#define MPU_CONFIG      0x1A
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_ACCEL_XOUT  0x3B
#define MPU_WHO_AM_I    0x75
#define GYRO_SCALE      131.0f
#define ACCEL_SCALE     16384.0f
#define CALIB_SAMPLES   500

#define KAL_Q_ANGLE   0.001f
#define KAL_Q_BIAS    0.003f
#define KAL_R_MEASURE 0.03f

bool          Movement::_gyroReady       = false;
float         Movement::_pitch           = 0;
float         Movement::_roll            = 0;
float         Movement::_yaw             = 0;
float         Movement::_targetHeading   = 0;

float Movement::_kP_angle = 0, Movement::_kP_bias = 0;
float Movement::_kP_P00   = 0, Movement::_kP_P01  = 0;
float Movement::_kP_P10   = 0, Movement::_kP_P11  = 0;

// Kalman roll
float Movement::_kR_angle = 0, Movement::_kR_bias = 0;
float Movement::_kR_P00   = 0, Movement::_kR_P01  = 0;
float Movement::_kR_P10   = 0, Movement::_kR_P11  = 0;

// Offset kalibrasi
float Movement::_offAx = 0, Movement::_offAy = 0, Movement::_offAz = 0;
float Movement::_offGx = 0, Movement::_offGy = 0, Movement::_offGz = 0;

unsigned long Movement::_gyroLastMicros  = 0;
bool          Movement::_gyroFirstUpdate = true;

Movement::Movement(float Kp, float Ki, float Kd, int chanelA, int ChanelB, float ppr,int RPWM, int LPWM)
: RPWM(RPWM), LPWM(LPWM){
    encoder = new Encoder(chanelA,ChanelB,ppr);
    pid = new PID(Kp,Ki,Kd,-255,255);

}

void Movement::update(float target){
    encoder->update();
    nilai_PWM_ke_roda =  pid->calculate(target, encoder->getRPM()); // output kecepatan setiap roda setelah dikoreksi
    //untuk tuning
    if(target != 0){
        Serial.print(">Target:");
        Serial.println(target);
        Serial.print("Actual: ");
        Serial.print(encoder->getRPM());
        Serial.print("PWM: ");
        Serial.println(nilai_PWM_ke_roda);

    }
    
    //menulis lpwm dan pwm ke motor driver
    if(nilai_PWM_ke_roda > 0){
        analogWrite(RPWM, nilai_PWM_ke_roda);
        analogWrite(LPWM,0);
    } else if(nilai_PWM_ke_roda < 0){
        analogWrite(LPWM, nilai_PWM_ke_roda*(-1));
        analogWrite(RPWM,0);
    } else {
        analogWrite(LPWM,0);
        analogWrite(RPWM,0);
    }
}

void Movement::begin(){
    encoder -> begin();
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
}

void Movement::resetPID() {
    pid->reset();
}

bool Movement::gyroBegin() {
    Wire.begin();
    Wire.setClock(400000); // Fast mode 400kHz

    uint8_t who;
    if (!_mpuRead(MPU_WHO_AM_I, &who, 1))         return false;
    if (who != 0x68 && who != 0x72)               return false;

    if (!_mpuWrite(MPU_PWR_MGMT_1, 0x01))         return false; // wake up
    delay(100);

    _mpuWrite(MPU_SMPLRT_DIV, 0x07); // sample rate 125Hz
    _mpuWrite(MPU_CONFIG,     0x03); // DLPF ~44Hz
    _mpuWrite(MPU_GYRO_CFG,   0x00); // ±250°/s
    _mpuWrite(MPU_ACCEL_CFG,  0x00); // ±2g
    delay(50);

    // ── Kalibrasi offset (robot harus DIAM dan RATA) ──
    Serial.println(F("[Movement-Gyro] Kalibrasi... jangan gerakkan robot!"));
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

    _offAx = (float)sAx / CALIB_SAMPLES / ACCEL_SCALE;
    _offAy = (float)sAy / CALIB_SAMPLES / ACCEL_SCALE;
    _offAz = (float)sAz / CALIB_SAMPLES / ACCEL_SCALE - 1.0f; // kurangi 1g gravitasi
    _offGx = (float)sGx / CALIB_SAMPLES / GYRO_SCALE;
    _offGy = (float)sGy / CALIB_SAMPLES / GYRO_SCALE;
    _offGz = (float)sGz / CALIB_SAMPLES / GYRO_SCALE;

    Serial.println(F("[Movement-Gyro] Siap!"));

    _gyroReady       = true;
    _gyroFirstUpdate = true;
    _gyroLastMicros  = micros();
    return true;
}

bool Movement::gyroUpdate() {
    if (!_gyroReady) return false;

    float ax, ay, az, gx, gy, gz;
    if (!_readImu(ax, ay, az, gx, gy, gz)) return false;

    unsigned long now = micros();
    float dt = (now - _gyroLastMicros) / 1e6f;
    _gyroLastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 0.01f;

    float accelPitch = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG;
    float accelRoll  = atan2f(-ax, az)                  * RAD_TO_DEG;

    if (_gyroFirstUpdate) {
        _kP_angle = accelPitch;
        _kR_angle = accelRoll;
        _pitch = accelPitch;
        _roll  = accelRoll;
        _yaw   = 0;
        _gyroFirstUpdate = false;
        return true;
    }

    // Kalman filter pitch & roll
    _pitch = _kalmanUpdate(_kP_angle, _kP_bias,
                           _kP_P00, _kP_P01, _kP_P10, _kP_P11,
                           accelPitch, gy, dt);

    _roll  = _kalmanUpdate(_kR_angle, _kR_bias,
                           _kR_P00, _kR_P01, _kR_P10, _kR_P11,
                           accelRoll,  gx, dt);

    // Yaw = integrasi gyro
    _yaw += gz * dt;
    if (_yaw >  180.0f) _yaw -= 360.0f;
    if (_yaw < -180.0f) _yaw += 360.0f;

    return true;
}

float Movement::getHeadingError() {
    float err = _targetHeading - _yaw;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

bool Movement::isTilted(float threshold) {
    return (fabsf(_pitch) > threshold || fabsf(_roll) > threshold);
}

float Movement::getTiltCorrection(float corrLimit) {
    const float kp = 3.0f;
    float correction = (_pitch + _roll) * kp;
    return constrain(correction, -corrLimit, corrLimit);
}

float Movement::getHeadingCorrection(float corrLimit) {
    const float kp = 2.5f;
    float correction = getHeadingError() * kp;
    return constrain(correction, -corrLimit, corrLimit);
}

float Movement::_kalmanUpdate(
        float& angle, float& bias,
        float& P00, float& P01, float& P10, float& P11,
        float measAngle, float gyroRate, float dt)
{
    // Predict
    float rate = gyroRate - bias;
    angle += dt * rate;
    P00 += dt * (dt*P11 - P01 - P10 + KAL_Q_ANGLE);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += KAL_Q_BIAS * dt;

    // Update
    float S  = P00 + KAL_R_MEASURE;
    float K0 = P00 / S;
    float K1 = P10 / S;
    float y  = measAngle - angle;
    angle   += K0 * y;
    bias    += K1 * y;
    float p00 = P00, p01 = P01;
    P00 -= K0 * p00;
    P01 -= K0 * p01;
    P10 -= K1 * p00;
    P11 -= K1 * p01;

    return angle;
}

bool Movement::_mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool Movement::_mpuRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, len);
    uint8_t i = 0;
    while (Wire.available() && i < len) buf[i++] = Wire.read();
    return (i == len);
}

bool Movement::_readImu(float& ax, float& ay, float& az,
                         float& gx, float& gy, float& gz) {
    uint8_t buf[14];
    if (!_mpuRead(MPU_ACCEL_XOUT, buf, 14)) return false;

    ax = (int16_t)(buf[0]<<8|buf[1])  / ACCEL_SCALE - _offAx;
    ay = (int16_t)(buf[2]<<8|buf[3])  / ACCEL_SCALE - _offAy;
    az = (int16_t)(buf[4]<<8|buf[5])  / ACCEL_SCALE - _offAz;
    gx = (int16_t)(buf[8]<<8|buf[9])  / GYRO_SCALE  - _offGx;
    gy = (int16_t)(buf[10]<<8|buf[11])/ GYRO_SCALE  - _offGy;
    gz = (int16_t)(buf[12]<<8|buf[13])/ GYRO_SCALE  - _offGz;
    return true;
}