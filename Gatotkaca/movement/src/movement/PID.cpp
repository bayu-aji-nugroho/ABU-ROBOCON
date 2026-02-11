#include "PID.h"
#include <Arduino.h>

PID::PID(float p, float i, float d, float minVal, float maxVal) 
    : kp(p), ki(i), kd(d), outMin(minVal), outMax(maxVal) {
    
    alpha = 0.25; //semakin kecil semakin halus, tapi respon melambat
    integralLimit = 150.0;
    reset();
}

float PID::calculate(float target, float current) {
    unsigned long now = micros();
    
    // Pertama kali di panggil
    if(lastTime == 0) {
        lastTime = now;
        prevError = target - current;
        prevComponent = current;
        lastFilteredDerivative = 0;
        return 0; 
    }

    float dt = (now - lastTime) / 1000.0; 
    if (dt <= 0) return 0; 

    lastTime = now;
    float error = target - current;
    
    // 1. Integral Term dengan Anti-windup
    integral += error * dt;
    integral = constrain(integral, -integralLimit, integralLimit); 
    
    // 2. Derivative on Measurement
    // Menggunakan -(current - prevComponent) karena kenaikan 'current' 
    // secara logis akan mengurangi 'error'.
    float rawDerivative = -(current - prevComponent) / dt;
    
    // 3. Low Pass Filter (LPF) pada Derivatif
    // Menghilangkan noise frekuensi tinggi dari encoder/sensor
    float filteredDerivative = (alpha * rawDerivative) + ((1.0 - alpha) * lastFilteredDerivative);
    
    // Update data untuk siklus berikutnya
    prevError = error;
    prevComponent = current;
    lastFilteredDerivative = filteredDerivative;
    
    // 4. Hitung Output PID
    float output = (error * kp) + (integral * ki) + (filteredDerivative * kd);
    
    // Batasi output ke rentang PWM
    return constrain(output, outMin, outMax);
}

void PID::reset() {
    prevError = 0;
    integral = 0;
    lastTime = 0;
    prevComponent = 0;
    lastFilteredDerivative = 0;
}

bool PID::gyroBegin() {
    Wire.begin();
    Wire.setClock(400000);

    // Cek WHO_AM_I
    uint8_t who;
    if (!mpuRead(MPU_REG_WHO_AM_I, &who, 1)) return false;
    if (who != 0x68 && who != 0x72) return false;

    // Wake up dari sleep mode, pakai gyro X sebagai clock
    if (!mpuWrite(MPU_REG_PWR_MGMT_1, 0x01)) return false;
    delay(100);

    mpuWrite(MPU_REG_SMPLRT_DIV, 0x07); // Sample rate 125Hz
    mpuWrite(MPU_REG_CONFIG,     0x03); // DLPF ~44Hz (kurangi noise)
    mpuWrite(MPU_REG_GYRO_CFG,   0x00); // ±250°/s
    mpuWrite(MPU_REG_ACCEL_CFG,  0x00); // ±2g
    delay(50);

    // ---- Kalibrasi offset ----
    Serial.println(F("[PID-Gyro] Kalibrasi... jangan gerakkan robot!"));
    long sAx=0,sAy=0,sAz=0,sGx=0,sGy=0,sGz=0;
    float ax,ay,az,gx,gy,gz;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        readImu(ax,ay,az,gx,gy,gz);
        sAx += (long)(ax * ACCEL_SCALE);
        sAy += (long)(ay * ACCEL_SCALE);
        sAz += (long)(az * ACCEL_SCALE);
        sGx += (long)(gx * GYRO_SCALE);
        sGy += (long)(gy * GYRO_SCALE);
        sGz += (long)(gz * GYRO_SCALE);
        delay(2);
    }

    offAx =  (float)sAx / CALIB_SAMPLES / ACCEL_SCALE;
    offAy =  (float)sAy / CALIB_SAMPLES / ACCEL_SCALE;
    offAz =  (float)sAz / CALIB_SAMPLES / ACCEL_SCALE - 1.0f; // kurangi 1g
    offGx =  (float)sGx / CALIB_SAMPLES / GYRO_SCALE;
    offGy =  (float)sGy / CALIB_SAMPLES / GYRO_SCALE;
    offGz =  (float)sGz / CALIB_SAMPLES / GYRO_SCALE;

    Serial.println(F("[PID-Gyro] Selesai, siap dipakai!"));

    _gyroConnected  = true;
    _gyroFirstUpdate = true;
    gyroLastMicros   = micros();
    return true;
}

bool PID::gyroUpdate() {
    if (!_gyroConnected) return false;

    float ax,ay,az,gx,gy,gz;
    if (!readImu(ax,ay,az,gx,gy,gz)) return false;

    unsigned long now = micros();
    float dt = (now - gyroLastMicros) / 1e6f;
    gyroLastMicros = now;

    if (dt <= 0 || dt > 0.5f) dt = 0.01f;

    // Sudut dari akselerometer
    float accelPitch = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG;
    float accelRoll  = atan2f(-ax, az)                  * RAD_TO_DEG;

    if (_gyroFirstUpdate) {
        _kPitch.angle = accelPitch;
        _kRoll.angle  = accelRoll;
        gyroPitch = accelPitch;
        gyroRoll  = accelRoll;
        gyroYaw   = 0;
        _gyroFirstUpdate = false;
        return true;
    }

    // Kalman filter
    gyroPitch = kalmanUpdate(_kPitch, accelPitch, gy, dt);
    gyroRoll  = kalmanUpdate(_kRoll,  accelRoll,  gx, dt);

    // Yaw = integrasi gyro (akselerometer tidak bisa ukur yaw)
    gyroYaw += gz * dt;
    if (gyroYaw >  180.0f) gyroYaw -= 360.0f;
    if (gyroYaw < -180.0f) gyroYaw += 360.0f;

    return true;
}

float PID::getHeadingError() const {
    float err = targetHeading - gyroYaw;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

bool PID::isTilted(float threshold) const {
    return (fabsf(gyroPitch) > threshold || fabsf(gyroRoll) > threshold);
}

float PID::getTiltCorrection(float targetPitch, float targetRoll, float corrLimit) {
    float pc = _pidCalc(targetPitch, gyroPitch,
                        3.0f, 0.02f, 1.2f, corrLimit,
                        _pitchIntegral, _pitchPrevErr,
                        _pitchPrevComp, _pitchLastFD, _pitchLastT);

    float rc = _pidCalc(targetRoll, gyroRoll,
                        3.0f, 0.02f, 1.2f, corrLimit,
                        _rollIntegral, _rollPrevErr,
                        _rollPrevComp, _rollLastFD, _rollLastT);

    // Gabungkan pitch dan roll (magnitude koreksi total)
    return constrain(pc + rc, -corrLimit, corrLimit);
}

float PID::getHeadingCorrection(float corrLimit) {
    float err = getHeadingError();
    return _pidCalc(0.0f, -err,
                    2.5f, 0.05f, 0.8f, corrLimit,
                    _yawIntegral, _yawPrevErr,
                    _yawPrevComp, _yawLastFD, _yawLastT);
}

float PID::_pidCalc(float target, float current,
                    float p, float i, float d,
                    float limit,
                    float& integ, float& prevErr,
                    float& prevComp, float& lastFD,
                    unsigned long& lastT) {

    unsigned long now = micros();
    if (lastT == 0) {
        lastT   = now;
        prevErr  = target - current;
        prevComp = current;
        lastFD   = 0;
        return 0;
    }

    float dt = (now - lastT) / 1000.0f;
    if (dt <= 0) return 0;
    lastT = now;

    float error = target - current;
    integ += error * dt;
    integ  = constrain(integ, -integralLimit, integralLimit);

    float raw      = -(current - prevComp) / dt;
    float filtered = (alpha * raw) + ((1.0f - alpha) * lastFD);

    prevErr  = error;
    prevComp = current;
    lastFD   = filtered;

    float out = (error * p) + (integ * i) + (filtered * d);
    return constrain(out, -limit, limit);
}

float PID::kalmanUpdate(KalmanState& s, float measAngle,
                         float gyroRate, float dt) {
    // Predict
    float rate = gyroRate - s.bias;
    s.angle += dt * rate;
    s.P[0][0] += dt * (dt*s.P[1][1] - s.P[0][1] - s.P[1][0] + s.Q_angle);
    s.P[0][1] -= dt * s.P[1][1];
    s.P[1][0] -= dt * s.P[1][1];
    s.P[1][1] += s.Q_bias * dt;

    // Update
    float S  = s.P[0][0] + s.R_measure;
    float K0 = s.P[0][0] / S;
    float K1 = s.P[1][0] / S;
    float y  = measAngle - s.angle;
    s.angle  += K0 * y;
    s.bias   += K1 * y;
    float P00 = s.P[0][0], P01 = s.P[0][1];
    s.P[0][0] -= K0 * P00;
    s.P[0][1] -= K0 * P01;
    s.P[1][0] -= K1 * P00;
    s.P[1][1] -= K1 * P01;

    return s.angle;
}

bool PID::mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool PID::mpuRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU6050_ADDR, len);
    uint8_t i = 0;
    while (Wire.available() && i < len) buf[i++] = Wire.read();
    return (i == len);
}

bool PID::readImu(float& ax, float& ay, float& az,
                  float& gx, float& gy, float& gz) {
    uint8_t buf[14];
    if (!mpuRead(MPU_REG_ACCEL_XOUT, buf, 14)) return false;

    ax = (int16_t)(buf[0]<<8|buf[1])  / ACCEL_SCALE - offAx;
    ay = (int16_t)(buf[2]<<8|buf[3])  / ACCEL_SCALE - offAy;
    az = (int16_t)(buf[4]<<8|buf[5])  / ACCEL_SCALE - offAz;
    gx = (int16_t)(buf[8]<<8|buf[9])  / GYRO_SCALE  - offGx;
    gy = (int16_t)(buf[10]<<8|buf[11])/ GYRO_SCALE  - offGy;
    gz = (int16_t)(buf[12]<<8|buf[13])/ GYRO_SCALE  - offGz;
    return true;
}
