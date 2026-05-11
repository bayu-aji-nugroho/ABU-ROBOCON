#include "gyroscope.h"
#include <Arduino.h>

Mpu::Mpu(int sda, int scl, float Kp, float Ki, float Kd)
    : SDA_PIN(sda), SCL_PIN(scl)
{
    headingPID = new MyPID(Kp, Ki, Kd, -30.0f, 30.0f);
}

Mpu::~Mpu() {
    delete headingPID;
}

void Mpu::init() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    mpu.initialize();

    if (!mpu.testConnection()) {
        return;
    }
    Gyro = true;
    

    uint8_t dmpStatus = mpu.dmpInitialize();
    if (dmpStatus != 0) {
        return;
    }

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);
    dmpReady   = true;
    packetSize = mpu.dmpGetFIFOPacketSize();


    for (int i = 0; i < 50; i++) {
        update();
        delay(10);
    }

    // Set referensi heading awal
    yawOffset = yaw;
}

void Mpu::update() {
    if (!dmpReady) return;

    fifoCount = mpu.getFIFOCount();


    if (fifoCount >= 1024) {
        mpu.resetFIFO();
        
        return;
    }

    if (fifoCount < packetSize) return; // data belum siap

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // radian → derajat
    const float rawYaw = ypr[0] * RAD_TO_DEG;
    pitch              = ypr[1] * RAD_TO_DEG;
    roll               = ypr[2] * RAD_TO_DEG;


    yaw = rawYaw;
    
}

float Mpu::CorectionHeading() {
    if (!headingPID) return 0.0f;


    float error = yawOffset - yaw;
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    return headingPID->calculate(0.0f, -error);
}

void Mpu::measureDrift(uint16_t durationMs) {
    if (!dmpReady) return;

    Serial.println("[IMU] Auto-Calibration DMP sudah dijalankan. measureDrift manual dilewati.");
}