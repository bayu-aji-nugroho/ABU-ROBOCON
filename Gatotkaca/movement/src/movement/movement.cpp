#include <Arduino.h>
#include "../lib/movementLIB/movement.h"
#include "../lib/movementLIB/encoder.h"
#include "../lib/movementLIB/PID.h"

Movement::Movement(float Kp, float Ki, float Kd, int chanelA, int ChanelB, float ppr,int RPWM, int LPWM)
: RPWM(RPWM), LPWM(LPWM), SDA_PIN(), SCL_PIN(){
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

    // #if defined(ESP32) || defined(ESP8266)
    //     Wire.begin(SDA_PIN, SCL_PIN);
    // #else
    //     Wire.begin();
    // #endif

    if(!pid->gyroBegin()) {
        Serial.println(F("[Movement] ERROR: MPU6050 not detected"));
        Serial.println(F("Cek kabel SDA/SCL dan I2C (0x68 atau 0x69)"));
    } else {
        Serial.println(F("[Movement] ready"));
    }
}

void Movement::resetPID() {
    pid->reset();
}

bool Movement::gyroUpdate() {
    return pid->gyroUpdate();
}

float Movement::getPitch() {
    return pid->getPitch();
}

float Movement::getRoll() {
    return pid->getRoll();
}

float Movement::getHeading() {
    return pid->getHeading();
}

void Movement::setTargetHeading(float deg) {
    pid->setTargetHeading(deg);
}

void Movement::resetHeading() {
    pid->resetHeading();
}

bool Movement::isTilted(float threshold) {
    return pid->isTilted(threshold);
}

float Movement::getTiltCorrection(float targetPitch, float targetRoll, float corrLimit) {
    return pid->getTiltCorrection(targetPitch, targetRoll, corrLimit);
}

float Movement::getHeadingCorrection(float corrLimit) {
    return pid->getHeadingCorrection(corrLimit);
}