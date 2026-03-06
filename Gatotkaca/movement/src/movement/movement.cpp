#include <Arduino.h>
#include "../lib/movementLIB/movement.h"
#include "../lib/movementLIB/encoder.h"
#include "../lib/movementLIB/PID.h"


Movement::Movement(String name, float Kp, float Ki, float Kd,
                   int chanelA, int ChanelB, float ppr, int RPWM, int LPWM)
    : RPWM(RPWM), LPWM(LPWM), name(name)
{
    encoder = new Encoder(chanelA, ChanelB, ppr, name);
    if (!encoder) {  
        Serial.printf("[FATAL] Encoder '%s' alloc gagal! Heap: %d\n",
                      name.c_str(), ESP.getFreeHeap());
        ESP.restart();
    }

    pid = new MyPID(Kp, Ki, Kd, -255, 255);
    if (!pid) {
        Serial.printf("[FATAL] PID '%s' alloc gagal! Heap: %d\n",
                      name.c_str(), ESP.getFreeHeap());
        ESP.restart();
    }
}


Movement::~Movement() {
    delete encoder;
    delete pid;
}



void Movement::update(float target) {
    if (!encoder || !pid) return; 

    encoder->update();
    _targetRPM = (target / 127.0f) * 499.0f;
    nilai_PWM_ke_roda = pid->calculate(_targetRPM, encoder->getRPM());
    _actualRPM = encoder->getRPM();
    _count     = encoder->getCount();

    
    // if (millis() - lastDebug > 100) {
    //     lastDebug = millis();
    //     // Serial.printf(">%sTarget:%.1f\n", this->name.c_str(), targetRPM);
    //     // Serial.printf(">%sActual:%.1f\n", this->name.c_str(), encoder->getRPM());
    //     // Serial.printf(">%spwm:%.1f\n", this->name.c_str(), nilai_PWM_ke_roda);
        
        
    // }

    if (nilai_PWM_ke_roda > 0) {
        analogWrite(RPWM, constrain((int)roundf(nilai_PWM_ke_roda), 0, 255));
        analogWrite(LPWM, 0);
    } else if (nilai_PWM_ke_roda < 0) {
        analogWrite(LPWM, constrain((int)roundf(-nilai_PWM_ke_roda), 0, 255));
        analogWrite(RPWM, 0);
    } else {
        analogWrite(LPWM, 0);
        analogWrite(RPWM, 0);
    }
}


void Movement::begin() {
    if (encoder) encoder->begin();
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    Serial.printf("(2) Pin motor '%s' terpasang\n", name.c_str());
}


void Movement::resetPID() {
    if (pid) pid->reset();
}

