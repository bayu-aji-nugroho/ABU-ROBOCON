#include <Arduino.h>
#include "../lib/movementLIB/movement.h"
#include "../lib/movementLIB/encoder.h"
#include "../lib/movementLIB/PID.h"

Movement::Movement(String name, float Kp, float Ki, float Kd, int chanelA, int ChanelB, float ppr,int RPWM, int LPWM)
: RPWM(RPWM), LPWM(LPWM), name(name){

    encoder = new Encoder(chanelA,ChanelB,ppr,name);
    pid = new MyPID(Kp,Ki,Kd,-255,255);

}

void Movement::update(float target){
    encoder->update();
    nilai_PWM_ke_roda =  pid->calculate(target, encoder->getRPM()); // output kecepatan setiap roda setelah dikoreksi
    //untuk tuning
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 100) {
        lastDebug = millis();
        Serial.printf(">%sTarget:%.1f",this->name,target);
        Serial.printf(">%sActual:%.1f",this->name,encoder->getRPM()); 
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
    Serial.println("(2)pin terpasang");

}

void Movement::resetPID() {
    pid->reset();
}

