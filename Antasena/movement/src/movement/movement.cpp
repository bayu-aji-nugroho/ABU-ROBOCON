#include <Arduino.h>
#include "../lib/movementLIB/movement.h"
#include "../lib/movementLIB/encoder.h"
#include "../lib/movementLIB/PID.h"


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

