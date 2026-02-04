#include <Arduino.h>
#include "../lib/movement.h"

class rearright {
private:
    Movement* movement;
    int pwmPin;
    int dir1Pin;
    int dir2Pin;
    int pwmChannel;
    
public:
    rearright(int pwm, int d1, int d2, int channel, float kp, float ki, float kd,
            int encA, int encB, float diameter, float ppr) {
        pwmPin = pwm;
        dir1Pin = d1;
        dir2Pin = d2;
        pwmChannel = channel;

        movement = new Movement(kp, ki, kd, encA, encB, diameter, ppr);
}

void begin() {
        pinMode(pwmPin, OUTPUT);
        pinMode(dir1Pin, OUTPUT);
        pinMode(dir2Pin, OUTPUT);
        
        ledcSetup(pwmChannel, 5000, 8);
        ledcAttachPin(pwmPin, pwmChannel);
        
        movement->begin();
}

void setSpeed(float targetSpeed) {
    float correctedSpeed = movement->update(targetSpeed);
    applySpeed(correctedSpeed);
}

void applySpeed(float speed) {
        speed = constrain(speed, -255, 255);
        
        if (speed >= 0) {
            digitalWrite(dir1Pin, HIGH);
            digitalWrite(dir2Pin, LOW);
            ledcWrite(pwmChannel, (int)abs(speed));
        } else {
            digitalWrite(dir1Pin, LOW);
            digitalWrite(dir2Pin, HIGH);
            ledcWrite(pwmChannel, (int)abs(speed));
        }
}

void stop() {
        digitalWrite(dir1Pin, LOW);
        digitalWrite(dir2Pin, LOW);
        ledcWrite(pwmChannel, 0);
}
};
