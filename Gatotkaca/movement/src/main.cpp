#include <Arduino.h>
#include <HardwareSerial.h>
#include <PS4Controller.h>
#include <nvs_flash.h>

#include "../lib/movementLIB/gyroscope.h"
#include "../lib/movementLIB/movement.h"


#define PPR         7      
#define SDA_PIN     21      
#define SCL_PIN     22     


Gyroscope* gyro;
Movement *Fr, *Fl, *Br, *Bl;

float targetHeading    = 0.0f; //target awal
bool  headingLockActive = true;

void move(int forward, int strafe, int turn);
void control();

void setup() {
    Serial.begin(115200);
    nvs_flash_erase();
    nvs_flash_init();
    delay(2000);

    PS4.begin("40:1A:58:62:D6:A2");
    Serial.println(F("Program berjalan"));

    
    gyro = new Gyroscope(SDA_PIN, SCL_PIN);
    
    if (!gyro->begin()) {
        Serial.println("Gyroscope");
        while(1); 
    }

    //                Kp    Ki    Kd    chA  chB  ppr  rpwm  lpwm
    Fr = new Movement(0.1f, 0.01f, 0.1f, 34, 35, PPR, 12, 13);
    Fl = new Movement(0.1f, 0.01f, 0.1f, 36, 39, PPR, 14, 27);
    Br = new Movement(0.1f, 0.01f, 0.1f, 16, 17, PPR, 19, 18);
    Bl = new Movement(0.1f, 0.01f, 0.1f, 32, 33, PPR, 26, 25);

    Fr->begin(); Fl->begin(); Br->begin(); Bl->begin();

    gyro->resetHeading(); 
    targetHeading = 0.0f;

    Serial.println(F("semua siap!!!"));
}

void loop() {
    
    gyro->update();

    // matikan semua motor jika robot miring >20° 
    if (gyro->isTilted(20.0f)) {
        Fr->update(0); Fl->update(0); Br->update(0); Bl->update(0);
        Fr->resetPID(); Fl->resetPID(); Br->resetPID(); Bl->resetPID();
        return;
    }

    if (PS4.isConnected()) control();

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 100) {
        lastDebug = millis();
        Serial.print(F("Pitch: "));   Serial.print(gyro->getPitch(), 1);
        Serial.print(F("° | Roll: ")); Serial.print(gyro->getRoll(), 1);
        Serial.print(F("° | Yaw: "));  Serial.print(gyro->getHeading(), 1);
        Serial.print(F("° | Target: ")); Serial.print(targetHeading, 1);
        Serial.println(F("°"));
    }
}

void move(int forward, int strafe, int turn) {
    const int DEADZONE = 8; 

    if (abs(turn) > DEADZONE) {
        headingLockActive = false;

        float headingChange = (turn / 127.0f) * 2.0f; 
        targetHeading += headingChange;

        if (targetHeading >  180.0f) targetHeading -= 360.0f;
        if (targetHeading < -180.0f) targetHeading += 360.0f;

        gyro->setTargetHeading(targetHeading);
    } else headingLockActive = true;
    
    float headingCorr = 0.0f;
    if (headingLockActive) headingCorr = gyro->getHeadingCorrection(40.0f); 

    float tiltCorr = gyro->getTiltCorrection(0.0f, 0.0f, 30.0f); 

    // jika nilai joystick < DEADZONE maka semua roda berhenti
    if (abs(forward) < DEADZONE && abs(strafe) < DEADZONE && abs(turn) < DEADZONE) {
        Fr->resetPID(); Fl->resetPID(); Br->resetPID(); Bl->resetPID();
        Fr->update(0); Fl->update(0); Br->update(0); Bl->update(0);
        return;
    }
    
    float vfl = forward + strafe + headingCorr + tiltCorr;
    float vfr = forward - strafe - headingCorr + tiltCorr;
    float vbl = forward - strafe + headingCorr + tiltCorr;
    float vbr = forward + strafe - headingCorr + tiltCorr;

    // Normalisasi 
    float maxVal = max({fabsf(vfl), fabsf(vfr), fabsf(vbl), fabsf(vbr)});
    float scale  = (maxVal > 127.0f) ? (100.0f / maxVal) : (100.0f / 127.0f);

    vfl *= scale; vfr *= scale; vbl *= scale; vbr *= scale;
    Fr->update(vfr); Fl->update(vfl); Br->update(vbr); Bl->update(vbl);
}


void control() {
    if (PS4.Triangle()) {
        Serial.println(F("Segitiga ditekan"));
    }

    if (PS4.Square()) {
        Serial.println(F("Kotak ditekan"));
    }

    if (PS4.Circle()) {
        Serial.println(F("Lingkaran ditekan"));
        
    }

    move(PS4.LStickY(), PS4.LStickX(), PS4.RStickX());
}