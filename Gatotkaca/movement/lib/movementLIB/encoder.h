#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>
#include <ESP32Encoder.h>

class Encoder{
    private:
        ESP32Encoder encoder;
        int pinA, pinB;
        float ppr;
        long lastCount = 0;
        unsigned long lastUpdateMicros = 0;
        float RPMsekarang = 0;

    public:
        Encoder(int a, int b, float pulses);
        void begin();
        void update(); 
        float getRPM() const; // mengoutputkan rpm
};

#endif