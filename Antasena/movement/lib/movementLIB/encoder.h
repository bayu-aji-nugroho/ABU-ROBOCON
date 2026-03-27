#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>
#include <ESP32Encoder.h>

class Encoder { 
    private:
        ESP32Encoder encoder;
        int pinA, pinB;
        float ppr;
        long long lastCount = 0;
        unsigned long lastUpdateMicros  = 0;
        unsigned long lastTickMicros    = 0;  
        float RPMsekarang = 0.0f;
        String name;
        long long _count = 0.0f;

        static constexpr unsigned long IDLE_TIMEOUT_US = 100000UL;
        static constexpr float         MIN_DELTA_TIME  = 0.01f;   

    public:
        Encoder(int a, int b, float pulses, String name);
        void  begin();
        void  update();
        float getRPM()   const;
        float getCount() const { return _count; }
};

#endif