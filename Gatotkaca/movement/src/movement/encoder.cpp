#include "../lib/movementLIB/encoder.h"


Encoder::Encoder(int a, int b, float pulses, String name)
    : pinA(a), pinB(b), ppr(pulses), name(name) {}

void Encoder::begin() {
    
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachFullQuad(pinA,pinB);
    encoder.setCount(0);
    lastUpdateMicros = micros();
}

void Encoder::update() {
    unsigned long now = micros();
    float deltaTime = (now - lastUpdateMicros) / 1000000.0; // Konversi ke detik
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 100) {
        lastDebug = millis();
        Serial.printf(">%scount:%.1f",this->name,encoder.getCount());
    }

    if (deltaTime > 0.01) { // Hitung setiap 10ms
        long currentCount = (long) encoder.getCount();
        long deltaTicks = currentCount - lastCount;

        float instantRPM = (deltaTicks / ppr) / (deltaTime / 60.0);
        
        const float alpha = 0.5; 
        RPMsekarang = (alpha * instantRPM) + ((1.0 - alpha) * RPMsekarang);
        
        lastCount = currentCount;
        lastUpdateMicros = now;
    }
   
}

float Encoder::getRPM() const { 
    return RPMsekarang; 
}
