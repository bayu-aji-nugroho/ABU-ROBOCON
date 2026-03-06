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
    if (lastUpdateMicros == 0) {  
        lastUpdateMicros = micros();
        return;
    }
    unsigned long now = micros();
    float deltaTime = (now - lastUpdateMicros) / 1000000.0; // Konversi ke detik
    
    if (millis() - lastDebug > 100) {
        lastDebug = millis();
        // Serial.printf(">%scount:%.1f\n",this->name.c_str(),encoder.getCount());
    }

    if (deltaTime > 0.01) { // Hitung setiap 10ms
        _count = (long) encoder.getCount();
        long deltaTicks = _count - lastCount;

        float instantRPM = (float)(deltaTicks * 60.0f) / (ppr * deltaTime);
        
        const float alpha = 0.5; // jika lemot naikkan, jika noise turunkan
        RPMsekarang = (alpha * instantRPM) + ((1.0 - alpha) * RPMsekarang);
        
        lastCount = _count;
        lastUpdateMicros = now;
    }
   
}

float Encoder::getRPM() const { 
    return RPMsekarang; 
}
