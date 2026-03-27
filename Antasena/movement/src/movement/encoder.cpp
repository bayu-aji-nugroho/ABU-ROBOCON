#include "../lib/movementLIB/encoder.h"


Encoder::Encoder(int a, int b, float pulses, String name)
    : pinA(a), pinB(b), ppr(pulses), name(name) {}

void Encoder::begin() {  
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachFullQuad(pinA,pinB);
    encoder.setCount(0); 
    lastUpdateMicros = micros();
    lastTickMicros   = micros();
    
}

void Encoder::update() {
    const unsigned long now = micros();

    if (lastUpdateMicros == 0) {
        lastUpdateMicros = now;
        lastTickMicros   = now;
        return;
    }
    const float deltaTime = (now - lastUpdateMicros) / 1000000.0f;

  
    if ((now - lastTickMicros) >= IDLE_TIMEOUT_US) {
        RPMsekarang      = 0.0f;
        lastCount        = (long long)encoder.getCount();
        lastUpdateMicros = now;
        lastTickMicros   = now; 
        return;
    }

   
    if (deltaTime >= MIN_DELTA_TIME) {
        _count = (long long)encoder.getCount();
        const long long deltaTicks = (long long)_count - lastCount;

        
        if (deltaTicks != 0) {
            lastTickMicros = now;
        }
        const float instantRPM = (deltaTicks * 60.0f) / (ppr * deltaTime);

        // Low-pass filter (tau = 50ms)
        const float tau   = 0.05f;
        const float alpha = deltaTime / (deltaTime + tau);
        RPMsekarang = alpha * instantRPM + (1.0f - alpha) * RPMsekarang;

        lastCount        = (long long)_count;
        lastUpdateMicros = now;
    }
   
}

float Encoder::getRPM() const { 
    return RPMsekarang; 
}
