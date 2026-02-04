#include "../lib/movementLIB/encoder.h"

// Inisialisasi static variables
Encoder* Encoder::instances[4] = {nullptr, nullptr, nullptr, nullptr};
int Encoder::instanceCount = 0;

// Static ISR implementations
void IRAM_ATTR Encoder::ISR0() { 
    if(instances[0]) instances[0]->handleInterrupt(); 
}

void IRAM_ATTR Encoder::ISR1() { 
    if(instances[1]) instances[1]->handleInterrupt(); 
}

void IRAM_ATTR Encoder::ISR2() { 
    if(instances[2]) instances[2]->handleInterrupt(); 
}

void IRAM_ATTR Encoder::ISR3() { 
    if(instances[3]) instances[3]->handleInterrupt(); 
}

Encoder::Encoder(int a, int b, float pulses)
    : pinA(a), pinB(b), ppr(pulses) {
    // Simpan instance pointer
    if(instanceCount < 4) {
        instanceIndex = instanceCount;
        instances[instanceCount] = this;
        instanceCount++;
    } else {
        instanceIndex = -1;
    }
}

void Encoder::begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    
    // Attach interrupt berdasarkan instance index
    if(instanceIndex >= 0 && instanceIndex < 4) {
        switch(instanceIndex) {
            case 0: 
                attachInterrupt(digitalPinToInterrupt(pinA), ISR0, RISING); 
                break;
            case 1: 
                attachInterrupt(digitalPinToInterrupt(pinA), ISR1, RISING); 
                break;
            case 2: 
                attachInterrupt(digitalPinToInterrupt(pinA), ISR2, RISING); 
                break;
            case 3: 
                attachInterrupt(digitalPinToInterrupt(pinA), ISR3, RISING); 
                break;
        }
        Serial.print("Encoder ");
        Serial.print(instanceIndex);
        Serial.print(" attached to pin ");
        Serial.println(pinA);
    }
}

/* IRAM_ATTR adalah instruksi khusus untuk ESP32 agar fungsi 
   diletakkan di RAM internal (bukan Flash memory). Biar lebih cepat */ 

void IRAM_ATTR Encoder::handleInterrupt() {
    // Menentukan apakah roda bergerak maju atau mundur
    if (digitalRead(pinB) == HIGH) count++;
    else count--;
}

void Encoder::update() {
    unsigned long Tsekarang = millis();
    unsigned deltaTime = Tsekarang - Tlama;
    
    if(deltaTime >= 50) { // Setiap 50 ms akan dijalankan
        long deltaCount = count - lastCount;
        RPMsekarang = ((float)deltaCount / ppr) * (60000.0 / (float)deltaTime);
        lastCount = count;
        Tlama = Tsekarang;
    }
}

float Encoder::getRPM() const { 
    return RPMsekarang; 
}