#include "PID.h"
#include <Arduino.h>

PID::PID(float p, float i, float d, float minVal, float maxVal) 
    : kp(p), ki(i), kd(d), outMin(minVal), outMax(maxVal) {
    
    alpha = 0.25; //semakin kecil semakin halus, tapi respon melambat
    integralLimit = 150.0;
    reset();
}

float PID::calculate(float target, float current) {
    unsigned long now = micros();
    
    // Pertama kali di panggil
    if(lastTime == 0) {
        lastTime = now;
        prevError = target - current;
        prevComponent = current;
        lastFilteredDerivative = 0;
        return 0; 
    }

    float dt = (now - lastTime) / 1000.0; 
    if (dt <= 0) return 0; 

    lastTime = now;
    float error = target - current;
    
    // 1. Integral Term dengan Anti-windup
    integral += error * dt;
    integral = constrain(integral, -integralLimit, integralLimit); 
    
    // 2. Derivative on Measurement
    // Menggunakan -(current - prevComponent) karena kenaikan 'current' 
    // secara logis akan mengurangi 'error'.
    float rawDerivative = -(current - prevComponent) / dt;
    
    // 3. Low Pass Filter (LPF) pada Derivatif
    // Menghilangkan noise frekuensi tinggi dari encoder/sensor
    float filteredDerivative = (alpha * rawDerivative) + ((1.0 - alpha) * lastFilteredDerivative);
    
    // Update data untuk siklus berikutnya
    prevError = error;
    prevComponent = current;
    lastFilteredDerivative = filteredDerivative;
    
    // 4. Hitung Output PID
    float output = (error * kp) + (integral * ki) + (filteredDerivative * kd);
    
    // Batasi output ke rentang PWM
    return constrain(output, outMin, outMax);
}

void PID::reset() {
    prevError = 0;
    integral = 0;
    lastTime = 0;
    prevComponent = 0;
    lastFilteredDerivative = 0;
}