#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float p, float i, float d, float minVal, float maxVal);
    float calculate(float target, float current);
    void reset();

private:
    float kp, ki, kd;
    float outMin, outMax;
    float integral, prevError;
    float integralLimit;
    unsigned long lastTime;
    float prevComponent;           // Menyimpan nilai 'current' sebelumnya
    float lastFilteredDerivative;  // Menyimpan hasil filter derivatif terakhir
    float alpha;                   // Koefisien filter (0.0 - 1.0)
};

#endif