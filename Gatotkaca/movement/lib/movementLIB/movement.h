#pragma once
#include <stdint.h>
#include "driver/ledc.h"


class Encoder;
class MyPID;


class Movement {
public:
    
    Movement(const char* name,
             float Kp, float Ki, float Kd,
             int chanelA, int chanelB,
             float ppr, int RPWM, int LPWM);
    ~Movement();

    void  begin();
    void  update(float targetRPM);
    void  resetPID();
    void  setPID(float kp, float ki, float kd);

    // Getter untuk telemetri
    float getActualRPM()  const { return _actualRPM;   }
    float getTargetRPM()  const { return _targetRPM;   }
    float getPWM()        const { return _pwm;         }
    const char* getName() const { return _name;        }

private:
    const char* _name;
    int         _RPWM, _LPWM;

    Encoder*    _encoder = nullptr;
    MyPID*      _pid     = nullptr;

    
    ledc_timer_t   _timer;
    ledc_channel_t _rpwmCh;
    ledc_channel_t _lpwmCh;

    
    static int _chCounter;      // ledc channel counter max 7
    static int _timerCounter;   // ledc timer counter max 3

    float _targetRPM = 0.0f;
    float _actualRPM = 0.0f;
    float _pwm       = 0.0f;   

    static constexpr float DEADZONE_RPM = (10.0f / 128.0f) * 500;
};