#include "movement.h"
#include "encoder.h"
#include "PID.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"
#include <math.h>

static const char* TAG = "Movement";

int Movement::_chCounter    = 0;
int Movement::_timerCounter = 0;

static void ledc_timer_init(ledc_timer_t timer)
{
    ledc_timer_config_t cfg = {};
    cfg.speed_mode      = LEDC_LOW_SPEED_MODE;
    cfg.timer_num       = timer;
    cfg.duty_resolution = LEDC_TIMER_12_BIT;   
    cfg.freq_hz         = 18000;              
    cfg.clk_cfg         = LEDC_AUTO_CLK;

    esp_err_t ret = ledc_timer_config(&cfg);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "ledc_timer_config timer=%d gagal: %s",
                 (int)timer, esp_err_to_name(ret));
}

static void ledc_ch_init(ledc_channel_t ch, ledc_timer_t timer, int pin)
{
    ledc_channel_config_t cfg = {};
    cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    cfg.channel    = ch;
    cfg.timer_sel  = timer;
    cfg.intr_type  = LEDC_INTR_DISABLE;
    cfg.gpio_num   = pin;
    cfg.duty       = 0;
    cfg.hpoint     = 0;

    esp_err_t ret = ledc_channel_config(&cfg);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "ledc_channel_config ch=%d pin=%d gagal: %s",
                 (int)ch, pin, esp_err_to_name(ret));
}

static inline void ledc_write(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}


Movement::Movement(const char* name,
                   float Kp, float Ki, float Kd,
                   int chanelA, int chanelB,
                   float ppr, int RPWM, int LPWM)
    : _name(name), _RPWM(RPWM), _LPWM(LPWM)
{
    _encoder = new Encoder(chanelA, chanelB, ppr, name);
    if (!_encoder) {
        ESP_LOGE(TAG, "[FATAL] Encoder '%s' alloc gagal!", name);
        esp_restart();
    }

    _pid = new MyPID(Kp, Ki, Kd, -4095.0f, 4095.0f);
    if (!_pid) {
        ESP_LOGE(TAG, "[FATAL] PID '%s' alloc gagal!", name);
        esp_restart();
    }
}

Movement::~Movement()
{
    delete _encoder;
    delete _pid;
}

void Movement::begin()
{
    if (_encoder) _encoder->begin();

    _timer   = static_cast<ledc_timer_t>  (_timerCounter++);
    _rpwmCh  = static_cast<ledc_channel_t>(_chCounter++);
    _lpwmCh  = static_cast<ledc_channel_t>(_chCounter++);

    if ((int)_timer > 3 || (int)_lpwmCh > 7) {
        ESP_LOGE(TAG, "[FATAL] '%s': LEDC timer/channel melebihi batas!", _name);
        esp_restart();
    }

    ledc_timer_init(_timer);
    ledc_ch_init(_rpwmCh, _timer, _RPWM);
    ledc_ch_init(_lpwmCh, _timer, _LPWM);

    ESP_LOGI(TAG, "'%s' begin OK (timer=%d ch=%d/%d RPWM=%d LPWM=%d)",
             _name, (int)_timer, (int)_rpwmCh, (int)_lpwmCh, _RPWM, _LPWM);
}

void Movement::update(float targetRPM)
{
    if (!_encoder || !_pid) return;

    _encoder->update();

    if (fabsf(targetRPM) <= DEADZONE_RPM) {
        _pid->reset();
        _targetRPM = 0.0f;
        _actualRPM = _encoder->getRPM();
        _pwm       = 0.0f;
        ledc_write(_rpwmCh, 0);
        ledc_write(_lpwmCh, 0);
        return;
    }

    _targetRPM = targetRPM;
    _actualRPM = _encoder->getRPM();
    _pwm       = _pid->calculate(_targetRPM, _actualRPM);

    const uint32_t duty = static_cast<uint32_t>(roundf(fabsf(_pwm)));

    if (_pwm > 0.0f) {
        ledc_write(_rpwmCh, duty);
        ledc_write(_lpwmCh, 0);
    } else if (_pwm < 0.0f) {
        ledc_write(_lpwmCh, duty);
        ledc_write(_rpwmCh, 0);
    } else {
        ledc_write(_rpwmCh, 0);
        ledc_write(_lpwmCh, 0);
    }
}

void Movement::resetPID()
{
    if (_pid) _pid->reset();
}

void Movement::setPID(float kp, float ki, float kd)
{
    if (_pid) _pid->setTunings(kp, ki, kd);
}