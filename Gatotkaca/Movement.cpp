#include "Movement.h"
#include <cmath>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const float POS_TOLERANCE = 0.5f;      // cm
static const float ANGLE_TOLERANCE = 0.017f;  // ~1 derajat

Movement::Movement() : currentX(0), currentY(0), currentTheta(0), currentLiftLevel(0) {}

void Movement::setPose(float x, float y, float theta) {
    currentX = x;
    currentY = y;
    currentTheta = normalizeAngle(theta);
}

bool Movement::moveTo(float x, float y, float theta, float speed, int timeout) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    float dx = x - currentX;
    float dy = y - currentY;
    float distance = std::sqrt(dx*dx + dy*dy);

    if (distance < POS_TOLERANCE) {
        // sudah di (x,y), tinggal rotasi akhir
        return pivot(theta - currentTheta, speed, timeout);
    }

    // Rotasi menghadap target
    float targetAngle = std::atan2(dy, dx);
    float angleError = normalizeAngle(targetAngle - currentTheta);
    if (std::abs(angleError) > ANGLE_TOLERANCE) {
        if (!pivot(angleError, speed, timeout)) return false;
    }

    // Gerak lurus
    unsigned long start = getCurrentTimeMs();
    while (true) {
        updatePoseFromEncoders();
        dx = x - currentX;
        dy = y - currentY;
        float remaining = std::sqrt(dx*dx + dy*dy);
        if (remaining < POS_TOLERANCE) break;
        if (getCurrentTimeMs() - start > (unsigned long)timeout) {
            stop();
            return false;
        }
        setMotorSpeeds(speed, speed);
        // polling tanpa delay: loop kecil agar CPU tidak terlalu tinggi
        unsigned long t = getCurrentTimeMs();
        while (getCurrentTimeMs() - t < 10) {}
    }
    stop();

    // Rotasi akhir ke sudut theta
    float finalError = normalizeAngle(theta - currentTheta);
    if (std::abs(finalError) > ANGLE_TOLERANCE) {
        if (!pivot(finalError, speed, timeout)) return false;
    }
    return true;
}

void Movement::moveRelative(float dx, float dy, float dTheta, int duration) {
    if (duration <= 0) return;

    // Hitung kecepatan linier dan angular (asumsi kecepatan max 50 cm/s, 180 deg/s)
    float distance = std::sqrt(dx*dx + dy*dy);
    float linearSpeedCmPerSec = distance / (duration / 1000.0f);
    float maxLinSpeed = 50.0f; // cm/s pada 100% power
    float linearPercent = (linearSpeedCmPerSec / maxLinSpeed) * 100.0f;
    if (linearPercent > 100) linearPercent = 100;
    if (linearPercent < 0) linearPercent = 0;

    float angularSpeedRadPerSec = dTheta / (duration / 1000.0f);
    float maxAngSpeed = M_PI; // rad/s (180 deg/s)
    float angularPercent = (angularSpeedRadPerSec / maxAngSpeed) * 100.0f;
    if (angularPercent > 100) angularPercent = 100;
    if (angularPercent < -100) angularPercent = -100;

    // Differential drive: left = linear - angular, right = linear + angular
    float left = linearPercent - angularPercent;
    float right = linearPercent + angularPercent;
    if (left > 100) left = 100; if (left < -100) left = -100;
    if (right > 100) right = 100; if (right < -100) right = -100;

    unsigned long start = getCurrentTimeMs();
    while (getCurrentTimeMs() - start < (unsigned long)duration) {
        setMotorSpeeds(left, right);
        updatePoseFromEncoders();
        unsigned long t = getCurrentTimeMs();
        while (getCurrentTimeMs() - t < 5) {}
    }
    stop();

    // Update pose ideal
    currentX += dx;
    currentY += dy;
    currentTheta = normalizeAngle(currentTheta + dTheta);
}

bool Movement::pivot(float angle, float speed, int timeout) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    float target = normalizeAngle(currentTheta + angle);
    float error = normalizeAngle(target - currentTheta);
    if (std::abs(error) < ANGLE_TOLERANCE) return true;

    float left, right;
    if (error > 0) { // putar CCW (kiri)
        left = -speed;
        right = speed;
    } else {
        left = speed;
        right = -speed;
    }

    unsigned long start = getCurrentTimeMs();
    while (true) {
        updatePoseFromEncoders();
        error = normalizeAngle(target - currentTheta);
        if (std::abs(error) < ANGLE_TOLERANCE) break;
        if (getCurrentTimeMs() - start > (unsigned long)timeout) {
            stop();
            return false;
        }
        setMotorSpeeds(left, right);
        unsigned long t = getCurrentTimeMs();
        while (getCurrentTimeMs() - t < 5) {}
    }
    stop();
    currentTheta = target;
    return true;
}

void Movement::setLiftHeight(int level, float speed) {
    if (level < 0) level = 0;
    if (level > 2) level = 2;
    if (level == currentLiftLevel) return;

    float direction = (level > currentLiftLevel) ? 1.0f : -1.0f;
    float motorSpeed = direction * speed;
    if (motorSpeed > 100) motorSpeed = 100;
    if (motorSpeed < -100) motorSpeed = -100;

    setLiftMotorSpeed(motorSpeed);
    while (!isLiftAtLevel(level)) {
        // polling tanpa delay
        unsigned long t = getCurrentTimeMs();
        while (getCurrentTimeMs() - t < 5) {}
    }
    setLiftMotorSpeed(0);
    currentLiftLevel = level;
}

void Movement::stop() {
    setMotorSpeeds(0, 0);
    setLiftMotorSpeed(0);
}

// ----- Implementasi abstraksi (platform-dependent, contoh dengan std::chrono) -----
void Movement::setMotorSpeeds(float leftPercent, float rightPercent) {
    // Dalam implementasi nyata, panggil driver motor (PWM, GPIO, dll)
    // Di sini hanya placeholder.
}

void Movement::setLiftMotorSpeed(float percent) {
    // Placeholder
}

void Movement::updatePoseFromEncoders() {
    // Placeholder: baca encoder dan hitung odometri
}

bool Movement::isLiftAtLevel(int level) {
    // Placeholder: baca limit switch
    return true;
}

unsigned long Movement::getCurrentTimeMs() {
    using namespace std::chrono;
    static auto start = steady_clock::now();
    auto now = steady_clock::now();
    return duration_cast<milliseconds>(now - start).count();
}

float Movement::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}