#include "Tools.h"
#include <chrono>

Tools::Tools() : isGripperClosed(false) {
    isToolDeployed[0] = false;
    isToolDeployed[1] = false;
}

bool Tools::grabKFS(float pressure, int duration) {
    if (pressure < 0) pressure = 0;
    if (pressure > 100) pressure = 100;
    // Gerakan gripper menutup dengan tekanan tertentu
    setGripperMotor(pressure);   // asumsi positif = menutup
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start).count() < duration) {
        // polling sensor gripper (misal limit switch)
        if (checkSensor(0)) { // sensorID 0 = gripper closed sensor
            setGripperMotor(0);
            isGripperClosed = true;
            return true;
        }
        // delay kecil
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    setGripperMotor(0);
    isGripperClosed = false;
    return false;
}

bool Tools::releaseKFS(int duration) {
    // Buka gripper
    setGripperMotor(-100); // asumsi negatif = membuka
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start).count() < duration) {
        // Bisa berhenti lebih awal jika sensor terbuka penuh
        if (!checkSensor(0)) { // sensor gripper sudah terbuka
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    setGripperMotor(0);
    isGripperClosed = false;
    return true;
}

bool Tools::deployTool(int toolID) {
    if (toolID < 0 || toolID >= 2) return false;
    setToolActuator(toolID, true);
    isToolDeployed[toolID] = true;
    // Simulasi waktu deploy
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

void Tools::resetMechanism() {
    // Kembalikan gripper ke posisi terbuka
    setGripperMotor(-100);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    setGripperMotor(0);
    isGripperClosed = false;

    // Kembalikan semua tool ke posisi home
    for (int i = 0; i < 2; i++) {
        if (isToolDeployed[i]) {
            setToolActuator(i, false);
            isToolDeployed[i] = false;
        }
    }
    homeAllActuators();
}

bool Tools::checkSensor(int sensorID) {
    // Implementasi polling sensor nyata (limit switch, IR, dll)
    // Di sini dummy: selalu false (tidak terdeteksi)
    return false;
}

// ----- Fungsi abstraksi hardware (ganti dengan driver nyata) -----
void Tools::setGripperMotor(float speedPercent) {
    // Kirim sinyal PWM ke driver motor gripper
    // speedPercent -100..100
}

void Tools::setToolActuator(int toolID, bool activate) {
    // Nyalakan solenoid, servo, atau motor DC untuk tool
}

void Tools::homeAllActuators() {
    // Kembalikan semua aktuator ke posisi home (misal dengan sensor limit)
}