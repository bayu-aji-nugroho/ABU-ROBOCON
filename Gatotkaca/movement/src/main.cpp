#include <Arduino.h>
#include <PS4Controller.h>
#include <nvs_flash.h>
// #include <esp_now.h>
// #include <WiFi.h>
#include <array>
#include <utility>

#include "../lib/movementLIB/gyroscope.h"
#include "../lib/movementLIB/movement.h"


typedef struct struct_telemetry {
    float fr_actual, fr_target, fr_count, fr_error, fr_pwm;
    float fl_actual, fl_target, fl_count, fl_error, fl_pwm;
    float br_actual, br_target, br_count, br_error, br_pwm;
    float bl_actual, bl_target, bl_count, bl_error, bl_pwm;
    bool  gamepadConnected;
    int   stickLY, stickLX, stickRX;
    int   battery;
} struct_telemetry;

struct_telemetry myData;
uint8_t broadcastAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // MAC Address Penerima
// ---------------------------------------

namespace Config { // pengganti #define, cpp modern
    constexpr bool  LOG                = true;      // log [untuk testing]   
    constexpr float PPR                = 134.4f;    // PPR Dasar(7) × Gear Ratio
    constexpr int   SDA_PIN            = 21;        // pin SDA 
    constexpr int   SCL_PIN            = 22;        // pin SCL
    constexpr int   DEADZONE           = 10;        // gamepad kadang error/ tidak akurat
    constexpr float MAX_RPM            = 499.0f;    // rpm max motor
    constexpr float MAX_TURN_DEG_PER_S = 120.0f;    // Batas kecepatan rotasi robot
    constexpr float TILT_LIMIT         = 20.0f;     // batas bahaya, roda berhenti!
    constexpr float HEADING_CORR       = 40.0f;     // mengoreksi arah heading robot agar tetap lurus
    constexpr float TILT_CORR          = 30.0f;     // mengoreksi kemiringan badan robot
          
}

Gyroscope gyro(Config::SDA_PIN, Config::SCL_PIN);

static Movement* motors[4] = {nullptr};

enum Motor : uint8_t { M_FR = 0, M_FL = 1, M_BR = 2, M_BL = 3 };

static float targetHeading      = 0.0f;
static bool  headingLockActive  = true;
static bool  isGyroAvaible       = false;



static void      move(int forward, int strafe, int turn);   // x-drive
static void      control();                                 // mapping dari gamepad
static void      stopAll();                                 // hentikan semua motor
static void      pushTelemetry(bool gamepad);               // LOG
static void      resetAllPID();                             // reset PID
[[nodiscard]] static
std::pair<float, float>  gyroLogic(int turn);               // berisi semua logic gyro


void setup() {
    Serial.begin(115200);
    nvs_flash_erase();
    nvs_flash_init();
    delay(2000);

    
    // WiFi.mode(WIFI_STA);
    // if (esp_now_init() == ESP_OK) {
    //     esp_now_peer_info_t peerInfo = {};
    //     memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    //     peerInfo.channel = 0;  
    //     peerInfo.encrypt = false;
    //     esp_now_add_peer(&peerInfo);
    // }
   

    PS4.begin("40:1A:58:62:D6:A2");
    Serial.println("status: ");

    if (gyro.begin()) Serial.println("!gyro terpasang");
    else Serial.println("Gyro tidak terdeteksi [warning]");
    
    delay(2000);
    //                          name  Kp    Ki    Kd    chA chB  ppr   rpwm  lpwm
    motors[M_FR] = new Movement("Fr", 0.5f,0.0f,0.0f, 36,39, Config::PPR, 27,14);
    motors[M_FL] = new Movement("Fl", 0.5f,0.0f,0.0f, 34,35, Config::PPR, 13,23);
    motors[M_BR] = new Movement("Br", 0.5f,0.0f,0.0f, 25,26, Config::PPR, 17,16);
    motors[M_BL] = new Movement("Bl", 0.5f,0.0f,0.0f, 32,33, Config::PPR, 19,18);
    for (int i = 0; i < 4; i++) if (motors[i]) motors[i]->begin();

    gyro.resetHeading();
    targetHeading = 0.0f;
    Serial.print("deteksi selesai!");
    
}


void loop() {
    gyro.update();

    if (gyro.isTilted(Config::TILT_LIMIT)) {
        stopAll();
        resetAllPID();
        return;
    }
    if (PS4.isConnected()) control();
    else                   stopAll();
    // motors[M_FR].update(65534); motors[M_FL].update(65534); motors[M_BR].update(65534); motors[M_BL].update(65534);
    // motors[M_FR]->update(255); 
    if constexpr (Config::LOG) {
        pushTelemetry(PS4.isConnected());
    }
    delay(5);

}


static void control() {
    if (PS4.Triangle()) Serial.println(F("Segitiga ditekan"));
    if (PS4.Square())   Serial.println(F("Kotak ditekan"));
    if (PS4.Circle())   Serial.println(F("Lingkaran ditekan"));

    move(PS4.LStickY(), PS4.LStickX(), PS4.RStickX());
}


static void move(int forward, int strafe, int turn) {
    const bool cekGamePad = abs(forward) < Config::DEADZONE
                    && abs(strafe)  < Config::DEADZONE
                    && abs(turn)    < Config::DEADZONE;
    if (cekGamePad) {
        resetAllPID();
        stopAll();
        return;
    }

    auto [headingCorr, tiltCorr] = gyroLogic(turn);

    float vfl = forward + strafe + turn + headingCorr + tiltCorr;
    float vfr = (forward - strafe - turn - headingCorr + tiltCorr)*(-1);
    float vbl = forward - strafe + turn + headingCorr - tiltCorr;
    float vbr = (forward + strafe - turn - headingCorr - tiltCorr)*(-1);

    // Normalisasi agar tidak melebihi 127
    const float maxVal = max({fabsf(vfl), fabsf(vfr), fabsf(vbl), fabsf(vbr)});
    const float scale  = (maxVal > 127.0f) ? (127.0f / maxVal) : 1.0f;
    vfl *= scale; vfr *= scale; vbl *= scale; vbr *= scale;

    
    motors[M_FR]->update(vfr);
    motors[M_FL]->update(vfl);
    motors[M_BR]->update(vbr);
    motors[M_BL]->update(vbl);
}


static void stopAll() {
    for (int i = 0; i < 4; i++)
        if (motors[i]) motors[i]->update(0);
}

static void resetAllPID() {
    for (int i = 0; i < 4; i++)
        if (motors[i]) motors[i]->resetPID();
}


static std::pair<float, float> gyroLogic(int turn) {
    static unsigned long lastGyroTime = 0;

    const unsigned long now = millis();
    const float dt = (lastGyroTime == 0)
                   ? 0.02f
                   : (now - lastGyroTime) / 1000.0f;
    lastGyroTime = now;

    if (abs(turn) > Config::DEADZONE) {
        headingLockActive = false;

        targetHeading += (turn / 127.0f) * Config::MAX_TURN_DEG_PER_S * dt;
        if (targetHeading >  180.0f) targetHeading -= 360.0f;
        if (targetHeading < -180.0f) targetHeading += 360.0f;

        gyro.setTargetHeading(targetHeading);
    } else {
        headingLockActive = true;
    }

    const float headingCorr = headingLockActive
                            ? gyro.getHeadingCorrection(Config::HEADING_CORR)
                            : 0.0f;
    const float tiltCorr    = gyro.getTiltCorrection(0.0f, 0.0f, Config::TILT_CORR);

    
    // static unsigned long lastDebug = 0;
    // if (now - lastDebug > 100) {
    //     lastDebug = now;
    //     Serial.printf("Pitch: %.1f° | Roll: %.1f° | Yaw: %.1f° | Target: %.1f°\n",
    //         gyro.getPitch(), gyro.getRoll(),
    //         gyro.getHeading(), targetHeading);
    // }

    return {headingCorr, tiltCorr};
}





//--------------------------------------------------------------------------------//

static void pushTelemetry(bool gamepad) {
   
    for (int i = 0; i < 4; i++) {
        if (!motors[i]) return;
    }

    
    // myData.fr_actual = motors[M_FR]->getActualRPM();
    // myData.fr_target = motors[M_FR]->getTargetRPM();
    // myData.fr_count  = motors[M_FR]->getTargetCount();
    // myData.fr_error  = myData.fr_target - myData.fr_actual;
    // myData.fr_pwm    = motors[M_FR]->getPWM(); 

    // myData.fl_actual = motors[M_FL]->getActualRPM();
    // myData.fl_target = motors[M_FL]->getTargetRPM();
    // myData.fl_count  = motors[M_FL]->getTargetCount();
    // myData.fl_error  = myData.fl_target - myData.fl_actual;
    // myData.fl_pwm    = motors[M_FL]->getPWM();

    // myData.br_actual = motors[M_BR]->getActualRPM();
    // myData.br_target = motors[M_BR]->getTargetRPM();
    // myData.br_count  = motors[M_BR]->getTargetCount();
    // myData.br_error  = myData.br_target - myData.br_actual;
    // myData.br_pwm    = motors[M_BR]->getPWM();

    // myData.bl_actual = motors[M_BL]->getActualRPM();
    // myData.bl_target = motors[M_BL]->getTargetRPM();
    // myData.bl_count  = motors[M_BL]->getTargetCount();
    // myData.bl_error  = myData.bl_target - myData.bl_actual;
    // myData.bl_pwm    = motors[M_BL]->getPWM();

    // myData.gamepadConnected = gamepad;
    // myData.stickLY = PS4.LStickY();
    // myData.stickLX = PS4.LStickX();
    // myData.stickRX = PS4.RStickX();
    // myData.battery = PS4.Battery();

    // esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    // // ----------------------------------------

    // Format: $actual,target per roda, dipisah koma, diakhiri newline
    Serial.printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d\n",
        motors[M_FR]->getActualRPM(), motors[M_FR]->getTargetRPM(), motors[M_FR]->getTargetCount(),motors[M_FR]->getTargetRPM() - motors[M_FR]->getActualRPM(), motors[M_FR] -> getPWM(),
        motors[M_FL]->getActualRPM(), motors[M_FL]->getTargetRPM(), motors[M_FL]->getTargetCount(), motors[M_FL]->getTargetRPM() - motors[M_FL]->getActualRPM(),motors[M_FL] -> getPWM(),
        motors[M_BR]->getActualRPM(), motors[M_BR]->getTargetRPM(), motors[M_BR]->getTargetCount(), motors[M_BR]->getTargetRPM() - motors[M_BR]->getActualRPM(), motors[M_BR] -> getPWM(),
        motors[M_BL]->getActualRPM(), motors[M_BL]->getTargetRPM(), motors[M_BL]->getTargetCount(), motors[M_BL]->getTargetRPM() - motors[M_BL]->getActualRPM(), motors[M_BL] -> getPWM(),
        gamepad,
        PS4.LStickY(), 
        PS4.LStickX(),
        PS4.RStickX(),
        PS4.Battery()
    );
}