#include <Arduino.h>
#include <PS4Controller.h>  
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "../lib/movementLIB/gyroscope.h"
#include "../lib/movementLIB/movement.h"
namespace Config {
    constexpr bool    UART_MODE        = false;             // false: gamepad -> esp32
    constexpr bool    LOG              = true;              // print telemetri ke USB Serial
    constexpr bool    FIELD_ORIENTED   = false;             // field-oriented control
    constexpr float   PPR              = 134.4f * 7;        // total ppr
    constexpr float   MAX_RPM          = 500.0f;            // max rpm yang bisa di capai roda
    constexpr float   TILT_LIMIT       = 20.0f;             // batas error kemiringan
    constexpr float   DEADZONE_RPM     = 40;                // Batas RPM
    constexpr int     SDA_PIN          = 21;                // pin SDA
    constexpr int     SCL_PIN          = 22;                // pin SCL
    constexpr float   HEADING_KP       = 2.0f;              // Kp imu
    constexpr float   HEADING_KI       = 0.0f;              // Ki imu
    constexpr float   HEADING_KD       = 0.1f;              // Kd imu
    constexpr const char* PS4_MAC      = "40:1A:58:62:D6:A2"; // mac address gamepad
    constexpr int     UART2_BAUD       = 115200;            // Uart2 baud
    constexpr int     UART2_RX         = 16;                //pin UART RX
    constexpr int     UART2_TX         = 17;                // pin UART TX
    constexpr uint8_t PKT_HEADER       = 0xAA;              // protokol keamanan 
    constexpr uint32_t UART_TIMEOUT_MS = 500;               // watchdog: stop jika tak ada paket
    constexpr uint32_t CTRL_PERIOD_MS  = 10;                // 100 Hz control loop
    constexpr uint32_t COMMS_PERIOD_MS = 5;                 // 200 Hz comms polling
}


struct __attribute__((packed)) RobotCmd {
    uint8_t  header;       // 0xAA
    float    targetX;       
    float    targetY;      
    float    targetOmega;  
    uint8_t  checksum;    
};


static Mpu mpu(Config::SDA_PIN, Config::SCL_PIN,
               Config::HEADING_KP, Config::HEADING_KI, Config::HEADING_KD);

enum Motor : uint8_t { M_FR = 0, M_FL = 1, M_BR = 2, M_BL = 3 };
static Movement* motors[4] = {};

// Shared state akan dikunci jika diakses bersamaan
static SemaphoreHandle_t xMutex        = nullptr;
static float             gTargetX      = 0.0f;
static float             gTargetY      = 0.0f;  
static float             gTargetO      = 0.0f;  
static bool              gConnected    = false;
static uint32_t          gLastPktMs    = 0;
static float             headingCorr   = 0.0f;
static bool              wasConnected  = false;


static uint8_t xorChecksum(const uint8_t* buf, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= buf[i];
    return cs;
}


static void stopAll() {
    for (auto& m : motors) if (m) m->update(0.0f);
}

static void resetAllPID() {
    for (auto& m : motors) if (m) m->resetPID();
}


static void applyMovement(float forward, float strafe, float omega) {
    if (fabsf(forward) < Config::DEADZONE_RPM &&
        fabsf(strafe)  < Config::DEADZONE_RPM &&
        fabsf(omega)   < Config::DEADZONE_RPM) {
        headingCorr = 0.0f;
        mpu.setheading(mpu.outputYaw());
        resetAllPID();
        stopAll();
        return;
    }

    // Heading lock / correction
    if (fabsf(omega) > Config::DEADZONE_RPM) {
        mpu.setheading(mpu.outputYaw());
        headingCorr = 0.0f;
    } else {headingCorr = mpu.CorectionHeading();}

    // Field-oriented 
    if constexpr (Config::FIELD_ORIENTED) {
        if (mpu.isReady()) {
            const float theta = mpu.outputYaw() * DEG_TO_RAD;
            const float cosT  = cosf(theta);
            const float sinT  = sinf(theta);
            const float xb    =  strafe  * cosT + forward * sinT;
            const float yb    = -strafe  * sinT + forward * cosT;
            strafe  = xb;
            forward = yb;
        }
    }
    // persamaan matematis omniwheel + penyesuaian
    float vfl =   forward + strafe + omega + headingCorr;
    float vfr = -(forward - strafe - omega - headingCorr);
    float vbl =   forward - strafe + omega + headingCorr;
    float vbr = -(forward + strafe - omega - headingCorr);

    
    // Penormalisasian 
    const float maxV = max({fabsf(vfl), fabsf(vfr), fabsf(vbl), fabsf(vbr)});
    if (maxV > Config::MAX_RPM) {
        const float s = Config::MAX_RPM / maxV;
        vfl *= s; vfr *= s; vbl *= s; vbr *= s;
    }

    motors[M_FL]->update(vfl); motors[M_FR]->update(vfr); motors[M_BL]->update(vbl); motors[M_BR]->update(vbr);
}

//  Selalu print ke USB Serial dan jika uart mode kirim ke serial 2
static void pushTelemetry(bool connected) {
    for (auto& m : motors) if (!m) return;

    int battery = 0;
    if constexpr (!Config::UART_MODE) {
        if (connected) battery = (int)PS4.Battery();
    }

    char buf[256];
    const int n = snprintf(buf, sizeof(buf),
        // FR: actual, target, error, pwm
        "%.1f,%.1f,%.1f,%.0f,"
        // FL
        "%.1f,%.1f,%.1f,%.0f,"
        // BR
        "%.1f,%.1f,%.1f,%.0f,"
        // BL
        "%.1f,%.1f,%.1f,%.0f,"
        // connected, battery
        "%d,%d,"
        // yaw, pitch, roll, gyroReady, dmpReady
        "%.1f,%.1f,%.1f,%d,%d,"
        // encoder synced per roda (error < 50 RPM)
        "%d,%d,%d,%d\n",

        motors[M_FR]->getActualRPM(), motors[M_FR]->getTargetRPM(),
        fabsf(motors[M_FR]->getTargetRPM() - motors[M_FR]->getActualRPM()),
        fabsf(motors[M_FR]->getPWM()),

        motors[M_FL]->getActualRPM(), motors[M_FL]->getTargetRPM(),
        fabsf(motors[M_FL]->getTargetRPM() - motors[M_FL]->getActualRPM()),
        fabsf(motors[M_FL]->getPWM()),

        motors[M_BR]->getActualRPM(), motors[M_BR]->getTargetRPM(),
        fabsf(motors[M_BR]->getTargetRPM() - motors[M_BR]->getActualRPM()),
        fabsf(motors[M_BR]->getPWM()),

        motors[M_BL]->getActualRPM(), motors[M_BL]->getTargetRPM(),
        fabsf(motors[M_BL]->getTargetRPM() - motors[M_BL]->getActualRPM()),
        fabsf(motors[M_BL]->getPWM()),

        (int)connected, battery,
        mpu.outputYaw(), mpu.outputPitch(), mpu.outputRoll(),
        (int)mpu.isGyroReady(), (int)mpu.isReady(),

        (int)(fabsf(motors[M_FR]->getTargetRPM() - motors[M_FR]->getActualRPM()) < 50.0f),
        (int)(fabsf(motors[M_FL]->getTargetRPM() - motors[M_FL]->getActualRPM()) < 50.0f),
        (int)(fabsf(motors[M_BR]->getTargetRPM() - motors[M_BR]->getActualRPM()) < 50.0f),
        (int)(fabsf(motors[M_BL]->getTargetRPM() - motors[M_BL]->getActualRPM()) < 50.0f)
    );

    if (n <= 0 || n >= (int)sizeof(buf)) return;
    Serial.print(buf);                            
    if constexpr (Config::UART_MODE) {
        Serial2.write(reinterpret_cast<uint8_t*>(buf), n); // kirim ke Master
    }
}


static bool readRobotPacket(RobotCmd& out) {
    constexpr size_t N = sizeof(RobotCmd);

    while (Serial2.available() >= (int)N) {
        if ((uint8_t)Serial2.peek() != Config::PKT_HEADER) {
            Serial2.read();
            continue;
        }
        uint8_t buf[N];
        if (Serial2.readBytes(buf, N) != N) break;

         uint8_t cs = xorChecksum(buf, N - 1);
        if (cs != buf[N - 1]) continue;
            memcpy(&out, buf, N);
            return true;

    }
       
    return false;
}
// digunakan untuk tuning dari serial studio
static void handlePIDSerial() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (!cmd.startsWith("PID,")) return;

    const int c1 = cmd.indexOf(',');
    const int c2 = cmd.indexOf(',', c1 + 1);
    const int c3 = cmd.indexOf(',', c2 + 1);
    const int c4 = cmd.indexOf(',', c3 + 1);

    const String idxStr = cmd.substring(c1 + 1, c2);
    const float  kp     = cmd.substring(c2 + 1, c3).toFloat();
    const float  ki     = cmd.substring(c3 + 1, c4).toFloat();
    const float  kd     = cmd.substring(c4 + 1).toFloat();

    static const char* names[] = {"FR", "FL", "BR", "BL"};

    auto apply = [&](int i) {
        if (i >= 0 && i < 4 && motors[i]) {
            motors[i]->setPID(kp, ki, kd);
            Serial.printf("[PID] %s → Kp=%.3f Ki=%.3f Kd=%.3f\n",
                          names[i], kp, ki, kd);
        }
    };

    if (idxStr == "*") { for (int i = 0; i < 4; i++) apply(i); }
    else               { apply(constrain(idxStr.toInt(), 0, 3)); }
}

//core 1
static void taskControl(void*) {
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        mpu.update();
        float fwd, str, omg;
        bool  conn;

        // fwd, str, omg,conn dimasukkan ke semaphore yang dilindungi xmutex
        // ambil data max 2ms jika tidak conn = false
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2))) {
            fwd  = gTargetY;
            str  = gTargetX;
            omg  = gTargetO;
            conn = gConnected;
            xSemaphoreGive(xMutex);
        } else {
            conn = false;  
        }

        if (mpu.isReady() && mpu.getTilt() > Config::TILT_LIMIT) {
            stopAll();
        } else if (conn) {
            applyMovement(fwd, str, omg);
        } else {
            stopAll();
        }

        if constexpr (Config::LOG) {
            pushTelemetry(conn);
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(Config::CTRL_PERIOD_MS));
    }
}

// core 0
static void taskComms(void*) {
    TickType_t xLastWake = xTaskGetTickCount(); // 

    for (;;) {
        handlePIDSerial();
        // dijalankan hanya untuk UART
        if constexpr (Config::UART_MODE) {
            RobotCmd pkt;
            if (readRobotPacket(pkt)) {
                if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                    gTargetX   = pkt.targetX;
                    gTargetY   = pkt.targetY;
                    gTargetO   = pkt.targetOmega;
                    gConnected = true;
                    xSemaphoreGive(xMutex);
                }
                gLastPktMs = millis();
            }

            if ((millis() - gLastPktMs) > Config::UART_TIMEOUT_MS) {
                if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                    gConnected = false;
                    xSemaphoreGive(xMutex);
                }
            }

        // normal mode
        } else {
            const bool isConn = PS4.isConnected();

            // Disconnect -> restart
            if (wasConnected && !isConn) {
                if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                    gConnected = false;
                    xSemaphoreGive(xMutex);
                }
                vTaskDelay(pdMS_TO_TICKS(200));
                ESP.restart();
            }
            wasConnected = isConn;

            if (isConn) {
                if (PS4.Triangle()) Serial.println(F("[BTN] Triangle"));
                if (PS4.Square())   Serial.println(F("[BTN] Square"));
                if (PS4.Circle())   Serial.println(F("[BTN] Circle"));
                float fwd = (float)PS4.LStickY();
                float str = (float)PS4.LStickX();
                float omg = (float)PS4.RStickX();

                if (PS4.Up())      { fwd =  100.0f; str =    0.0f; omg = 0.0f; }
                if (PS4.Down())    { fwd = -100.0f; str =    0.0f; omg = 0.0f; }
                if (PS4.Right())   { fwd =    0.0f; str =  100.0f; omg = 0.0f; }
                if (PS4.Left())    { fwd =    0.0f; str = -100.0f; omg = 0.0f; }
                if (PS4.UpRight()) { fwd =  100.0f; str =  100.0f; omg = 0.0f; }

                // Konversi stik → RPM
                constexpr float SCALE = Config::MAX_RPM / 128.0f;

                if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                    gTargetY   = fwd * SCALE;
                    gTargetX   = str * SCALE;
                    gTargetO   = omg * SCALE;
                    gConnected = true;
                    xSemaphoreGive(xMutex);
                }
                } else {
                    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                        gConnected = false;
                        xSemaphoreGive(xMutex);
                    }
                }
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(Config::COMMS_PERIOD_MS));
    }
}


void setup() {
    Serial.begin(115200);
    nvs_flash_erase();
    nvs_flash_init();

    Serial2.begin(Config::UART2_BAUD, SERIAL_8N1,
                  Config::UART2_RX, Config::UART2_TX);

    if constexpr (!Config::UART_MODE) {
        delay(2000);
        PS4.begin(Config::PS4_MAC);
        delay(1000);
    }

    mpu.init();
    if (mpu.isReady()) mpu.measureDrift(3000);

    // Motor                    name  Kp      Ki    Kd    chA  chB  ppr        rpwm  lpwm
    motors[M_FR] = new Movement("Fr", 1.5f , 0.0f, 0.0f,  36,  39, Config::PPR,  27,  14);
    motors[M_FL] = new Movement("Fl", 0.0f , 0.0f, 0.0f,  34,  35, Config::PPR,  13,  23);
    motors[M_BR] = new Movement("Br", 0.0f , 0.0f, 0.0f,  25,  26, Config::PPR,  17,  16);
    motors[M_BL] = new Movement("Bl", 0.0f , 0.0f, 0.0f,  32,  33, Config::PPR,  19,  18);
    for (auto& m : motors) if (m) m->begin();


    xMutex = xSemaphoreCreateMutex();
    configASSERT(xMutex);

    xTaskCreatePinnedToCore(taskControl, "Control", 8192, nullptr, 5, nullptr, 1); // control [core 1]
    xTaskCreatePinnedToCore(taskComms,   "Comms",   4096, nullptr, 3, nullptr, 0); // log     [core 0]
}

void loop() { vTaskDelay(portMAX_DELAY); }