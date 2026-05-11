#include <Arduino.h>
#include <PS4Controller.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace Config {
    constexpr const char* PS4_MAC      = "40:1A:58:62:D6:A2"; // mac address gamepad
    constexpr int     UART2_BAUD       = 115200;            // Uart2 baud
    constexpr int     UART2_RX         = 16;                // pin UART RX
    constexpr int     UART2_TX         = 17;                // pin UART TX
    constexpr uint8_t PKT_HEADER       = 0xAA;              // protokol keamanan 
    constexpr uint32_t COMMS_PERIOD_MS = 10;                // 100 Hz comms polling
    constexpr float   MAX_RPM          = 500.0f;            // max rpm (harus sama dengan slave)
}

struct __attribute__((packed)) RobotCmd {
    uint8_t  header;       // 0xAA
    float    targetX;       
    float    targetY;      
    float    targetOmega;  
    uint8_t  checksum;    
};

static uint8_t xorChecksum(const uint8_t* buf, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= buf[i];
    return cs;
}

// Shared state
static float gTargetX = 0.0f;
static float gTargetY = 0.0f;
static float gTargetO = 0.0f;
static bool  gConnected = false;
static SemaphoreHandle_t xMutex = nullptr;

static void sendRobotPacket(float x, float y, float omega) {
    RobotCmd pkt;
    pkt.header = Config::PKT_HEADER;
    pkt.targetX = x;
    pkt.targetY = y;
    pkt.targetOmega = omega;
    pkt.checksum = xorChecksum(reinterpret_cast<uint8_t*>(&pkt), sizeof(RobotCmd) - 1);
    
    Serial2.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(RobotCmd));
}

static void taskGamepad(void*) {
    TickType_t xLastWake = xTaskGetTickCount();
    bool wasConnected = false;

    for (;;) {
        const bool isConn = PS4.isConnected();
        
        if (wasConnected && !isConn) {
            Serial.println(F("[PS4] Disconnected!"));
        }
        if (!wasConnected && isConn) {
            Serial.println(F("[PS4] Connected!"));
        }
        wasConnected = isConn;

        if (isConn) {
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
                gTargetY = fwd * SCALE;
                gTargetX = str * SCALE;
                gTargetO = omg * SCALE;
                gConnected = true;
                xSemaphoreGive(xMutex);
            }
        } else {
            if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                gTargetX = 0; gTargetY = 0; gTargetO = 0;
                gConnected = false;
                xSemaphoreGive(xMutex);
            }
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(Config::COMMS_PERIOD_MS));
    }
}

static void taskUARTMaster(void*) {
    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        float tx, ty, to;
        bool conn;

        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            tx = gTargetX;
            ty = gTargetY;
            to = gTargetO;
            conn = gConnected;
            xSemaphoreGive(xMutex);
        }

        if (conn) {
            sendRobotPacket(tx, ty, to);
        } else {
            sendRobotPacket(0, 0, 0);
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(20)); // Kirim setiap 20ms (50Hz)
    }
}

void setup() {
    Serial.begin(115200);
    
    // Sesuai permintaan: jangan rubah nvs flash erase dan init
    nvs_flash_erase();
    nvs_flash_init();

    Serial2.begin(Config::UART2_BAUD, SERIAL_8N1, Config::UART2_RX, Config::UART2_TX);

    delay(2000);
    PS4.begin(Config::PS4_MAC);
    delay(1000);

    xMutex = xSemaphoreCreateMutex();
    
    xTaskCreatePinnedToCore(taskGamepad, "Gamepad", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(taskUARTMaster, "UARTMaster", 4096, nullptr, 3, nullptr, 1);
    
    Serial.println("ESP32 Main (Master) Ready");
}

void loop() { vTaskDelay(portMAX_DELAY); }
