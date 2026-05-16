/*
 * ==========================================================
 * ROBOT GATOT - GRAND UNIFIED CODE
 * Integrasi A1 (Mecanum/Kamera), A3 (Sensor Kompleks), 
 * dan Flowchart (Arena, Medan Forest, Martial Clue)
 * ==========================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_APDS9960.h> // Opsional: bisa untuk deteksi jarak dekat KFS

// ===== PIN MOTOR (Mecanum 4WD dari A1) =====
const int FL_PWM = 6;  const int FL_DIR = 7;
const int FR_PWM = 8;  const int FR_DIR = 9;
const int BL_PWM = 10; const int BL_DIR = 11;
const int BR_PWM = 12; const int BR_DIR = 13;

// ===== PIN SENSOR ULTRASONIK (4 Arah dari A3) =====
#define TRIG_FRONT 22
#define ECHO_FRONT 23
#define TRIG_BACK  24
#define ECHO_BACK  25
#define TRIG_LEFT  26
#define ECHO_LEFT  27
#define TRIG_RIGHT 28
#define ECHO_RIGHT 29

// ===== PIN SENSOR LAIN (Dari A1 & A3) =====
#define VIBRATION_PIN      34 // KW12-3 (Digital)
#define CURRENT_SENSOR_PIN 35 // ACS712 (Analog)
const int PIN_FOREST_LURUS = A0; // Kamera: Forest lurus
const int PIN_ADA_KOTAK    = A1; // Kamera: Ada kotak di atas forest
#define TOMBAK_SENSOR_PIN  A3    // Sensor deteksi tombak
#define LIFT_SENSOR_PIN    A4    // Sensor ketinggian lift

// ===== KONSTANTA & THRESHOLD =====
#define MOTOR_SPEED        150
#define MOTOR_SLOW         80
#define N_BLOCK_FOREST     4
#define MAX_RETRY_KFS      3
#define JARAK_DEKAT_RAK    20.0
#define OVERCURRENT_LIMIT  3.0   // Ampere

// ===== OBJEK SENSOR =====
MPU6050 mpu;
Adafruit_APDS9960 apds;

// ===== GLOBAL STATE =====
bool kfsTerdeteksi = false;
bool kfsTerambil   = false;
bool rakKiri       = false;
int  currentBlock  = 1;

// ==========================================================
//  CLASS ROBOTGATOT (Penggerak & Pembacaan Sensor)
// ==========================================================
class RobotGatot {
public:
    void init() {
        // Motor
        pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
        pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
        pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);
        pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);

        // Ultrasonik
        pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
        pinMode(TRIG_BACK,  OUTPUT); pinMode(ECHO_BACK,  INPUT);
        pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
        pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

        // Sensor Ekstra
        pinMode(VIBRATION_PIN, INPUT);
        pinMode(PIN_FOREST_LURUS, INPUT);
        pinMode(PIN_ADA_KOTAK, INPUT);
        pinMode(TOMBAK_SENSOR_PIN, INPUT);
    }

    // --- KONTROL MOTOR MECANUM ---
    void move(int fl, int fr, int bl, int br) {
        digitalWrite(FL_DIR, fl >= 0 ? HIGH : LOW); analogWrite(FL_PWM, abs(fl));
        digitalWrite(FR_DIR, fr >= 0 ? HIGH : LOW); analogWrite(FR_PWM, abs(fr));
        digitalWrite(BL_DIR, bl >= 0 ? HIGH : LOW); analogWrite(BL_PWM, abs(bl));
        digitalWrite(BR_DIR, br >= 0 ? HIGH : LOW); analogWrite(BR_PWM, abs(br));
    }
    void maju(int sp)        { move( sp,  sp,  sp,  sp); }
    void mundur(int sp)      { move(-sp, -sp, -sp, -sp); }
    void geserkanan(int sp)  { move( sp, -sp, -sp,  sp); }
    void geserkiri(int sp)   { move(-sp,  sp,  sp, -sp); }
    void putarkiri(int sp)   { move(-sp,  sp, -sp,  sp); }
    void putarkanan(int sp)  { move( sp, -sp,  sp, -sp); }
    void berhenti()          { move(0, 0, 0, 0); }

    // --- ROTASI DENGAN MPU6050 (Akurasi Tinggi) ---
    void sudut(float targetDerajat) {
        // Implementasi rotasi presisi menggunakan Gyro Z dari MPU6050
        // Jika target positif = Kanan, negatif = Kiri
        Serial.print("Berputar: "); Serial.println(targetDerajat);
        if(targetDerajat > 0) putarkanan(MOTOR_SPEED);
        else putarkiri(MOTOR_SPEED);
        
        delay(abs(targetDerajat) * 8); // Placeholder kalibrasi waktu (ganti dengan loop baca IMU)
        berhenti();
        delay(200);
    }

    // --- SENSOR ULTRASONIK ---
    float bacaUS(int trig, int echo) {
        digitalWrite(trig, LOW); delayMicroseconds(2);
        digitalWrite(trig, HIGH); delayMicroseconds(10);
        digitalWrite(trig, LOW);
        long dur = pulseIn(echo, HIGH, 30000);
        float dist = dur * 0.034f / 2.0f;
        return (dist == 0 || dist > 400) ? 400.0f : dist;
    }
    float depan()    { return bacaUS(TRIG_FRONT, ECHO_FRONT); }
    float belakang() { return bacaUS(TRIG_BACK,  ECHO_BACK); }
    float kiri()     { return bacaUS(TRIG_LEFT,  ECHO_LEFT); }
    float kanan()    { return bacaUS(TRIG_RIGHT, ECHO_RIGHT); }

    // --- SENSOR LAINNYA ---
    bool adaKFS() {
        // Bisa menggunakan APDS9960 Proximity atau Kamera dari A1
        return digitalRead(PIN_ADA_KOTAK) == HIGH; 
    }
    bool kameraLurus() { return digitalRead(PIN_FOREST_LURUS) == HIGH; }
    bool adaTombak()   { return digitalRead(TOMBAK_SENSOR_PIN) == HIGH; }
    float bacaArus() {
        int v = analogRead(CURRENT_SENSOR_PIN);
        return abs(((v / 4095.0f) * 3.3f - 2.5f) / 0.185f);
    }
    int ketinggianRak() { return analogRead(LIFT_SENSOR_PIN); }
};

RobotGatot gatot;

// ==========================================================
//  SUB-FUNGSI KFS (Dari Diagram)
// ==========================================================
bool toolsKFS() {
    Serial.println("[toolsKFS] Mencoba mengambil...");
    int retryCount = 0;
    while (retryCount < MAX_RETRY_KFS) {
        delay(500); // Aktuator capit KFS bekerja
        bool berhasil = true; // Ganti dengan sensor aktual (limit switch/berat)
        
        if (berhasil) {
            Serial.println("[toolsKFS] Berhasil!");
            kfsTerambil = true;
            return true;
        }
        retryCount++;
        Serial.println("[toolsKFS] Gagal, backup (mundur-maju)...");
        gatot.mundur(MOTOR_SLOW); delay(300);
        gatot.maju(MOTOR_SLOW); delay(300); gatot.berhenti();
    }
    Serial.println("[toolsKFS] Gagal ambil, KFS disimpan sbg false");
    kfsTerambil = false;
    return false;
}

void deteksiKFS() {
    Serial.println("[deteksiKFS] Mulai Scan 3 Arah...");
    if (gatot.adaKFS()) { kfsTerdeteksi = true; toolsKFS(); } // Depan
    
    gatot.sudut(-90); // R2 Rotasi Kiri
    if (gatot.adaKFS()) { kfsTerdeteksi = true; toolsKFS(); } // Kiri
    
    gatot.sudut(180); // R7 Rotasi Balik (Kanan)
    if (gatot.adaKFS()) { kfsTerdeteksi = true; toolsKFS(); } // Kanan

    if (kfsTerdeteksi && !kfsTerambil) toolsKFS(); // Coba lagi jika terdeteksi tapi gagal

    gatot.sudut(-90); // R2 Rotasi kembali ke depan
}

void kfsPulse() {
    Serial.println("[kfsPulse] Mulai...");
    gatot.geserkanan(MOTOR_SPEED); delay(500); gatot.berhenti();
    
    currentBlock = 1;
    bool diBlockTerakhir = false;

    while (!diBlockTerakhir) {
        deteksiKFS();
        if (currentBlock >= N_BLOCK_FOREST) {
            diBlockTerakhir = true;
            gatot.geserkiri(MOTOR_SPEED); delay(600); gatot.berhenti();
            deteksiKFS();
            gatot.maju(MOTOR_SPEED); delay(600); gatot.berhenti();
        } else {
            gatot.maju(MOTOR_SPEED); delay(600); gatot.berhenti();
            currentBlock++;
        }
    }
}

// ==========================================================
//  ALGORITMA UTAMA (Sesuai Flowchart)
// ==========================================================

void algoritmaArena() {
    Serial.println("=== FASE 1: ARENA ===");
    // Penentuan Sisi
    if (gatot.kiri() < 100) rakKiri = true;
    else if (gatot.kanan() < 100) rakKiri = false;

    for (int baris = 1; baris <= 2; baris++) {
        Serial.print("[Arena] Roda naik & deteksi baris: "); Serial.println(baris);
        delay(500); // Roda tengah naik
        
        int tinggi = gatot.ketinggianRak();
        Serial.print("[Arena] Tinggi rak: "); Serial.println(tinggi);

        Serial.println("[Arena] Bergerak menuju rak...");
        while (gatot.depan() > JARAK_DEKAT_RAK) {
            gatot.maju(MOTOR_SLOW); delay(50);
        }
        gatot.berhenti();
        
        Serial.println("[Arena] Taruh kotak..."); delay(1000); // Aktuator taruh kotak
        gatot.mundur(MOTOR_SPEED); delay(500); gatot.berhenti();
    }
}

void medanForest() {
    Serial.println("=== FASE 2: MEDAN FOREST ===");
    Serial.println("[Forest] Robot turun dari arena");
    gatot.mundur(MOTOR_SPEED); delay(800); gatot.berhenti();

    Serial.println("[Forest] Maju & deteksiKFS sampai blok terakhir");
    for (int i = 0; i < N_BLOCK_FOREST; i++) {
        gatot.maju(MOTOR_SLOW); delay(600); gatot.berhenti();
        deteksiKFS();
    }

    kfsPulse();

    Serial.println("[Forest] Evaluasi KFS Depan (Palsu/Biasa)");
    int kfsPalsuCount = 0;
    while (gatot.adaKFS()) {
        bool kfsPalsu = (gatot.kiri() < 15.0 || gatot.kanan() < 15.0); // Asumsi di pinggir = palsu
        if (kfsPalsu) {
            kfsPalsuCount++;
            gatot.maju(MOTOR_SLOW); delay(400); gatot.berhenti();
            deteksiKFS();
            if (kfsPalsuCount >= 2) {
                Serial.println("[Forest] 2x Gagal palsu -> Paksa Maju");
                gatot.maju(MOTOR_SPEED); delay(600); gatot.berhenti();
                deteksiKFS();
                kfsPalsuCount = 0;
            }
        } else {
            Serial.println("[Forest] KFS Biasa");
            gatot.maju(MOTOR_SLOW); delay(400); gatot.berhenti();
            deteksiKFS();
            kfsPalsuCount = 0;
        }
    }

    Serial.println("[Forest] Naik ke Forest menggunakan Kamera (Dari A1)");
    while (!gatot.kameraLurus()) {
        gatot.maju(100); delay(50); // Pelan mencari posisi lurus
    }
    gatot.berhenti();
    deteksiKFS();
    Serial.println("[Forest] Prodak arena selesai.");
}

void martialClue() {
    Serial.println("=== FASE 3: MARTIAL CLUE ===");
    Serial.println("[Martial] Maju ke rak mata tombak");
    while (!gatot.adaTombak()) {
        gatot.maju(MOTOR_SLOW); delay(50);
    }
    gatot.berhenti(); delay(300);

    Serial.println("[Martial] Bergerak monoton");
    gatot.maju(MOTOR_SLOW); delay(300); gatot.berhenti();

    Serial.println("[Martial] Berbalik 180 (Ultrasonik Aktif)");
    gatot.sudut(180);
    gatot.depan(); gatot.belakang(); gatot.kiri(); gatot.kanan(); // Aktifkan scan 4 arah

    Serial.println("[Martial] Menangkis..."); delay(500);

    Serial.println("[Martial] Putar 4x (Sena Baris 4 Tombak)");
    for(int i=0; i<4; i++) { gatot.sudut(90); }

    Serial.println("[Martial] Delay n detik (Sena masuk)");
    delay(3000);

    Serial.println("[Martial] Maju ke kotak 2");
    gatot.maju(MOTOR_SPEED); delay(800); gatot.berhenti();

    kfsTerdeteksi = false;
    deteksiKFS();
    if(kfsTerdeteksi) toolsKFS();
}

// ==========================================================
//  SETUP & LOOP
// ==========================================================
void setup() {
    Serial.begin(115200);
    Wire.begin();
    gatot.init();

    // Init IMU (Dari A3)
    mpu.initialize();
    if (mpu.testConnection()) {
        Serial.println("IMU Terkoneksi. Kalibrasi...");
        mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
    }
    
    // Init APDS9960 (Dari A3)
    if (apds.begin()) apds.enableProximity(true);

    delay(2000); // Jeda sebelum jalan
    
    // Eksekusi Flowchart
    algoritmaArena();
    medanForest();
    martialClue();
}

void loop() {
    // Pengamanan sistem berdasar A3 (Overcurrent Protection)
    if (gatot.bacaArus() > OVERCURRENT_LIMIT) {
        Serial.println("OVERCURRENT DETECTED! Mesin dihentikan.");
        gatot.berhenti();
        delay(3000);
    }
}