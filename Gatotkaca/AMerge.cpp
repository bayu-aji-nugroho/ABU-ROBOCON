/*
 * ==========================================================
 *  ROBOT GATOT - Kode Terintegrasi
 *  Sesuai diagram: Arena | Medan Forest | Martial Clue
 *  Sub-fungsi: kfsPulse() | toolsKFS() | deteksiKFS()
 * ==========================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// ===== PIN MOTOR (mecanum 4WD) =====
// FL=Front Left, FR=Front Right, BL=Back Left, BR=Back Right
const int FL_PWM = 6;  const int FL_DIR = 7;
const int FR_PWM = 8;  const int FR_DIR = 9;
const int BL_PWM = 10; const int BL_DIR = 11;
const int BR_PWM = 12; const int BR_DIR = 13;

// ===== PIN SENSOR ULTRASONIK (4 arah) =====
#define TRIG_FRONT 22
#define ECHO_FRONT 23
#define TRIG_BACK  24
#define ECHO_BACK  25
#define TRIG_LEFT  26
#define ECHO_LEFT  27
#define TRIG_RIGHT 28
#define ECHO_RIGHT 29

// ===== PIN SENSOR LAIN =====
#define VIBRATION_PIN      34   // KW12-3  – digital
#define CURRENT_SENSOR_PIN 35   // ACS712  – analog
#define KFS_SENSOR_PIN     A2   // sensor deteksi KFS
#define TOMBAK_SENSOR_PIN  A3   // sensor deteksi mata tombak
#define LIFT_SENSOR_PIN    A4   // sensor ketinggian lift/rak

// ===== KONSTANTA =====
#define MOTOR_SPEED        150
#define MOTOR_SLOW         80
#define OBSTACLE_DIST      30.0   // cm
#define KFS_THRESHOLD      500    // nilai analog
#define MAX_RETRY          3      // maksimum percobaan ambil KFS
#define JARAK_RAK          100.0  // cm – ambang deteksi rak
#define DELAY_1_BLOCK      600    // ms – waktu maju 1 blok (kalibrasi)
#define DELAY_GESER        500    // ms – waktu geser 1 blok
#define N_BLOCK_FOREST     4      // jumlah blok di medan forest
#define DELAY_SENA         3000   // ms – tunggu sena masuk arena
#define ACS712_SENSITIVITY 0.185
#define ACS712_OFFSET      2.5
#define OVERCURRENT_LIMIT  3.0    // Ampere

// ===== SENSOR IMU =====
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

// ===== GLOBAL STATE =====
bool kfsTerdeteksi  = false;
bool kfsTerambil    = false;
bool rakKiri        = false;
int  currentBlock   = 1;

// ==========================================================
//  CLASS ROBOTGATOT
//  Menggabungkan gerakan dasar A1 + sensor 4 arah A3
// ==========================================================
class RobotGatot {
public:
    // ----- Init -----
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

        // Sensor lain
        pinMode(VIBRATION_PIN, INPUT);
        pinMode(KFS_SENSOR_PIN, INPUT);
        pinMode(TOMBAK_SENSOR_PIN, INPUT);
        pinMode(LIFT_SENSOR_PIN, INPUT);
    }

    // ----- Kontrol motor dasar -----
    void move(int fl, int fr, int bl, int br) {
        digitalWrite(FL_DIR, fl >= 0 ? HIGH : LOW);
        analogWrite(FL_PWM, abs(fl));
        digitalWrite(FR_DIR, fr >= 0 ? HIGH : LOW);
        analogWrite(FR_PWM, abs(fr));
        digitalWrite(BL_DIR, bl >= 0 ? HIGH : LOW);
        analogWrite(BL_PWM, abs(bl));
        digitalWrite(BR_DIR, br >= 0 ? HIGH : LOW);
        analogWrite(BR_PWM, abs(br));
    }

    void maju(int sp)        { move( sp,  sp,  sp,  sp); }
    void mundur(int sp)      { move(-sp, -sp, -sp, -sp); }
    void geserkanan(int sp)  { move( sp, -sp, -sp,  sp); }
    void geserkiri(int sp)   { move(-sp,  sp,  sp, -sp); }
    void putarkiri(int sp)   { move(-sp,  sp, -sp,  sp); }
    void putarkanan(int sp)  { move( sp, -sp,  sp, -sp); }
    void berhenti()          { move(0, 0, 0, 0); }

    // ----- Rotasi ke sudut (berbasis delay dikalibrasi) -----
    // Positif = kanan, negatif = kiri
    void sudut(int derajat) {
        int sp = MOTOR_SPEED;
        if (derajat == 0) return;

        if (derajat > 0) {
            putarkanan(sp);
        } else {
            putarkiri(sp);
            derajat = -derajat;
        }
        // ~8 ms per derajat pada speed 150 — kalibrasi manual
        delay((unsigned long)derajat * 8);
        berhenti();
        delay(200);
    }

    // ----- Naik 20cm + jalan ke tengah (Medan Forest) -----
    void naik() {
        maju(MOTOR_SPEED);
        delay(800);   // ~20cm — kalibrasi manual
        berhenti();
        delay(200);
        maju(MOTOR_SLOW);
        delay(400);   // align tengah
        berhenti();
        Serial.println("[naik] selesai");
    }

    // ----- Turun 20cm + jalan ke tengah -----
    void turun() {
        mundur(MOTOR_SPEED);
        delay(800);
        berhenti();
        delay(200);
        mundur(MOTOR_SLOW);
        delay(400);
        berhenti();
        Serial.println("[turun] selesai");
    }

    // ----- Baca ultrasonik single -----
    float bacaUS(int trig, int echo) {
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        long dur = pulseIn(echo, HIGH, 30000);
        float dist = dur * 0.034f / 2.0f;
        return (dist == 0 || dist > 400) ? 400.0f : dist;
    }

    float depan()     { return bacaUS(TRIG_FRONT, ECHO_FRONT); }
    float belakang()  { return bacaUS(TRIG_BACK,  ECHO_BACK);  }
    float kiri()      { return bacaUS(TRIG_LEFT,  ECHO_LEFT);  }
    float kanan()     { return bacaUS(TRIG_RIGHT, ECHO_RIGHT); }

    // ----- Sensor KFS & Tombak -----
    bool adaKFS()     { return analogRead(KFS_SENSOR_PIN) > KFS_THRESHOLD; }
    bool adaTombak()  { return digitalRead(TOMBAK_SENSOR_PIN) == HIGH; }

    // ----- Sensor arus (ACS712) -----
    float bacaArus() {
        int v = analogRead(CURRENT_SENSOR_PIN);
        float volt = (v / 4095.0f) * 3.3f;
        return abs((volt - ACS712_OFFSET) / ACS712_SENSITIVITY);
    }

    // ----- Deteksi ketinggian rak (lift sensor analog) -----
    int ketinggianRak() {
        return analogRead(LIFT_SENSOR_PIN);
    }
};

RobotGatot gatot;

// ==========================================================
//  tools_kfs()
//  Diagram: kfs terdeteksi → coba ambil
//           gagal n kali → backup
//           simpan kfs → kirim variabel kfs terambil
// ==========================================================
bool toolsKFS() {
    Serial.println("[toolsKFS] Mulai...");
    int retryCount = 0;

    while (retryCount < MAX_RETRY) {
        Serial.print("[toolsKFS] Percobaan ke-");
        Serial.println(retryCount + 1);

        // === AKTUATOR AMBIL KFS ===
        // Isi dengan perintah servo/claw sesuai hardware
        // contoh: digitalWrite(CLAW_PIN, HIGH); delay(500);
        delay(500); // placeholder

        // Cek keberhasilan (limit switch / sensor berat)
        // Ganti 'true' dengan kondisi sensor aktual
        bool berhasil = true; // TODO: ganti dengan sensor aktual

        if (berhasil) {
            Serial.println("[toolsKFS] KFS berhasil diambil!");
            kfsTerambil = true;   // kirim variabel kfs sudah terambil
            return true;
        }

        // Jika gagal: backup lalu coba lagi
        retryCount++;
        Serial.println("[toolsKFS] Gagal, backup...");
        gatot.mundur(MOTOR_SLOW);
        delay(300);
        gatot.maju(MOTOR_SLOW);
        delay(300);
        gatot.berhenti();
    }

    // Simpan status meski gagal (kfs disimpan)
    Serial.println("[toolsKFS] KFS disimpan (gagal setelah max retry)");
    kfsTerambil = false;
    return false;
}

// ==========================================================
//  deteksi_kfs()
//  Diagram:
//    deteksi kfs depan → tools_kfs() → R2 berotasi -90°
//    deteksi kfs kiri  → R7 berotasi 180° → tools_kfs()
//    deteksi kfs kanan → tools_kfs()
//    R2 berotasi -90° (kembali ke orientasi awal)
// ==========================================================
void deteksiKFS() {
    Serial.println("[deteksiKFS] Mulai...");

    // 1. Deteksi KFS depan
    if (gatot.adaKFS()) {
        Serial.println("[deteksiKFS] KFS depan terdeteksi!");
        kfsTerdeteksi = true;
        toolsKFS();
    }

    // R2 berotasi -90° (putar kiri 90°)
    gatot.sudut(-90);

    // 2. Deteksi KFS kiri
    if (gatot.adaKFS()) {
        Serial.println("[deteksiKFS] KFS kiri terdeteksi!");
        kfsTerdeteksi = true;
        toolsKFS();
    }

    // R7 berotasi 180°
    gatot.sudut(180);

    // 3. Deteksi KFS kanan
    if (gatot.adaKFS()) {
        Serial.println("[deteksiKFS] KFS kanan terdeteksi!");
        kfsTerdeteksi = true;
        toolsKFS();
    }

    // Jika terdeteksi tapi belum diambil, coba lagi
    if (kfsTerdeteksi && !kfsTerambil) {
        toolsKFS();
    }

    // R2 berotasi -90° → kembali ke orientasi semula
    gatot.sudut(-90);

    Serial.println("[deteksiKFS] Selesai.");
}

// ==========================================================
//  kfs_pulse()
//  Diagram:
//    start → bergerak ke kanan
//    loop: deteksi kfs() → maju 1 blok
//          apakah di blok terakhir?
//            ya → kekiri 1 blok → deteksi kfs() → maju 1 blok → end
//            tidak → ulangi
// ==========================================================
void kfsPulse() {
    Serial.println("[kfsPulse] Mulai...");

    // Bergerak ke kanan (posisi awal scan)
    gatot.geserkanan(MOTOR_SPEED);
    delay(DELAY_GESER);
    gatot.berhenti();
    delay(200);

    currentBlock = 1;
    bool diBlockTerakhir = false;

    while (!diBlockTerakhir) {
        // Deteksi KFS di posisi saat ini
        deteksiKFS();

        // Cek blok terakhir
        if (currentBlock >= N_BLOCK_FOREST) {
            diBlockTerakhir = true;
            Serial.println("[kfsPulse] Di blok terakhir!");

            // Kekiri 1 blok
            gatot.geserkiri(MOTOR_SPEED);
            delay(DELAY_1_BLOCK);
            gatot.berhenti();
            delay(200);

            // Deteksi KFS sekali lagi
            deteksiKFS();

            // Maju 1 blok
            gatot.maju(MOTOR_SPEED);
            delay(DELAY_1_BLOCK);
            gatot.berhenti();
            delay(200);

        } else {
            // Belum di blok terakhir: maju 1 blok
            gatot.maju(MOTOR_SPEED);
            delay(DELAY_1_BLOCK);
            gatot.berhenti();
            delay(200);
            currentBlock++;
            Serial.print("[kfsPulse] Maju ke blok: ");
            Serial.println(currentBlock);
        }
    }

    Serial.println("[kfsPulse] Selesai.");
}

// ==========================================================
//  MEDAN FOREST
//  Diagram:
//    robot keluar/turun arena
//    → [FASE 1] maju + deteksiKFS() sampai blok terakhir
//    → [FASE 2] jalankan kfsPulse()
//    → [FASE 3] LOOP: apakah depan ada KFS?
//         kfs palsu  → maju + deteksiKFS()
//                      jika program tidak mendeteksi kfs palsu:
//                        kfs palsu berada di pinggir
//                      jika 2x palsu tidak terdeteksi → maju + deteksiKFS()
//         kfs biasa  → maju + deteksiKFS()
//         tidak ada  → keluar loop
//    → [FASE 4] naik, lalu deteksiKFS()
//    → prodak arena (selesai)
// ==========================================================
void medanForest() {
    Serial.println("=== MEDAN FOREST ===");

    // ── FASE 1: Robot keluar / turun dari arena ──
    gatot.turun();
    delay(500);

    // ── FASE 2: Maju + deteksiKFS() sampai blok terakhir ──
    Serial.println("[Forest] Fase 1: Scan per blok...");
    for (int i = 0; i < N_BLOCK_FOREST; i++) {
        gatot.maju(MOTOR_SLOW);
        delay(DELAY_1_BLOCK);
        gatot.berhenti();
        delay(200);
        deteksiKFS();
        Serial.print("[Forest] Blok "); Serial.print(i + 1);
        Serial.println(" selesai.");
    }

    // ── FASE 3: Jalankan kfsPulse() ──
    Serial.println("[Forest] Fase 2: kfsPulse...");
    kfsPulse();

    // ── FASE 4: Loop penanganan KFS depan (palsu / biasa) ──
    //           SELESAI DULU sebelum naik
    Serial.println("[Forest] Fase 3: Cek KFS depan...");
    int kfsPalsuCount = 0;

    // Loop sampai tidak ada KFS di depan
    while (true) {
        bool adaKFSDepan = gatot.adaKFS();

        if (!adaKFSDepan) {
            // Tidak ada KFS → keluar loop, lanjut ke fase naik
            Serial.println("[Forest] Tidak ada KFS depan, lanjut naik.");
            break;
        }

        // Ada KFS → tentukan palsu atau biasa
        // KFS palsu: terdeteksi di pinggir (dekat dinding kiri/kanan)
        bool kfsPalsu = (gatot.kiri() < 15.0 || gatot.kanan() < 15.0);

        if (kfsPalsu) {
            kfsPalsuCount++;
            Serial.print("[Forest] KFS Palsu ke-"); Serial.println(kfsPalsuCount);

            // KFS palsu berada di pinggir → tetap maju pelan + deteksiKFS
            gatot.maju(MOTOR_SLOW);
            delay(400);
            gatot.berhenti();
            deteksiKFS();

            // Jika 2x kfs palsu tidak berhasil terdeteksi/diambil
            // → paksa maju 1 blok penuh + deteksiKFS
            if (kfsPalsuCount >= 2) {
                Serial.println("[Forest] 2x palsu → maju paksa 1 blok.");
                gatot.maju(MOTOR_SPEED);
                delay(DELAY_1_BLOCK);
                gatot.berhenti();
                deteksiKFS();
                kfsPalsuCount = 0; // reset counter setelah paksa
            }

        } else {
            // KFS biasa
            Serial.println("[Forest] KFS Biasa → maju + deteksiKFS.");
            gatot.maju(MOTOR_SLOW);
            delay(400);
            gatot.berhenti();
            deteksiKFS();
            kfsPalsuCount = 0; // reset karena ketemu biasa
        }

        delay(100);
    }

    // ── FASE 5: Naik 20cm, lalu deteksiKFS() ──
    //           Baru dilakukan setelah semua KFS depan ditangani
    Serial.println("[Forest] Fase 4: Naik + deteksiKFS...");
    gatot.naik();
    deteksiKFS();

    // ── Prodak arena ──
    Serial.println("[Forest] Prodak arena tercapai!");
    Serial.println("=== MEDAN FOREST SELESAI ===");
}

// ==========================================================
//  ARENA
//  Diagram:
//    [LOOP per baris rak]
//      roda tengah naik + deteksi jarak
//      → deteksi ketinggian
//      → bergerak menuju rak (dekati sena)
//      → rak baris ke ? (cek apakah masih ada baris)
//          masih ada → taruh kotak → mundur → naik ke baris berikut → ulangi
//          habis     → selesai
// ==========================================================

// Jumlah baris rak yang harus diisi
#define JUMLAH_BARIS_RAK   2
// Ambang jarak "sudah dekat ke rak / sena"
#define JARAK_DEKAT_RAK   20.0   // cm
// Tinggi lift per baris (nilai analog threshold)
#define TINGGI_BARIS_1    300
#define TINGGI_BARIS_2    600

void taruKotak() {
    // === AKTUATOR TARUH KOTAK ===
    // Ganti dengan perintah servo/lift sesuai hardware:
    // contoh: digitalWrite(LIFT_PIN, HIGH); delay(800);
    //         digitalWrite(LIFT_PIN, LOW);
    Serial.println("[Arena] Taruh kotak...");
    delay(1000); // placeholder
}

void naikKeBaris(int baris) {
    // === AKTUATOR NAIK KE BARIS BERIKUT ===
    // Sesuaikan dengan mekanisme lift/elevator robot:
    Serial.print("[Arena] Naik ke baris "); Serial.println(baris);
    delay(800); // placeholder
}

void algoritmaArena() {
    Serial.println("=== ARENA ===");

    // ── Deteksi sisi rak (kiri / kanan) hanya sekali di awal ──
    float jKiri  = gatot.kiri();
    float jKanan = gatot.kanan();

    if (jKiri < JARAK_RAK) {
        rakKiri = true;
        Serial.println("[Arena] Rak di kiri");
    } else if (jKanan < JARAK_RAK) {
        rakKiri = false;
        Serial.println("[Arena] Rak di kanan");
    } else {
        Serial.println("[Arena] Rak tidak terdeteksi, berhenti.");
        gatot.berhenti();
        return;
    }

    // ── LOOP per baris rak ──
    for (int baris = 1; baris <= JUMLAH_BARIS_RAK; baris++) {
        Serial.print("[Arena] === Baris rak ke-"); Serial.print(baris);
        Serial.println(" ===");

        // STEP 1: Roda tengah naik + deteksi jarak semua arah
        float jDepan = gatot.depan();
        float jBelakang = gatot.belakang();
        Serial.print("[Arena] Jarak depan: "); Serial.print(jDepan);
        Serial.print(" cm | belakang: "); Serial.print(jBelakang);
        Serial.println(" cm");

        // STEP 2: Deteksi ketinggian rak
        int tinggiRak = gatot.ketinggianRak();
        Serial.print("[Arena] Ketinggian rak baris "); Serial.print(baris);
        Serial.print(": "); Serial.println(tinggiRak);

        // Validasi ketinggian sesuai baris
        int targetTinggi = (baris == 1) ? TINGGI_BARIS_1 : TINGGI_BARIS_2;
        if (abs(tinggiRak - targetTinggi) > 100) {
            Serial.println("[Arena] Ketinggian tidak sesuai, koreksi...");
            // Sesuaikan lift ke ketinggian target (implementasi hardware)
            delay(500); // placeholder koreksi lift
        }

        // STEP 3: Bergerak menuju rak (dekati sena) sampai jarak aman
        Serial.println("[Arena] Bergerak menuju rak...");
        while (gatot.depan() > JARAK_DEKAT_RAK) {
            gatot.maju(MOTOR_SLOW);
            delay(80);
        }
        gatot.berhenti();
        Serial.println("[Arena] Sudah dekat rak.");

        // STEP 4: Roda tengah di posisi rak → taruh kotak
        taruKotak();

        // STEP 5: Mundur setelah taruh kotak
        gatot.mundur(MOTOR_SPEED);
        delay(600);
        gatot.berhenti();

        // STEP 6: Cek apakah masih ada baris berikutnya
        if (baris < JUMLAH_BARIS_RAK) {
            // Masih ada baris → naik ke baris berikut lalu ulangi
            naikKeBaris(baris + 1);
            delay(300);
        } else {
            // Semua baris sudah diisi
            Serial.println("[Arena] Semua baris rak selesai.");
        }
    }

    Serial.println("=== ARENA SELESAI ===");
}

// ==========================================================
//  MARTIAL CLUE
//  Diagram:
//    start → maju ke depan rak mata tombak
//    → deteksi mata tombak → tunggu sampai terdeteksi
//    → bergerak monoton
//    → berbalik 180° + pakai sensor ultrasonic 4 sisi
//    → menangkis mata tombak
//    → putar 4x (sena baris 4 tombak)
//    → berhenti → delay n detik (tunggu sena masuk arena)
//    → bergerak ke depan kotak 2
//    → deteksi kfs di kotak 2
//         jika ada → tools kfs dijalankan
//         jika tidak ada → lanjut
// ==========================================================
void martialClue() {
    Serial.println("=== MARTIAL CLUE ===");

    // Maju ke depan rak mata tombak
    Serial.println("[Martial] Maju ke rak mata tombak...");
    while (!gatot.adaTombak()) {
        gatot.maju(MOTOR_SLOW);
        delay(100);
    }
    gatot.berhenti();
    Serial.println("[Martial] Mata tombak terdeteksi!");

    // Tunggu sampai terdeteksi dengan konfirmasi
    delay(300);

    // Bergerak monoton (posisi siap menangkis)
    gatot.maju(MOTOR_SLOW);
    delay(300);
    gatot.berhenti();

    // Berbalik 180° (sena ada di belakang)
    // Sambil sensor ultrasonic 4 sisi aktif
    Serial.println("[Martial] Berbalik 180°..."); 
    {
        float f = gatot.depan();
        float b = gatot.belakang();
        float l = gatot.kiri();
        float r = gatot.kanan();
        Serial.print("US 4 sisi → D:"); Serial.print(f);
        Serial.print(" B:"); Serial.print(b);
        Serial.print(" Ki:"); Serial.print(l);
        Serial.print(" Ka:"); Serial.println(r);
    }
    gatot.sudut(180);

    // Menangkis mata tombak
    Serial.println("[Martial] Menangkis mata tombak...");
    // Implementasi mekanisme menangkis sesuai hardware:
    // contoh: analogWrite(TANGKIS_PIN, 200); delay(500);
    delay(500); // placeholder

    // Putar 4x: sena baris 4 tombak
    for (int i = 0; i < 4; i++) {
        gatot.sudut(90);
        delay(300);
        Serial.print("[Martial] Putaran ke-"); Serial.println(i + 1);
    }
    gatot.berhenti();

    // Delay n detik menunggu sena masuk arena
    Serial.println("[Martial] Menunggu sena masuk arena...");
    delay(DELAY_SENA);

    // Bergerak ke depan kotak 2
    Serial.println("[Martial] Bergerak ke kotak 2...");
    gatot.maju(MOTOR_SPEED);
    delay(800);
    gatot.berhenti();

    // Deteksi apakah ada KFS di kotak 2
    kfsTerdeteksi = false;
    deteksiKFS();

    if (kfsTerdeteksi) {
        Serial.println("[Martial] KFS di kotak 2 → tools KFS dijalankan");
        toolsKFS();
    } else {
        Serial.println("[Martial] Tidak ada KFS di kotak 2, lanjut.");
    }

    Serial.println("=== MARTIAL CLUE SELESAI ===    ");
}

// ==========================================================
//  KALIBRASI IMU
// ==========================================================
void calibrateIMU() {
    Serial.println("Kalibrasi IMU...");
    delay(2000);
    mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);  mpu.setYGyroOffset(0);  mpu.setZGyroOffset(0);
    Serial.println("IMU siap.");
}

// ==========================================================
//  SETUP
// ==========================================================
void setup() {
    Serial.begin(115200);
    Wire.begin();

    gatot.init();

    // Init & kalibrasi IMU
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("ERROR: MPU6050 tidak terhubung!");
    } else {
        calibrateIMU();
    }

    Serial.println("====== SISTEM SIAP ======");
    delay(1000);

    // ---- Urutan eksekusi sesuai diagram ----
    algoritmaArena();   // 1. Arena
    medanForest();      // 2. Medan Forest
    martialClue();      // 3. Martial Clue

    Serial.println("====== PROGRAM SELESAI ======");
}

// ==========================================================
//  LOOP – monitoring pasif
// ==========================================================
void loop() {
    // Monitoring arus (proteksi overcurrent)
    float arus = gatot.bacaArus();
    if (arus > OVERCURRENT_LIMIT) {
        Serial.println("OVERCURRENT! Motor dihentikan.");
        gatot.berhenti();
        delay(2000);
    }
    delay(200);
}