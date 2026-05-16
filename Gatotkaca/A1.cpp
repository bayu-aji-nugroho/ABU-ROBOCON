//udah sampai algoritma naik ke forest, tapi masih kurang sempurna
#include <Arduino.h>
// --- KONFIGURASI PIN MOTOR ---
// FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
const int FL_PWM = 6;  const int FL_DIR = 7;
const int FR_PWM = 8;  const int FR_DIR = 9;
const int BL_PWM = 10; const int BL_DIR = 11;
const int BR_PWM = 12; const int BR_DIR = 13;

// --- PIN SENSOR ---
const int TRIG_KIRI = 2;  const int ECHO_KIRI = 3;
const int TRIG_KANAN = 4; const int ECHO_KANAN = 5;

// --- PIN TAMBAHAN UNTUK KAMERA ---
const int PIN_FOREST_LURUS = A0; // Input dari kamera: High jika forest tengah lurus
const int PIN_ADA_KOTAK = A1;    // Input dari kamera: High jika ada kotak di atas forest

class robotgatot{
public:
    void init() {
        pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
        pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
        pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);
        pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);
    }

    void move(int fl, int fr, int bl, int br){
        //maju=high mundur=low
        digitalWrite(FL_DIR, fl >= 0 ? HIGH : LOW);
        analogWrite(FL_PWM, abs(fl));

        digitalWrite(FR_DIR, fr >= 0 ? HIGH : LOW);
        analogWrite(FR_PWM, abs(fr));

        digitalWrite(BL_DIR, bl >= 0 ? HIGH : LOW);
        analogWrite(BL_PWM, abs(bl));

        digitalWrite(BR_DIR, br >= 0 ? HIGH : LOW);
        analogWrite(BR_PWM, abs(br));

    }
//FUNGSI GERAK DASAR RODA
    void maju (int speed) { move(speed, speed, speed, speed); }
    void mundur (int speed) { move(-speed, -speed, -speed, -speed); }
    void geserkanan (int speed){ move(speed, -speed, -speed, speed); }
    void geserkiri (int speed) { move(-speed, speed, speed, -speed); }
    void putarkiri(int speed) { move(-speed, speed, -speed, speed); }
    void putarkanan(int speed) { move(speed, -speed, speed, -speed); }
    void berhenti() { move(0, 0, 0, 0); }

    void putarbalik() {
        putarkanan(150); 
        delay(1200); // Kalibrasi manual sampai tepat 180 derajat
        berhenti();
    }

    long bacaUltrasonic(int trig, int echo) {
        digitalWrite(trig, LOW); delayMicroseconds(2);
        digitalWrite(trig, HIGH); delayMicroseconds(10);
        digitalWrite(trig, LOW);
        return pulseIn(echo, HIGH) * 0.343 / 2;
    }
};

robotgatot gatot;
bool rakKiri = false;

void setup() {
    gatot.init();
    Serial.begin(9600);

    // 1. Maju 60 cm
    gatot.maju(600);
    delay(2000); 
    gatot.berhenti();

    // 2. Deteksi Rak (875 mm)
    long dikiri = gatot.bacaUltrasonic(TRIG_KIRI, ECHO_KIRI);
    long dikanan = gatot.bacaUltrasonic(TRIG_KANAN, ECHO_KANAN);

if (dikiri < 1000) { 
    rakKiri = true;
    gatot.putarkiri(90);
    gatot.maju(600);

}
else if (dikanan < 1000) {
    rakKiri = false;
    gatot.putarkanan(90);
    gatot.maju(600);
}
else {
    gatot.berhenti();
}

for (int i=0; i<4; i++) {
    gatot.berhenti();

    gatot.putarbalik();
    delay(2000);

    if (i<3);{
        gatot.putarbalik();
    }    
}
 
delay(5000);

if (rakKiri) {
    gatot.putarkanan(90);
    gatot.maju(2000);
    gatot.putarkanan(90);
}
else {
    gatot.putarkiri(90);
    gatot.maju(2000);
    gatot.putarkiri(90);
}

//algortima buat naik ke forest menggunakan kamera
bool diDepanForest = false;
    while (!diDepanForest) {
        if (digitalRead(PIN_FOREST_LURUS) == HIGH) {
            gatot.berhenti();
            diDepanForest = true;
        } else {
            gatot.maju(100); // Pelan-pelan mencari posisi forest
        }
    }

    // Cek keberadaan kotak di atas forest sebelum naik
    if (digitalRead(PIN_ADA_KOTAK) == HIGH) {
        gatot.berhenti();
        naikKeForest();
    } else {
        naikKeForest(); // Langsung naik jika kosong
    }

void naikKeForest() {
    //di isi sesuai gerak angkat robot 
}

void loop() {}  

}



