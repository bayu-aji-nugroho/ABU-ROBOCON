// ============================================================
// INTEGRATED ROBOT SYSTEM: Forest Climbing + Jungle + Tic-Tac-Toe
// Based on A1.cpp, A2.cpp, A3.cpp + system design guidance
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_APDS9960.h>
#include <MPU6050.h>

// ========== PIN DEFINITIONS (MERGED) ==========
// Motor (Mecanum) – from A1 + A3
const int FL_PWM = 6;  const int FL_DIR = 7;
const int FR_PWM = 8;  const int FR_DIR = 9;
const int BL_PWM = 10; const int BL_DIR = 11;
const int BR_PWM = 12; const int BR_DIR = 13;

// Ultrasonic sensors (4x) – from A3
#define TRIG_FRONT  22
#define ECHO_FRONT  23
#define TRIG_BACK   24
#define ECHO_BACK   25
#define TRIG_LEFT   26
#define ECHO_LEFT   27
#define TRIG_RIGHT  28
#define ECHO_RIGHT  29

// Camera inputs – from A1
const int PIN_FOREST_LURUS = A0;   // HIGH = forest straight ahead
const int PIN_ADA_KOTAK    = A1;   // HIGH = object on forest

// Sensors – from A3
#define VIBRATION_PIN    34
#define CURRENT_SENSOR_PIN 35

// ========== CONSTANTS ==========
#define OBSTACLE_DISTANCE   30.0   // cm
#define PROXIMITY_THRESHOLD 50
#define SAFE_DISTANCE       50.0
#define MOTOR_SPEED         200
#define ACS712_OFFSET       2.5
#define ACS712_SENSITIVITY  0.185
#define OVERCURRENT_THRESHOLD 3.0
#define IMU_ACCEL_THRESHOLD 2.0

// ========== GLOBAL OBJECTS ==========
Adafruit_APDS9960 apds;
MPU6050 mpu;
bool imuCalibrated = false;
bool rakKiri = false;          // from A1

// Sensor readings
uint16_t proximity;
int16_t ax, ay, az, gx, gy, gz;
float distanceFront, distanceBack, distanceLeft, distanceRight;
bool vibrationDetected;
float currentSensor;

// ========== MOTOR CONTROL CLASS (from A1 enhanced) ==========
class RobotGatot {
public:
    void init() {
        pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
        pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
        pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);
        pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);
    }

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

    void maju(int sp)        { move(sp, sp, sp, sp); }
    void mundur(int sp)      { move(-sp, -sp, -sp, -sp); }
    void geserKanan(int sp)  { move(sp, -sp, -sp, sp); }
    void geserKiri(int sp)   { move(-sp, sp, sp, -sp); }
    void putarKiri(int sp)   { move(-sp, sp, -sp, sp); }
    void putarKanan(int sp)  { move(sp, -sp, sp, -sp); }
    void berhenti()          { move(0, 0, 0, 0); }

    void putarBalik() {
        putarKanan(150);
        delay(1200);   // calibrated 180°
        berhenti();
    }
};

RobotGatot gatot;

// ========== JUNGLE / KFS HANDLING (from A2) ==========
class Jungle {
private:
    bool objekTerdeteksi;
    bool isObjekTerambil;
public:
    Jungle(bool obj, bool ambil) : objekTerdeteksi(obj), isObjekTerambil(ambil) {}

    void cekKFS() {
        if (objekTerdeteksi) {
            for (int i = 0; i < 1000; i++) {
                if (isObjekTerambil) break;
                delay(1);
            }
        }
        // reset flags after check
        objekTerdeteksi = false;
        isObjekTerambil = false;
    }

    void sudut(int derajat) {
        // Use gyro from A3 to turn exactly 'derajat' degrees
        float target = gz / 131.0;   // gyro Z in deg/sec
        float angle = 0;
        unsigned long start = millis();
        if (derajat > 0) {
            gatot.putarKanan(100);
            while (angle < derajat && millis() - start < 5000) {
                mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
                angle += (gz / 131.0) * 0.02;
                delay(20);
            }
        } else {
            gatot.putarKiri(100);
            while (angle > derajat && millis() - start < 5000) {
                mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
                angle -= (gz / 131.0) * 0.02;
                delay(20);
            }
        }
        gatot.berhenti();
    }

    void hutan() {
        sudut(90);
        cekKFS();
        sudut(-90);
        cekKFS();
        sudut(0);
        cekKFS();
    }
};

// ========== SENSOR FUNCTIONS (from A3) ==========
float readUltrasonic(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW); delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long dur = pulseIn(echo, HIGH, 30000);
    float dist = dur * 0.034 / 2;
    return (dist == 0 || dist > 400) ? 400 : dist;
}

void readAllUltrasonics() {
    distanceFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    distanceBack  = readUltrasonic(TRIG_BACK, ECHO_BACK);
    distanceLeft  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
    distanceRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
}

bool readVibration() { return digitalRead(VIBRATION_PIN) == HIGH; }

float readCurrent() {
    int val = analogRead(CURRENT_SENSOR_PIN);
    float volt = (val / 4095.0) * 3.3;
    return abs((volt - ACS712_OFFSET) / ACS712_SENSITIVITY);
}

void readAllSensors() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    proximity = apds.readProximity();
    readAllUltrasonics();
    vibrationDetected = readVibration();
    currentSensor = readCurrent();
}

// ========== FOREST CLIMBING (from A1, completed) ==========
void naikKeForest() {
    Serial.println("Climbing forest...");
    // Lift mechanism: here we simply move forward with strafe correction
    gatot.maju(150);
    delay(1500);               // 20 cm up
    gatot.berhenti();
    delay(500);
    // Move to center of forest
    gatot.maju(100);
    delay(800);
    gatot.berhenti();
}

void forestMode() {
    bool diDepanForest = false;
    while (!diDepanForest) {
        if (digitalRead(PIN_FOREST_LURUS) == HIGH) {
            gatot.berhenti();
            diDepanForest = true;
        } else {
            gatot.maju(100);
            delay(50);
        }
    }
    if (digitalRead(PIN_ADA_KOTAK) == HIGH) {
        // object present – wait or perform action
        Serial.println("Object on forest, waiting...");
        delay(2000);
    }
    naikKeForest();
}

// ========== TIC‑TAC‑TOE MODE (from A3, adapted) ==========
void navigateToCell(int cell) {
    float targetDist;
    switch(cell) {
        case 1: case 2: case 3: targetDist = 40.0; break;
        case 4: case 5: case 6: targetDist = 25.0; break;
        default: targetDist = 10.0;
    }
    while (abs(distanceFront - targetDist) > 2.0) {
        readAllUltrasonics();
        (distanceFront > targetDist) ? gatot.maju(100) : gatot.mundur(100);
        delay(100);
    }
    gatot.berhenti();

    int col = ((cell - 1) % 3) + 1;
    if (col == 1) { gatot.putarKiri(80); delay(300); gatot.maju(100); delay(400); gatot.berhenti(); }
    else if (col == 3) { gatot.putarKanan(80); delay(300); gatot.maju(100); delay(400); gatot.berhenti(); }
}

void performTicTacToe() {
    Serial.println("Starting Tic‑Tac‑Toe...");
    // Approach board
    while (distanceFront > 20.0 && distanceFront < 200.0) {
        readAllUltrasonics();
        if (distanceFront > 30.0) gatot.maju(150);
        else if (distanceFront > 20.0) gatot.maju(80);
        delay(100);
    }
    gatot.berhenti();
    delay(500);

    // Play 5 moves (predefined)
    int moves[] = {5, 1, 3, 7, 9};
    for (int i = 0; i < 5; i++) {
        navigateToCell(moves[i]);
        Serial.print("Placing marker at cell "); Serial.println(moves[i]);
        delay(1000);   // actuate marker
        if (readVibration()) delay(300);
    }
    // Return home
    while (distanceFront < 50.0) {
        readAllUltrasonics();
        gatot.mundur(120);
        delay(100);
    }
    gatot.berhenti();
}

// ========== OBSTACLE AVOIDANCE (based on A3) ==========
int findBestDirection() {
    float dist[4] = {distanceFront, distanceBack, distanceLeft, distanceRight};
    int best = 0;
    for (int i = 1; i < 4; i++) if (dist[i] > dist[best]) best = i;
    return best;
}

void handleObstacle() {
    gatot.berhenti();
    delay(500);
    if (currentSensor > OVERCURRENT_THRESHOLD) { Serial.println("Overcurrent!"); return; }
    int best = findBestDirection();
    switch(best) {
        case 0: break;   // front is best – continue forward
        case 1: gatot.putarKanan(150); delay(1600); break;
        case 2: gatot.putarKiri(150);  delay(800);  break;
        case 3: gatot.putarKanan(150); delay(800);  break;
    }
    gatot.berhenti();
}

// ========== SETUP & LOOP (MERGED) ==========
void setup() {
    Serial.begin(115200);
    Wire.begin();
    gatot.init();

    // Init APDS9960
    if (!apds.begin()) Serial.println("APDS9960 error");
    else apds.enableProximity(true);
    // Init MPU6050
    mpu.initialize();
    if (mpu.testConnection()) Serial.println("MPU6050 OK");
    // Ultrasonic pins
    uint8_t trigs[4] = {TRIG_FRONT, TRIG_BACK, TRIG_LEFT, TRIG_RIGHT};
    uint8_t ech[4]   = {ECHO_FRONT, ECHO_BACK, ECHO_LEFT, ECHO_RIGHT};
    for (int i = 0; i < 4; i++) { pinMode(trigs[i], OUTPUT); pinMode(ech[i], INPUT); }
    pinMode(VIBRATION_PIN, INPUT);
    pinMode(CURRENT_SENSOR_PIN, INPUT);
    pinMode(PIN_FOREST_LURUS, INPUT);
    pinMode(PIN_ADA_KOTAK, INPUT);
    delay(1000);

    // ---- Original A1 sequence ----
    gatot.maju(600);
    delay(2000);
    gatot.berhenti();

    long dikiri  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
    long dikanan = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
    if (dikiri < 1000) { rakKiri = true;  gatot.putarKiri(90); }
    else if (dikanan < 1000) { rakKiri = false; gatot.putarKanan(90); }
    else gatot.berhenti();
    gatot.maju(600);
    delay(1000);
    for (int i = 0; i < 4; i++) {
        gatot.berhenti();
        gatot.putarBalik();
        delay(2000);
        if (i < 3) gatot.putarBalik();
    }
    delay(5000);
    if (rakKiri) { gatot.putarKanan(90); gatot.maju(2000); gatot.putarKanan(90); }
    else         { gatot.putarKiri(90);  gatot.maju(2000); gatot.putarKiri(90); }

    // Forest climbing
    forestMode();

    // Jungle & KFS check
    Jungle jungle(digitalRead(PIN_ADA_KOTAK) == HIGH, false);
    jungle.hutan();

    // Tic‑Tac‑Toe
    performTicTacToe();
}

void loop() {
    // Continuous obstacle avoidance & monitoring (from A3)
    readAllSensors();

    if (currentSensor > OVERCURRENT_THRESHOLD) {
        gatot.berhenti();
        return;
    }
    if (vibrationDetected) {
        gatot.berhenti();
        delay(300);
        handleObstacle();
        return;
    }
    // Check acceleration
    float accel = sqrt(sq(ax/16384.0) + sq(ay/16384.0) + sq(az/16384.0));
    if (abs(accel - 1.0) > IMU_ACCEL_THRESHOLD) handleObstacle();
    else if (proximity > PROXIMITY_THRESHOLD) handleObstacle();
    else if (distanceFront < OBSTACLE_DISTANCE || distanceBack < OBSTACLE_DISTANCE ||
             distanceLeft < OBSTACLE_DISTANCE || distanceRight < OBSTACLE_DISTANCE)
        handleObstacle();
    else {
        // Normal forward with wall following
        if (distanceLeft < 20.0) gatot.maju(MOTOR_SPEED * 0.7);   // adjust right
        else if (distanceRight < 20.0) gatot.maju(MOTOR_SPEED * 0.7);
        else gatot.maju(MOTOR_SPEED);
    }
    delay(50);
}