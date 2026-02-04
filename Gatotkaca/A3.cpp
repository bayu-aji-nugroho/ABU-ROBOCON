#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_APDS9960.h>
#include <MPU6050.h>

// ===== DEFINISI PIN =====
// HC-SR04 Ultrasonic Sensors (4 directions)
#define TRIG_FRONT 
#define ECHO_FRONT 
#define TRIG_BACK 
#define ECHO_BACK 
#define TRIG_LEFT 
#define ECHO_LEFT 
#define TRIG_RIGHT 
#define ECHO_RIGHT 

// Sensor Pins
#define VIBRATION_PIN 34      // KW12-3 Vibration Sensor (Digital Input)
#define CURRENT_SENSOR_PIN 35 // ACS712 Current Sensor (Analog Input)

// ===== THRESHOLD & KONSTANTA =====
#define OBSTACLE_DISTANCE 30.0    // cm - HC-SR04 obstacle threshold
#define PROXIMITY_THRESHOLD 50     // APDS9960 proximity threshold
#define IMU_ACCEL_THRESHOLD 2.0   // g - MPU6050 acceleration threshold
#define SAFE_DISTANCE 50.0        // cm - Safe distance for movement
#define MOTOR_SPEED 200           // PWM value (0-255)

// KW12-3 Vibration Sensor
#define VIBRATION_THRESHOLD 1     // Digital threshold (HIGH=vibration detected)

// ACS712 Current Sensor (5A version: 185mV/A, 20A version: 100mV/A, 30A version: 66mV/A)
#define ACS712_SENSITIVITY 0.185  // V/A for 5A module
#define ACS712_OFFSET 2.5         // VCC/2 for zero current
#define CURRENT_THRESHOLD 0.5     // Ampere - abnormal current threshold
#define OVERCURRENT_THRESHOLD 3.0 // Ampere - motor overcurrent protection

// ===== INISIALISASI SENSOR =====
Adafruit_APDS9960 apds;
MPU6050 mpu;

// Sensor readings
uint16_t proximity;
int16_t ax, ay, az, gx, gy, gz;

// 4 Ultrasonic distances
float distanceFront = 0.0;
float distanceBack = 0.0;
float distanceLeft = 0.0;
float distanceRight = 0.0;

bool vibrationDetected = false;
float currentSensor = 0.0;

// Timing
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 100;

// Status flags
bool systemRunning = false;
bool obstacleDetected = false;
bool imuCalibrated = false;
bool tictactoeMode = false;

// Array untuk loop ultrasonic
const uint8_t trigPins[4] = {TRIG_FRONT, TRIG_BACK, TRIG_LEFT, TRIG_RIGHT};
const uint8_t echoPins[4] = {ECHO_FRONT, ECHO_BACK, ECHO_LEFT, ECHO_RIGHT};
const char* directions[4] = {"FRONT", "BACK", "LEFT", "RIGHT"};

// ===== FUNGSI INISIALISASI SENSOR =====
void initSensors() {
  Serial.println("Initializing sensors...");
  
  // Init APDS9960 - Proximity Sensor
  if (!apds.begin()) {
    Serial.println("ERROR: APDS9960 not found!");
  } else {
    apds.enableProximity(true);
    Serial.println("✓ APDS9960 (Proximity) initialized");
  }
  
  // Init MPU6050 - IMU (Gyro + Accelerometer)
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 not found!");
  } else {
    Serial.println("✓ MPU6050 (IMU) initialized");
    calibrateIMU();
  }
  // Setup 4x HC-SR04 Ultrasonic Sensors
  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
  Serial.println("✓ 4x HC-SR04 (Front/Back/Left/Right) initialized");
  
  // Setup KW12-3 - Vibration Sensor (Digital Input)
  pinMode(VIBRATION_PIN, INPUT);
  Serial.println("✓ KW12-3 (Vibration Sensor) initialized");
  
  // Setup ACS712 - Current Sensor (Analog Input)
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  Serial.println("✓ ACS712 (Current Sensor) initialized");
}

// ===== KALIBRASI IMU =====
void calibrateIMU() {
  Serial.println("Calibrating IMU...");
  delay(2000);
  
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  
  imuCalibrated = true;
  Serial.println("✓ IMU calibrated");
}

// ===== BACA SINGLE ULTRASONIC =====
float readUltrasonicSensor(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  float dist = duration * 0.034 / 2;
  
  if (dist == 0 || dist > 400) {
    dist = 400;  // Max range or no echo
  }
  
  return dist;
}

// ===== BACA 4 ULTRASONIC SENSORS =====
void readAllUltrasonics() {
  distanceFront = readUltrasonicSensor(TRIG_FRONT, ECHO_FRONT);
  delay(10);  // Small delay between sensor readings
  
  distanceBack = readUltrasonicSensor(TRIG_BACK, ECHO_BACK);
  delay(10);
  
  distanceLeft = readUltrasonicSensor(TRIG_LEFT, ECHO_LEFT);
  delay(10);
  
  distanceRight = readUltrasonicSensor(TRIG_RIGHT, ECHO_RIGHT);
  delay(10);
}

// ===== BACA KW12-3 VIBRATION SENSOR =====
bool readVibration() {
  return digitalRead(VIBRATION_PIN) == HIGH;
}

// ===== BACA ACS712 CURRENT SENSOR =====
float readCurrent() {
  int sensorValue = analogRead(CURRENT_SENSOR_PIN);
  float voltage = (sensorValue / 4095.0) * 3.3;
  float current = (voltage - ACS712_OFFSET) / ACS712_SENSITIVITY;
  return abs(current);
}

// ===== BACA SEMUA SENSOR =====
void readAllSensors() {
  // Baca MPU6050 - IMU
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Baca APDS9960 - Proximity
  proximity = apds.readProximity();
  
  // Baca 4x HC-SR04 - Ultrasonic Distances
  readAllUltrasonics();
  
  // Baca KW12-3 - Vibration Detection
  vibrationDetected = readVibration();
  
  // Baca ACS712 - Current Sensor
  currentSensor = readCurrent();
  
  // Debug output
  Serial.print("IMU ax:"); Serial.print(ax);
  Serial.print(" | Prox:"); Serial.print(proximity);
  Serial.print(" | F:"); Serial.print(distanceFront); Serial.print("cm");
  Serial.print(" B:"); Serial.print(distanceBack); Serial.print("cm");
  Serial.print(" L:"); Serial.print(distanceLeft); Serial.print("cm");
  Serial.print(" R:"); Serial.print(distanceRight); Serial.print("cm");
  Serial.print(" | Vib:"); Serial.print(vibrationDetected ? "Y" : "N");
  Serial.print(" | Cur:"); Serial.print(currentSensor); Serial.println("A");
}

// ===== CEK AKSELERASI IMU =====
bool checkIMUAcceleration() {
  float accel_x = ax / 16384.0;
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;
  
  float total_accel = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
  
  if (abs(total_accel - 1.0) > IMU_ACCEL_THRESHOLD) {
    return false;
  }
  return true;
}
// ===== CEK OBSTACLE DARI SEMUA ARAH =====
bool checkObstacleAnyDirection() {
  // Check if any ultrasonic sensor detects obstacle
  if (distanceFront < OBSTACLE_DISTANCE) {
    Serial.println("Obstacle FRONT!");
    return true;
  }
  if (distanceBack < OBSTACLE_DISTANCE) {
    Serial.println("Obstacle BACK!");
    return true;
  }
  if (distanceLeft < OBSTACLE_DISTANCE) {
    Serial.println("Obstacle LEFT!");
    return true;
  }
  if (distanceRight < OBSTACLE_DISTANCE) {
    Serial.println("Obstacle RIGHT!");
    return true;
  }
  return false;
}

// ===== CARI ARAH TERBAIK (JARAK TERJAUH) =====
int findBestDirection() {
  // Return index: 0=Front, 1=Back, 2=Left, 3=Right
  float distances[4] = {distanceFront, distanceBack, distanceLeft, distanceRight};
  int bestDir = 0;
  float maxDist = distances[0];
  
  for (int i = 1; i < 4; i++) {
    if (distances[i] > maxDist) {
      maxDist = distances[i];
      bestDir = i;
    }
  }
  
  return bestDir;
}

// ===== KONTROL MOTOR =====
void stopAllMotors() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  Serial.println("Motors: STOP");
}

void moveForward() {
  analogWrite(MOTOR_LEFT_FWD, MOTOR_SPEED);
  analogWrite(MOTOR_RIGHT_FWD, MOTOR_SPEED);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  Serial.println("Motors: FORWARD");
}

void moveBackward() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  analogWrite(MOTOR_LEFT_BWD, MOTOR_SPEED);
  analogWrite(MOTOR_RIGHT_BWD, MOTOR_SPEED);
  Serial.println("Motors: BACKWARD");
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  analogWrite(MOTOR_RIGHT_FWD, MOTOR_SPEED);
  analogWrite(MOTOR_LEFT_BWD, MOTOR_SPEED/2);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  Serial.println("Motors: TURN LEFT");
}

void turnRight() {
  analogWrite(MOTOR_LEFT_FWD, MOTOR_SPEED);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  analogWrite(MOTOR_RIGHT_BWD, MOTOR_SPEED/2);
  Serial.println("Motors: TURN RIGHT");
}

// ===== HANDLE OBSTACLE (DENGAN 4 SENSOR) =====
void handleObstacle() {
  stopAllMotors();
  delay(500);
  
  // Display current sensor readings
  Serial.println("\nCurrent Distances:");
  Serial.print("  Front: "); Serial.print(distanceFront); Serial.println(" cm");
  Serial.print("  Back:  "); Serial.print(distanceBack); Serial.println(" cm");
  Serial.print("  Left:  "); Serial.print(distanceLeft); Serial.println(" cm");
  Serial.print("  Right: "); Serial.print(distanceRight); Serial.println(" cm");
  
  // Check vibration sensor (KW12-3)
  if (vibrationDetected) {
    Serial.println("VIBRATION! Possible collision!");
    delay(200);
  }
  
  // Check current sensor (ACS712)
  if (currentSensor > OVERCURRENT_THRESHOLD) {
    Serial.println("OVERCURRENT! Motor stalled!");
    stopAllMotors();
    delay(1000);
    return;
  }
  
  // Check for errors
  bool hasError = false;
  
  // 1. Check IMU
  float accel_total = sqrt((ax/16384.0)*(ax/16384.0) + 
                           (ay/16384.0)*(ay/16384.0) + 
                           (az/16384.0)*(az/16384.0));
  if (abs(accel_total - 1.0) > IMU_ACCEL_THRESHOLD) {
    Serial.println("IMU Error: Abnormal tilt");
    hasError = true;
  }
  
  // 2. Check proximity
  if (proximity > 150) {
    Serial.println("Proximity: Object very close");
    hasError = true;
  }
  
  // 3. Check vibration
  if (vibrationDetected) {
    Serial.println("Vibration: Continuous");
    hasError = true;
  }
  
  // 4. Check current
  if (currentSensor > CURRENT_THRESHOLD) {
    Serial.println("Current: High draw");
    hasError = true;
  }
  
  // === INTELLIGENT NAVIGATION BASED ON 4 SENSORS ===
  Serial.println("\nFinding best escape route...");
  
  // Find direction with most space
  int bestDir = findBestDirection();
  float distances[4] = {distanceFront, distanceBack, distanceLeft, distanceRight};
  
  Serial.print("✓ Best direction: "); Serial.print(directions[bestDir]);
  Serial.print(" ("); Serial.print(distances[bestDir]); Serial.println(" cm)");
  
  // If front is blocked, move backward first
  if (distanceFront < OBSTACLE_DISTANCE && distanceBack > OBSTACLE_DISTANCE) {
    Serial.println("→ Moving BACKWARD to create space...");
    moveBackward();
    delay(1000);
    stopAllMotors();
    delay(500);
  }
  
  // Navigate to best direction
  if (hasError) {
    // Error mode: use safe fallback
    Serial.println("⚠ Error mode: Safe fallback - turning left");
    turnLeft();
    delay(1200);
  } else {
    // Normal mode: navigate to best direction
    switch(bestDir) {
      case 0:  // Front is best
        Serial.println("→ Path ahead is clear - moving FORWARD");
        // Already facing forward, will move in main loop
        break;
        
      case 1:  // Back is best
        Serial.println("→ Turning around (180°)");
        turnRight();
        delay(1600);  // ~180 degree turn
        break;
        
      case 2:  // Left is best
        Serial.println("→ Turning LEFT (90°)");
        turnLeft();
        delay(800);
        break;
        
      case 3:  // Right is best
        Serial.println("→ Turning RIGHT (90°)");
        turnRight();
        delay(800);
        break;
    }
  }
  
  stopAllMotors();
  delay(300);
  
  // Re-check sensors after maneuvering
  readAllUltrasonics();
  Serial.println("\n📊 After maneuver:");
  Serial.print("  Front: "); Serial.print(distanceFront); Serial.println(" cm");
}

// ===== TIC-TAC-TOE GAME MODE =====
void performTicTacToeMode() {
  
  tictactoeMode = true;
  // STEP 1: Approach board using FRONT sensor
  Serial.println("Step 1: Approaching game board...");
  
  while (distanceFront > 20.0 && distanceFront < 200.0) {
    readAllUltrasonics();
    Serial.print("Distance to board: "); Serial.print(distanceFront); Serial.println(" cm");
    
    // Check side clearance
    if (distanceLeft < 15.0 || distanceRight < 15.0) {
      Serial.println("⚠ Too close to side - adjusting...");
      stopAllMotors();
      delay(500);
      
      if (distanceLeft < distanceRight) {
        // Adjust right
        turnRight();
        delay(200);
        stopAllMotors();
      } else {
        // Adjust left
        turnLeft();
        delay(200);
        stopAllMotors();
      }
    }
    
    if (distanceFront > 30.0) {
      moveForward();
    } else if (distanceFront > 20.0) {
      analogWrite(MOTOR_LEFT_FWD, MOTOR_SPEED/3);
      analogWrite(MOTOR_RIGHT_FWD, MOTOR_SPEED/3);
      digitalWrite(MOTOR_LEFT_BWD, LOW);
      digitalWrite(MOTOR_RIGHT_BWD, LOW);
    }
    delay(100);
  }
  stopAllMotors();
  Serial.println("✓ Reached game board position!");
  delay(500);
  
  // STEP 2: Check orientation (MPU6050)
  Serial.println("\nStep 2: Checking orientation...");
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float accel_z = az / 16384.0;
  if (abs(accel_z) < 0.8) {
    Serial.println("⚠ Robot tilted - adjusting...");
    moveBackward();
    delay(300);
    stopAllMotors();
    delay(500);
  } else {
    Serial.println("✓ Robot orientation OK");
  }
  
  // STEP 3: Surface stability (KW12-3)
  Serial.println("\nStep 3: Checking surface stability...");
  int vibrationCount = 0;
  for (int i = 0; i < 10; i++) {
    if (readVibration()) vibrationCount++;
    delay(100);
  }
  
  if (vibrationCount > 3) {
    Serial.println("⚠ Unstable surface - relocating...");
    moveForward();
    delay(500);
    stopAllMotors();
    delay(300);
  } else {
    Serial.println("✓ Surface stable");
  }
  
  // STEP 4: Motor check (ACS712)
  Serial.println("\nStep 4: Motor system check...");
  stopAllMotors();
  delay(500);
  
  float idleCurrent = readCurrent();
  Serial.print("Idle current: "); Serial.print(idleCurrent); Serial.println("A");
  
  moveForward();
  delay(200);
  float activeCurrent = readCurrent();
  stopAllMotors();
  
  Serial.print("Active current: "); Serial.print(activeCurrent); Serial.println("A");
  
  if (activeCurrent > OVERCURRENT_THRESHOLD) {
    Serial.println("Overcurrent - ABORT!");
    tictactoeMode = false;
    return;
  } else if (activeCurrent < 0.1) {
    Serial.println("No motor response - ABORT!");
    tictactoeMode = false;
    return;
  } else {
    Serial.println("✓ Motor current normal");
  }
  
  // STEP 5: Play game
  
  int moves[] = {5, 1, 3, 7, 9};
  
  for (int i = 0; i < 5; i++) {
    Serial.print("\nMove "); Serial.print(i+1);
    Serial.print(": Cell "); Serial.println(moves[i]);
    
    navigateToCell(moves[i]);
    
    if (readVibration()) {
      Serial.println("Vibration - waiting...");
      delay(500);
    }
    
    Serial.println("Placing marker...");
    delay(1000);
    
    currentSensor = readCurrent();
    if (currentSensor > CURRENT_THRESHOLD) {
      Serial.println("High current during placement");
      delay(300);
    }
    
    Serial.println("Marker placed!");
    delay(500);
  }

  returnToHome();
  
  tictactoeMode = false;
}

// Navigate to specific cell
void navigateToCell(int cell) {
  Serial.print("Navigating to cell "); Serial.println(cell);
  
  float targetDistance;
  
  switch(cell) {
    case 1: case 2: case 3:
      targetDistance = 40.0;
      break;
    case 4: case 5: case 6:
      targetDistance = 25.0;
      break;
    case 7: case 8: case 9:
      targetDistance = 10.0;
      break;
    default:
      targetDistance = 25.0;
  }
  
  // Navigate forward/backward using FRONT sensor
  readAllUltrasonics();
  while (abs(distanceFront - targetDistance) > 2.0) {
    readAllUltrasonics();
    
    if (distanceFront > targetDistance) {
      moveForward();
    } else {
      moveBackward();
    }
    delay(100);
  }
  stopAllMotors();
  
  // Lateral positioning
  int column = ((cell - 1) % 3) + 1;
  if (column == 1) {
    Serial.println("Left column");
    turnLeft();
    delay(300);
    moveForward();
    delay(400);
    stopAllMotors();
  } else if (column == 3) {
    Serial.println("Right column");
    turnRight();
    delay(300);
    moveForward();
    delay(400);
    stopAllMotors();
  } else {
    Serial.println("Center column");
  }
  
  delay(300);
}

// Return to home using BACK sensor
void returnToHome() {
  Serial.println("Returning to home...");
  
  moveBackward();
  delay(1000);
  
  while (distanceFront < 50.0) {
    readAllUltrasonics();
    moveBackward();
    delay(100);
  }
  
  stopAllMotors();
  
  // Final check
  readAllSensors();
  Serial.print("  Front: "); Serial.print(distanceFront); Serial.println(" cm");
  Serial.print("  Vibration: "); Serial.println(vibrationDetected ? "Yes" : "No");
  Serial.print("  Current: "); Serial.print(currentSensor); Serial.println(" A");
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Setup motor pins
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);
  
  delay(1000);
  
  // Initialize all sensors
  initSensors();
  
  delay(2000);
  systemRunning = true;
}

void loop() {
  if (!systemRunning) {
    stopAllMotors();
    return;
  }
  
  // Read all sensors periodically
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    readAllSensors();
    lastSensorRead = millis();
  }
  
  // 1. CRITICAL: Overcurrent protection
  if (currentSensor > OVERCURRENT_THRESHOLD) {
    Serial.println("\nCRITICAL: MOTOR OVERCURRENT!");
    stopAllMotors();
    delay(2000);
    return;
  }
  
  // 2. Vibration detection
  if (vibrationDetected) {
    Serial.println("Vibration detected...");
    stopAllMotors();
    delay(300);
    
    if (readVibration()) {
      Serial.println("Persistent vibration - collision!");
      handleObstacle();
      return;
    }
  }
  
  // 3. IMU acceleration check
  if (!checkIMUAcceleration()) {
    Serial.println("IMU: Abnormal acceleration!");
    handleObstacle();
    return;
  }
  
  // 4. Proximity sensor check
  if (proximity > PROXIMITY_THRESHOLD) {
    Serial.println("Proximity: Object very close!");
    obstacleDetected = true;
    handleObstacle();
    return;
  }
  
  // 5. Multi-directional obstacle check
  if (checkObstacleAnyDirection()) {
    obstacleDetected = true;
    handleObstacle();
  } else {
    obstacleDetected = false;
    if (distanceFront > SAFE_DISTANCE) {
      // Safe distance - full speed
      if (distanceLeft < 20.0) {
        // Too close to left wall - adjust right
        analogWrite(MOTOR_LEFT_FWD, MOTOR_SPEED);
        analogWrite(MOTOR_RIGHT_FWD, MOTOR_SPEED * 0.7);
        digitalWrite(MOTOR_LEFT_BWD, LOW);
        digitalWrite(MOTOR_RIGHT_BWD, LOW);
      } else if (distanceRight < 20.0) {
        // Too close to right wall - adjust left
        analogWrite(MOTOR_LEFT_FWD, MOTOR_SPEED * 0.7);
        analogWrite(MOTOR_RIGHT_FWD, MOTOR_SPEED);
        digitalWrite(MOTOR_LEFT_BWD, LOW);
        digitalWrite(MOTOR_RIGHT_BWD, LOW);
      } else {
        // Clear path - move straight
        moveForward();
      }
    } else if (distanceFront > OBSTACLE_DISTANCE) {
      // Moderate distance - reduced speed
      analogWrite(MOTOR_LEFT_FWD, MOTOR_SPEED/2);
      analogWrite(MOTOR_RIGHT_FWD, MOTOR_SPEED/2);
      digitalWrite(MOTOR_LEFT_BWD, LOW);
      digitalWrite(MOTOR_RIGHT_BWD, LOW);
    } else {
      // Too close - stop
      stopAllMotors();
    }
  }
  
  static unsigned long lastGameMode = 0;
  static bool nearHomePosition = false;
  
  // Home position: far from all obstacles
  if (distanceFront > 80.0 && distanceBack > 80.0 && 
      distanceLeft > 40.0 && distanceRight > 40.0 && !obstacleDetected) {
    nearHomePosition = true;
  }
  
  if ((millis() - lastGameMode > 60000) || 
      (nearHomePosition && (millis() - lastGameMode > 30000))) {
    
    Serial.println("\n🎮 Initiating Tic-Tac-Toe Mode...");
    stopAllMotors();
    delay(1000);
    
    performTicTacToeMode();
    
    lastGameMode = millis();
    nearHomePosition = false;
  }
  
  delay(50);
}