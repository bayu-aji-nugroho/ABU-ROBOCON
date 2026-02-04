#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_APDS9960.h>
#include <MPU6050.h>

// ===== DEFINISI PIN =====

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

// ===== VARIABEL GLOBAL =====
// Sensor readings
uint16_t proximity;
int16_t ax, ay, az, gx, gy, gz;
float distance;
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

// Array ultrasonic 
const uint8_t 

// ===== FUNGSI INISIALISASI SENSOR =====
void initSensors() {
  Serial.println("Initializing sensors...");
  
  // Init APDS9960 - Proximity Sensor
  if (!apds.begin()) {
    Serial.println("ERROR: APDS9960 not found!");
  } else {
    apds.enableProximity(true);
    Serial.println("APDS9960 (Proximity) initialized");
  }
  
  // Init MPU6050 - IMU (Gyro + Accelerometer)
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 not found!");
  } else {
    Serial.println("MPU6050 (IMU) initialized");
    calibrateIMU();
  }
  
  // Setup KW12-3 - Vibration Sensor (Digital Input)
  pinMode(VIBRATION_PIN, INPUT);
  Serial.println("KW12-3 (Vibration Sensor) initialized");
  
  // Setup ACS712 - Current Sensor (Analog Input)
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  Serial.println("ACS712 (Current Sensor) initialized");
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
  Serial.println("IMU calibrated");
}

// ===== BACA ULTRASONIC =====
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float dist = duration * 0.034 / 2;
  
  if (dist == 0 || dist > 400) {
    dist = 400;
  }
  
  return dist;
}

// ===== BACA KW12-3 VIBRATION SENSOR =====
bool readVibration() {
  // KW12-3 outputs HIGH when vibration is detected
  return digitalRead(VIBRATION_PIN) == HIGH;
}

// ===== BACA ACS712 CURRENT SENSOR =====
float readCurrent() {
  // Read analog value (0-4095 for ESP32 12-bit ADC)
  int sensorValue = analogRead(CURRENT_SENSOR_PIN);
  
  // Convert to voltage (ESP32: 0-3.3V for 0-4095)
  float voltage = (sensorValue / 4095.0) * 3.3;
  
  // Calculate current using ACS712 formula
  // Current = (Vout - Voffset) / Sensitivity
  float current = (voltage - ACS712_OFFSET) / ACS712_SENSITIVITY;
  
  return abs(current);  // Return absolute current value
}

// ===== BACA SEMUA SENSOR =====
void readAllSensors() {
  
  // Baca MPU6050 - IMU
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Baca APDS9960 - Proximity
  proximity = apds.readProximity();
  
  // Baca HC-SR04 - Ultrasonic Distance
  distance = readUltrasonic();
  
  // Baca KW12-3 - Vibration Detection
  vibrationDetected = readVibration();
  
  // Baca ACS712 - Current Sensor
  currentSensor = readCurrent();
  
  // Debug output
  Serial.print("IMU ax:"); Serial.print(ax);
  Serial.print(" | Prox:"); Serial.print(proximity);
  Serial.print(" | Dist:"); Serial.print(distance); Serial.print("cm");
  Serial.print(" | Vib:"); Serial.print(vibrationDetected ? "YES" : "NO");
  Serial.print(" | Cur:"); Serial.print(currentSensor); Serial.println("A");
}

// ===== CEK AKSELERASI IMU =====
bool checkIMUAcceleration() {
  float accel_x = ax / 16384.0;  // konversi ke g
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;
  
  float total_accel = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
  
  // Normal gravity is ~1g, check if deviation is too high
  if (abs(total_accel - 1.0) > IMU_ACCEL_THRESHOLD) {
    return false;  // Abnormal acceleration detected
  }
  return true;
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

// ===== HANDLE OBSTACLE =====
void handleObstacle() {
  Serial.println("\n=== OBSTACLE DETECTED (Movement Mode) ===");
  stopAllMotors();
  delay(500);
  
  // Check vibration sensor (KW12-3) - might indicate collision
  if (vibrationDetected) {
    Serial.println("VIBRATION DETECTED! Possible collision!");
    delay(200);
  }
  
  // Check current sensor (ACS712) - motor stall detection
  if (currentSensor > OVERCURRENT_THRESHOLD) {
    Serial.println("OVERCURRENT! Motor may be stalled!");
    stopAllMotors();
    delay(1000);
    return;
  }
  
  // Mundur sedikit
  Serial.println("Moving backward to avoid obstacle...");
  moveBackward();
  delay(1000);
  stopAllMotors();
  delay(500);
  
  bool hasError = false;
  
  // Periksa sensor untuk error/warning
  // 1. Check IMU for abnormal tilt/orientation
  float accel_total = sqrt((ax/16384.0)*(ax/16384.0) + 
                           (ay/16384.0)*(ay/16384.0) + 
                           (az/16384.0)*(az/16384.0));
  if (abs(accel_total - 1.0) > IMU_ACCEL_THRESHOLD) {
    Serial.println("IMU Error: Abnormal acceleration/tilt");
    hasError = true;
  }
  
  // 2. Check proximity sensor for very close objects
  if (proximity > 150) {
    Serial.println("Proximity Error: Object very close");
    hasError = true;
  }
  
  // 3. Check vibration persistence
  if (vibrationDetected) {
    Serial.println("Vibration Error: Continuous vibration");
    hasError = true;
  }
  
  // 4. Check current sensor for motor issues
  if (currentSensor > CURRENT_THRESHOLD) {
    Serial.println("Current Error: Motor drawing high current");
    hasError = true;
  }
  
  // Scan kiri dan kanan untuk cari jalan
  float leftDistance, rightDistance;
  
  // Scan kiri
  Serial.println("Scanning left...");
  turnLeft();
  delay(500);
  stopAllMotors();
  delay(300);
  leftDistance = readUltrasonic();
  Serial.print("Left distance: "); Serial.print(leftDistance); Serial.println("cm");
  
  // Kembali ke tengah
  turnRight();
  delay(500);
  stopAllMotors();
  delay(300);
  
  // Scan kanan
  Serial.println("Scanning right...");
  turnRight();
  delay(500);
  stopAllMotors();
  delay(300);
  rightDistance = readUltrasonic();
  Serial.print("Right distance: "); Serial.print(rightDistance); Serial.println("cm");
  
  // Kembali ke tengah
  turnLeft();
  delay(500);
  stopAllMotors();
  delay(300);
  
  // Decide direction based on error status and distance scan
  if (hasError) {
    Serial.println("Error mode: Using safe fallback - turn left");
    turnLeft();
    delay(1200);  // Longer turn for safety
  } else {
    // Choose direction with more space
    if (leftDistance > rightDistance) {
      Serial.println("Left path is clearer - turning left");
      turnLeft();
      delay(800);
    } else {
      Serial.println("Right path is clearer - turning right");
      turnRight();
      delay(800);
    }
  }
  
  stopAllMotors();
  delay(300);
  
  Serial.println("=== Obstacle avoidance complete ===\n");
}

// ===== TIC-TAC-TOE GAME MODE (FLOWCHART KANAN) =====
void performTicTacToeMode() {
  
  tictactoeMode = true;
  // Use HC-SR04 to navigate to board
  while (distance > 20.0 && distance < 200.0) {
    distance = readUltrasonic();
    Serial.print("Distance to board: "); Serial.print(distance); Serial.println("cm");
    
    if (distance > 30.0) {
      moveForward();
    } else if (distance > 20.0) {
      analogWrite(MOTOR_LEFT_FWD, MOTOR_SPEED/3);
      analogWrite(MOTOR_RIGHT_FWD, MOTOR_SPEED/3);
      digitalWrite(MOTOR_LEFT_BWD, LOW);
      digitalWrite(MOTOR_RIGHT_BWD, LOW);
    }
    delay(100);
  }
  stopAllMotors();
  Serial.println("Reached game board position!");
  delay(500);
  
  // STEP 2: Verify robot orientation using MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float accel_z = az / 16384.0;
  if (abs(accel_z) < 0.8) {  // Check if robot is tilted
    Serial.println("Robot orientation abnormal - adjusting...");
    // Adjust position
    moveBackward();
    delay(300);
    stopAllMotors();
    delay(500);
  } else {
    Serial.println("Robot orientation OK");
  }
  
  // STEP 3: Check vibration stability (KW12-3)
  Serial.println("Checking surface stability (KW12-3)...");
  int vibrationCount = 0;
  for (int i = 0; i < 10; i++) {
    if (readVibration()) vibrationCount++;
    delay(100);
  }
  
  if (vibrationCount > 3) {
    Serial.println("Surface unstable - high vibration detected");
    Serial.println("Moving to stable position...");
    moveForward();
    delay(500);
    stopAllMotors();
    delay(300);
  } else {
    Serial.println("Surface stable - ready for game");
  }
  
  // STEP 4: Check motor current status (ACS712)
  Serial.println("Motor system check (ACS712)...");
  stopAllMotors();
  delay(500);
  
  float idleCurrent = readCurrent();
  Serial.print("Idle current: "); Serial.print(idleCurrent); Serial.println("A");
  
  // Test motor current draw
  moveForward();
  delay(200);
  float activeCurrent = readCurrent();
  stopAllMotors();
  
  Serial.print("Active current: "); Serial.print(activeCurrent); Serial.println("A");
  
  if (activeCurrent > OVERCURRENT_THRESHOLD) {
    Serial.println("Motor overcurrent - system abort!");
    delay(1000);
    tictactoeMode = false;
    return;
  } else if (activeCurrent < 0.1) {
    Serial.println("Motor not responding - check connections!");
    delay(1000);
    tictactoeMode = false;
    return;
  } else {
    Serial.println("Motor current normal");
  }
  
  // Simulate game moves with sensor verification
  int moves[] = {5, 1, 3, 7, 9}; // Center, corners strategy
  
  for (int i = 0; i < 5; i++) {
    Serial.print("\nMove "); Serial.print(i+1); 
    Serial.print(": Positioning to cell "); Serial.println(moves[i]);
    
    // Navigate to position using ultrasonic and IMU
    navigateToCell(moves[i]);
    
    // Check vibration before placing marker
    if (readVibration()) {
      Serial.println("Vibration detected - waiting for stability...");
      delay(500);
    }
    
    // Check current during marker placement
    Serial.println("Placing marker...");
    // Simulate marker placement mechanism
    delay(1000);
    
    currentSensor = readCurrent();
    if (currentSensor > CURRENT_THRESHOLD) {
      Serial.println("High current during placement - checking...");
      delay(300);
    }
    
    Serial.println("Marker placed successfully!");
    delay(500);
  }
  
  returnToHome();
  
  tictactoeMode = false;
  Serial.println("✓ Tic-Tac-Toe mode completed!\n");
}

// Helper function to navigate to specific cell
void navigateToCell(int cell) {
  
  Serial.print("Moving to cell "); Serial.println(cell);
  
  // Use ultrasonic to measure position
  float targetDistance;
  
  // Simplified navigation logic
  switch(cell) {
    case 1: case 2: case 3: // Top row
      targetDistance = 40.0;
      break;
    case 4: case 5: case 6: // Middle row
      targetDistance = 25.0;
      break;
    case 7: case 8: case 9: // Bottom row
      targetDistance = 10.0;
      break;
    default:
      targetDistance = 25.0;
  }
  
  // Navigate forward/backward to reach target distance
  distance = readUltrasonic();
  while (abs(distance - targetDistance) > 2.0) {
    distance = readUltrasonic();
    
    if (distance > targetDistance) {
      moveForward();
    } else {
      moveBackward();
    }
    delay(100);
  }
  stopAllMotors();
  
  // Lateral positioning (left/right)
  int column = ((cell - 1) % 3) + 1;
  if (column == 1) {
    Serial.println("Adjusting to left column");
    turnLeft();
    delay(300);
    moveForward();
    delay(400);
    stopAllMotors();
  } else if (column == 3) {
    Serial.println("Adjusting to right column");
    turnRight();
    delay(300);
    moveForward();
    delay(400);
    stopAllMotors();
  } else {
    Serial.println("Center column - no lateral adjustment");
  }
  
  delay(300);
}

// Helper function to return to home position
void returnToHome() {
  Serial.println("Navigating back to home position...");
  // Use ultrasonic to find safe distance
  moveBackward();
  delay(1000);
  
  while (distance < 50.0) {
    distance = readUltrasonic();
    moveBackward();
    delay(100);
  }
  
  stopAllMotors();
  
  // Final sensor check
  Serial.print("  Distance: "); Serial.print(distance); Serial.println("cm");
  Serial.print("  Vibration: "); Serial.println(readVibration() ? "Detected" : "None");
  Serial.print("  Current: "); Serial.print(readCurrent()); Serial.println("A");
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("  IMU Z-axis: "); Serial.println(az);
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Setup pin motor
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);
  
  // Setup ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  delay(1000);
  
  // Inisialisasi semua sensor
  initSensors();
  
  delay(2000);
  systemRunning = true;
}

void loop() {
  // START
  if (!systemRunning) {
    stopAllMotors();
    return;
  }
  
  // Baca semua sensor secara periodik
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    readAllSensors();
    lastSensorRead = millis();
  }
  
  // CRITICAL: Check current sensor for motor protection
  if (currentSensor > OVERCURRENT_THRESHOLD) {
    Serial.println("CRITICAL: MOTOR OVERCURRENT DETECTED!");
    stopAllMotors();
    delay(2000);
    return;
  }
  
  // Check vibration sensor (KW12-3) - might indicate collision or unstable surface
  if (vibrationDetected) {
    Serial.println("Vibration detected - checking stability...");
    stopAllMotors();
    delay(300);
    
    // Verify if it's persistent vibration
    bool stillVibrating = readVibration();
    if (stillVibrating) {
      Serial.println("Persistent vibration - possible collision!");
      handleObstacle();
      return;
    }
  }
  
  // Check IMU (MPU6050) for abnormal acceleration/tilt
  if (!checkIMUAcceleration()) {
    Serial.println("IMU Alert: Abnormal acceleration/orientation!");
    handleObstacle();
    return;
  }
  
  // Check proximity sensor (APDS9960) and ultrasonic (HC-SR04) for obstacles
  if (proximity > PROXIMITY_THRESHOLD || distance < OBSTACLE_DISTANCE) {
    obstacleDetected = true;
    Serial.println("Obstacle detected by proximity/ultrasonic sensors");
    handleObstacle();
  } else {
    obstacleDetected = false;
    
    // Normal movement - check distance for speed control
    if (distance > SAFE_DISTANCE) {
      moveForward();
    } else if (distance > OBSTACLE_DISTANCE) {
      // Moderate distance - reduced speed for safety
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
  
  // Check if robot is near home position (distance > 80cm means far from obstacles)
  if (distance > 80.0 && distance < 100.0 && !obstacleDetected) {
    nearHomePosition = true;
  }
  
  // Trigger tic-tac-toe mode periodically or when near home
  if ((millis() - lastGameMode > 60000) || 
      (nearHomePosition && (millis() - lastGameMode > 30000))) {

    stopAllMotors();
    delay(1000);
    
    performTicTacToeMode();
    
    lastGameMode = millis();
    nearHomePosition = false;
  }
  
  delay(50);  // Small delay for stability
}