// Definisi Pin PWM
const int RPWM = 5; 
const int LPWM = 6; 

void setup() {
  // Hanya perlu setting pin PWM sebagai output
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  
  // Memastikan awal dimulai dari kondisi diam
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);

  Serial.begin(9600);
  Serial.println("--- BTS7960 Health Check (EN Jumpered) ---");
}

void loop() {
  // --- TEST ARAH A ---
  Serial.println("Menguji Sisi RPWM (Maju)...");
  analogWrite(LPWM, 0);          // Pastikan sisi lawan mati total
  for (int speed = 0; speed <= 200; speed += 5) {
    analogWrite(RPWM, speed);    // Naikkan kecepatan perlahan
    delay(30);
  }
  delay(2000);                   // Putar selama 2 detik
  
  // Berhenti perlahan
  for (int speed = 200; speed >= 0; speed -= 5) {
    analogWrite(RPWM, speed);
    delay(10);
  }
  delay(1000);

  // --- TEST ARAH B ---
  Serial.println("Menguji Sisi LPWM (Mundur)...");
  analogWrite(RPWM, 0);          // Pastikan sisi lawan mati total
  for (int speed = 0; speed <= 200; speed += 5) {
    analogWrite(LPWM, speed);    // Naikkan kecepatan perlahan
    delay(30);
  }
  delay(2000);                   // Putar selama 2 detik

  // Berhenti perlahan
  for (int speed = 200; speed >= 0; speed -= 5) {
    analogWrite(LPWM, speed);
    delay(10);
  }
  delay(1000);
}