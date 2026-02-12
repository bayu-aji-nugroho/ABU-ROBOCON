#include <Arduino.h>
#include <../lib/movementLIB/movement.h>
#include <HardwareSerial.h>
#include <PS4Controller.h>
#include <nvs_flash.h>

Movement *Fr,*Fl,*Br, *Bl;
void control();
int ppr = 7;

float targetHeading = 0.0;     
bool headingLockActive = true;

void setup() {
  Serial.begin(115200);
  nvs_flash_erase(); 
  nvs_flash_init();
  delay(2000);

  PS4.begin("40:1A:58:62:D6:A2"); 
  
  Serial.println("program berjalan");
  
  // Kp,Ki,Kd, chanel A, chanel B, ppr, rpwm, lpwm
  //note cha a dan b pake resistor 4.7k Ohm yang dihubungkan ke 3.3v
  Fr = new Movement(0.1,0.01,0.1,34,35,ppr,12,13);
  // Fl = new Movement(0.1,0.01,0.1,36,39,ppr,14,27);
  // Bl = new Movement(0.1, 0.01, 0.1, 32, 33, ppr, 26, 25);
  // Br = new Movement(0.1, 0.01, 0.1, 16, 17, ppr, 19, 18);

  Fr->begin();
  // Fl->begin();
  // Br->begin();
  // Bl->begin();

  Fr->resetHeading();
  targetHeading = 0.0;
  
}

void loop() {
  Fr -> gyroUpdate()

  if (Fr->isTilted(20.0)) {
    // Robot miring >20°, matikan semua motor
    Fr->update(0);
    Fl->update(0);
    Br->update(0);
    Bl->update(0);
    
    Serial.println("Motor dimatikan");
    delay(100);
    return;
  }

  if(PS4.isConnected()){
    control();
  }

  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 100) { //100 ms
    lastDebug = millis();
    Serial.print("Pitch: "); Serial.print(Fr->getPitch(), 1);
    Serial.print("° | Roll: "); Serial.print(Fr->getRoll(), 1);
    Serial.print("° | Heading: "); Serial.print(Fr->getHeading(), 1);
    Serial.print("° | Target: "); Serial.print(targetHeading, 1);
    Serial.println("°");
  }
  
}

void move(int forward, int strafe, int turn){

  int threshold = 8; // Deadzone, nilai def gamepad kadang error

  if (abs(turnInput) > threshold) {
    headingLockActive = false; // Matikan lock sementara
    
    float headingChange = (turnInput / 127.0) * 2.0; // Adjust sensitivity 
    targetHeading += headingChange;
    
    if (targetHeading > 180.0)  targetHeading -= 360.0;
    if (targetHeading < -180.0) targetHeading += 360.0;
    
    Fr->setTargetHeading(targetHeading);
  } else {
    headingLockActive = true;
  }

  float headingCorr = 0;
  float tiltCorr = 0;

  if (headingLockActive){
    headingCorr = Fr -> getHeadingCorrection(40.0) // Limit correction 40 pwm
  }
  
  tiltCorr = Fr->getTiltCorrection(0.0, 0.0, 30.0); // Limit 30 PWM

  if (abs(forward) < threshold && abs(strafe) < threshold && abs(turn) < threshold) {
    Fr->resetPID(); 
    // Fl->resetPID();
    // Br->resetPID();
    // Bl->resetPID();

    forward = 0;
    strafe = 0;
    // turn = 0;
  }
  // int vfl = forward + strafe + turn; 
  // int vfr = forward - strafe - turn; 
  // int vbl = forward - strafe + turn;
  // int vbr = forward + strafe - turn;

  // Turn -> headingCorr dari gyro
  float vfl = forward + strafe + headingCorr + tiltCorr;
  float vfr = forward - strafe - headingCorr + tiltCorr;
  float vbl = forward - strafe + headingCorr + tiltCorr;
  float vbr = forward + strafe - headingCorr + tiltCorr;
  
  float max_val = std::max({abs(vfl), abs(vfr), abs(vbl), abs(vbr)});

  if (max_val > 255) {
    vfl = (vfl / max_val) * 255;
    vfr = (vfr / max_val) * 255;
    vbl = (vbl / max_val) * 255;
    vbr = (vbr / max_val) * 255;
  }
  // float Vfr = map(vfr, -255, 255, -100, 100);  
  // float Vfl = map(vfl, -255, 255, -100, 100); 
  // float Vbr = map(vbr, -255, 255, -100, 100);  
  // float Vbl = map(vbl, -255, 255, -100, 100);  

  Fr->update(vfr);
  // Fl->update(vfl);
  // Br->update(vbr);
  // Bl->update(vbl);                                    
}

void control(){
  
  if(PS4.Triangle()){
    Serial.println("segitiga di tekan");
  }
  if(PS4.LStickY() || PS4.LStickX() || PS4.RStickX() ){
    move(PS4.LStickY(), PS4.LStickX(),PS4.RStickX());
  }
  if(PS4.Square()){
    Serial.println("kotak di tekan");
  }
  if(PS4.Circle()){
    Serial.println("lingkaran di tekan");   
    delay(200);
  }
  
}