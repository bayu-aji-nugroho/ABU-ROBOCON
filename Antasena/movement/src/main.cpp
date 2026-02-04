#include <Arduino.h>
#include <../lib/movementLIB/movement.h>
#include <HardwareSerial.h>
#include <PS4Controller.h>
#include <nvs_flash.h>

Movement *Fr,*Fl,*Br, *Bl;
void control();
int ppr = 7;

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
  
}

void loop() {
  if(PS4.isConnected()){
    control();
  }
  
}

void move(int forward, int strafe, int turn){

  int threshold = 8; // Deadzone, nilai def gamepad kadang error
  if (abs(forward) < threshold && abs(strafe) < threshold && abs(turn) < threshold) {
    Fr->resetPID(); 
    // Fl->resetPID();
    // Br->resetPID();
    // Bl->resetPID();

    forward = 0;
    strafe = 0;
    turn = 0;
  }
  int vfl = forward + strafe + turn; 
  int vfr = forward - strafe - turn; 
  int vbl = forward - strafe + turn;
  int vbr = forward + strafe - turn;

  
  
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