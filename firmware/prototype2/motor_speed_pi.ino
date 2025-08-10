/*
 Prototype 2 - DC motors + encoders + PI speed loop (skeleton)
 Guidance layer expected to set vrefL, vrefR (rad/s).
*/
#include <FlexiTimer2.h>

/*********** CONFIG ***********/
// Encoder pins (example, adjust to board)
#define ENC_L_A 2
#define ENC_L_B 3
#define ENC_R_A 18
#define ENC_R_B 19
// Motor driver pins
#define MOT_L_DIR 4
#define MOT_L_PWM 5
#define MOT_R_DIR 9
#define MOT_R_PWM 8
// Ticks per wheel revolution (post decoding)
const float TICKS_PER_REV = 1632.0f; // adjust from hardware
// Wheel & battery constants (for future kinematics / voltage comp)
const float WHEEL_RADIUS = 0.032f; // m
const float WHEEL_BASE   = 0.12f;  // m
const float BATTERY_VOLT = 11.1f;  // nominal
// Control gains (example values)
volatile float KP = 0.29f;
volatile float KI = 8.93f;
// Scheduler
const uint16_t SAMPLE_MS = 10; // 100 Hz
/*********** END CONFIG ***********/

volatile long ticksL = 0;
volatile long ticksR = 0;
volatile float omegaL = 0.f;
volatile float omegaR = 0.f;
volatile float vrefL = 0.f;
volatile float vrefR = 0.f;
volatile float I_L = 0.f;
volatile float I_R = 0.f;

static inline void motorWrite(int dirPin,int pwmPin,float cmd){
  // cmd: pseudo voltage (V), saturate to +/-BATTERY_VOLT
  if(cmd>BATTERY_VOLT) cmd = BATTERY_VOLT;
  if(cmd<-BATTERY_VOLT) cmd = -BATTERY_VOLT;
  int pwm = (int)(255.f * fabs(cmd)/BATTERY_VOLT);
  if(pwm>255) pwm=255;
  if(cmd>=0){ digitalWrite(dirPin, HIGH); analogWrite(pwmPin,pwm);} else { digitalWrite(dirPin, LOW); analogWrite(pwmPin,pwm);} }

void encL_A(){ if(digitalRead(ENC_L_A)==digitalRead(ENC_L_B)) ticksL--; else ticksL++; }
void encL_B(){ if(digitalRead(ENC_L_A)==digitalRead(ENC_L_B)) ticksL++; else ticksL--; }
void encR_A(){ if(digitalRead(ENC_R_A)==digitalRead(ENC_R_B)) ticksR--; else ticksR++; }
void encR_B(){ if(digitalRead(ENC_R_A)==digitalRead(ENC_R_B)) ticksR++; else ticksR--; }

void controlISR(){
  static const float dt = SAMPLE_MS/1000.f;
  long dL = ticksL; ticksL = 0;
  long dR = ticksR; ticksR = 0;
  omegaL = (2.f*PI * (float)dL / TICKS_PER_REV)/dt;
  omegaR = (2.f*PI * (float)dR / TICKS_PER_REV)/dt;
  float eL = vrefL - omegaL;
  float eR = vrefR - omegaR;
  float uL = KP*eL + I_L;
  float uR = KP*eR + I_R;
  I_L += KI*dt*eL;
  I_R += KI*dt*eR;
  motorWrite(MOT_L_DIR,MOT_L_PWM,uL);
  motorWrite(MOT_R_DIR,MOT_R_PWM,uR);
}

void setup(){
  pinMode(ENC_L_A,INPUT_PULLUP); pinMode(ENC_L_B,INPUT_PULLUP);
  pinMode(ENC_R_A,INPUT_PULLUP); pinMode(ENC_R_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A),encL_A,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B),encL_B,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A),encR_A,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B),encR_B,CHANGE);
  pinMode(MOT_L_DIR,OUTPUT); pinMode(MOT_L_PWM,OUTPUT);
  pinMode(MOT_R_DIR,OUTPUT); pinMode(MOT_R_PWM,OUTPUT);
  Serial.begin(115200);
  FlexiTimer2::set(SAMPLE_MS, controlISR);
  FlexiTimer2::start();
  Serial.println(F("Proto2 Motor Speed PI started"));
}

void loop(){
  // Simple demo: sweep references (replace with vision guidance interface)
  static uint32_t t0 = millis();
  uint32_t t = millis()-t0;
  if(t<3000){ vrefL = vrefR = 5.0f; }
  else if(t<6000){ vrefL = vrefR = 10.0f; }
  else { t0 = millis(); }
  if(t % 500 == 0){
    Serial.print(F("wL="));Serial.print(omegaL,2);
    Serial.print(F(",wR="));Serial.print(omegaR,2);
    Serial.print(F(",vref="));Serial.print(vrefL,1);
    Serial.print(F(",I_L="));Serial.print(I_L,2);
    Serial.print(F(",I_R="));Serial.println(I_R,2);
  }
}
