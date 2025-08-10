/*
 Prototype 1 - Servo based differential drive with Pixy2 lateral PI guidance
 Curated minimal sketch (original workspace cleaned).
*/
#include <Pixy2.h>
#include <Servo.h>

/*********** CONFIG ***********/
#define SERVO_LEFT_PIN 9
#define SERVO_RIGHT_PIN 8
#define PIXY_SIG_ID 1
#define X_CENTER 160L  // Pixy2 horizontal center (for 320x200 mode)
// Guidance PI gains (empirical)
const float KP = 0.8f;   // proportional on pixel error
const float KI = 0.0f;   // set >0 to remove residual bias (servo asymmetry may limit usefulness)
const int BASE_CMD = 0;  // forward bias if needed
const int SAMPLE_MS = 50; // guidance update period
/*********** END CONFIG ***********/

Pixy2 pixy;
Servo servoL;
Servo servoR;

int32_t errInt = 0; // integral accumulator (raw pixel*ms)
unsigned long lastUpdate = 0;

static int clamp(int v,int lo,int hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }

void setup(){
  servoL.attach(SERVO_LEFT_PIN);
  servoR.attach(SERVO_RIGHT_PIN);
  Serial.begin(115200);
  pixy.init();
  Serial.println(F("Proto1 Servo Guidance PI started"));
}

void loop(){
  if(millis()-lastUpdate < SAMPLE_MS) return;
  lastUpdate = millis();
  pixy.ccc.getBlocks();
  if(pixy.ccc.numBlocks){
    // choose largest block of signature PIXY_SIG_ID
    int best = -1; uint32_t bestArea = 0;
    for(uint16_t i=0;i<pixy.ccc.numBlocks;i++){
      if(pixy.ccc.blocks[i].m_signature==PIXY_SIG_ID){
        uint32_t area = (uint32_t)pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
        if(area>bestArea){ bestArea=area; best=i; }
      }
    }
    if(best>=0){
      int x = pixy.ccc.blocks[best].m_x;
      int32_t err = x - X_CENTER; // pixel error
      errInt += err * SAMPLE_MS;   // integrate (ms weight)
      // PI law -> differential speed cmd range [-400,400]
      float u = KP * err + KI * (errInt/1000.0f); // integral scaled to pixel*sec
      int leftCmd = BASE_CMD + (int)u;
      int rightCmd = BASE_CMD - (int)u;
      leftCmd = clamp(leftCmd,-400,400);
      rightCmd = clamp(rightCmd,-400,400);
      // Map to servo pulses (assumes 90 stop, 0/180 full speed opposite)
      int servoLeft = map(leftCmd,-400,400,90,180);
      int servoRight = map(rightCmd,-400,400,90,0);
      servoL.write(servoLeft);
      servoR.write(servoRight);
      Serial.print(F("ERR="));Serial.print(err);
      Serial.print(F(",U="));Serial.print(u,1);
      Serial.print(F(",L="));Serial.print(servoLeft-90);
      Serial.print(F(",R="));Serial.println(90-servoRight);
    }
  } else {
    // no detection -> stop
    servoL.write(90); servoR.write(90);
    Serial.println(F("NO_BALL"));
  }
}
