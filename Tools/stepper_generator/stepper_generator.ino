#include "Arduino.h"

#define STEPS 3200
#define PIN_STEP 8
#define PIN_RELAY1 9
#define PIN_RELAY2 10


unsigned long last_step_time = 0;
unsigned long step_delay = 0;
int incomingByte = 0;   // for incoming serial data

void setup(){
  Serial.begin(115200);
  
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_RELAY1, OUTPUT);
  pinMode(PIN_RELAY2, OUTPUT);

  setSpeed(0);
}

void loop(){
  stepperLoop();

  incomingByte = Serial.read();
  if(incomingByte>=0){
    if(incomingByte <= 250){
      setSpeed(incomingByte);
    }else switch(incomingByte){
      case 251:
        digitalWrite(PIN_RELAY1, HIGH);
        break;
      case 252:
        digitalWrite(PIN_RELAY1, LOW);
        break;
      case 253:
        digitalWrite(PIN_RELAY2, HIGH);
        break;
      case 254:
        digitalWrite(PIN_RELAY2, LOW);
        break;
      default:
        break;
    }
  }

}

void setSpeed(long whatSpeed){
  if(whatSpeed == 0){
    step_delay = 0;
  } else{
    step_delay = 60L * 1000L * 1000L / STEPS / whatSpeed;
  }
}

void stepperLoop(){
   unsigned long now = micros();
  if ((step_delay!=0)&&(now - last_step_time >= step_delay)){
    last_step_time = now;
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_STEP, LOW);
  }
}
