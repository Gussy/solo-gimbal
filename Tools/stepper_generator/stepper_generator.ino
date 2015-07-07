#include "Arduino.h"

#define STEPS 12800

unsigned long last_step_time = 0;
unsigned long step_delay = 0;
int incomingByte = 0;   // for incoming serial data

void setup(){
  Serial.begin(115200);
  
  pinMode(8, OUTPUT);
  setSpeed(0);
}

void loop(){
  incomingByte = Serial.read();
  if(incomingByte>=0){
    setSpeed(incomingByte);
  }
  stepperLoop();
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
    digitalWrite(8, HIGH);
    delayMicroseconds(1);
    digitalWrite(8, LOW);
  }
}
