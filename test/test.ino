#include <Arduino.h>
#include "rotBtn.h"
#include <SoftwareSerial.h>

#define PINA 31
#define PINB 32
#define PINS 35

SoftwareSerial mySerial(2, 3); // RX, TX

RotBtn rotBtn(PINA,PINB,PINS);
    
void setup(void) {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("start");
}

void loop(void) { 
  if(rotBtn.isUpdated()){    
      Serial.println(rotBtn.getPushedStatus());
      Serial.println(rotBtn.getValueRot());
  }
  rotBtn.rotBtnRefresh();  
}
