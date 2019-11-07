#include "U8glib.h"
#include "../lcd/lcdMatrix.h"
#include "../rotBtn/rotBtn.h"
#include <SoftwareSerial.h>

#define PINA 31
#define PINB 33
#define PINS 35

SoftwareSerial mySerial(2, 3); // RX, TX
LcdMatrix lcdMatrix;

RotBtn rotBtn(PINA,PINB,PINS);

char bufferItoa[4]; 

void setup(void) {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("start");
    lcdMatrix.setup();
    lcdMatrix.printLcd(0, 1, "       Compteur      ");
    lcdMatrix.printLcd(0, 2, itoa(rotBtn.getValueRot(),bufferItoa,10));
    lcdMatrix.printMatrix();
}

void loop(void) {
  if(rotBtn.isCntUpdated()){
      Serial.println(itoa(rotBtn.getValueRot(),bufferItoa,10));
      lcdMatrix.printLcd(rotBtn.getValueRot(), 2, itoa(rotBtn.getValueRot(),bufferItoa,10));
      lcdMatrix.printMatrix();
  }
  if(rotBtn.isBtnPushed()) {
    lcdMatrix.clearLcd();
  }
  rotBtn.rotBtnRefresh();  
}
