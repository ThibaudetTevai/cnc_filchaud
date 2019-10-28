#include <Bounce2.h>
#include "rotBtn.h"

bool valueA;
bool valueB;
bool btnPressed;
bool motionDetected;
bool cntUpdated;
bool CW;

int nettCounter;

Bounce debouncerA;
Bounce debouncerB;
Bounce debouncerSWITCH;


void rotBtnRefresh() {
  doDebounce();
  doEncoderRead();
  updateCounter();
}

bool getPushedStatus(){
  return btnPressed;
}

int getValueRot(){
  return nettCounter;
}

bool isUpdated(){
  if(cntUpdated){
    cntUpdated = false;
    return true;
  }
  return false;
}

void setupBtn(byte pinA, byte pinB, byte pinBtnSwitch) {
  Serial.begin(115200);
  Serial.println("Setup");

  // Flag
  btnPressed = false;
  motionDetected = false;
  cntUpdated = false;

  // Counter
  nettCounter = 0;
  
  // Instantiate 2 Bounce object
  debouncerA = Bounce(); 
  debouncerB = Bounce(); 
  debouncerSWITCH = Bounce();

  // Setup the buttons
  pinMode(pinA,INPUT_PULLUP);
  pinMode(pinB,INPUT_PULLUP);
  pinMode(pinBtnSwitch,INPUT_PULLUP);
  
  // After setting up the button, setup debouncer
  debouncerA.attach(pinA);
  debouncerA.interval(5);
  debouncerB.attach(pinB);
  debouncerB.interval(5);
  debouncerSWITCH.attach(pinBtnSwitch);
  debouncerSWITCH.interval(5);
}

void doDebounce(){
  debouncerA.update();
  debouncerB.update();
  debouncerSWITCH.update();
} //doDebounce

void doEncoderRead(){
  valueA = debouncerA.read();
  valueB = debouncerB.read();
  btnPressed = !debouncerSWITCH.read();
}

void updateCounter(){
    if (valueA && valueB && motionDetected ){ //in a detent and just arrived
      if (CW) nettCounter++; //cw +ve
      else nettCounter--; //ccw -ve
      motionDetected = false;
      cntUpdated = true;
    }

    if(!motionDetected) {
      if (valueA ^ valueB){ // valueA XOR valueB
         CW = (valueA) ? false : true; // Define if CW or CCW
         motionDetected=true; // just started CW  
      } 
    }    
}