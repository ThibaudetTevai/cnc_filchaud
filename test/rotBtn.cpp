#include "rotBtn.h"

RotBtn::RotBtn(uint8_t pinA, uint8_t pinB, uint8_t pinBtnSwitch) {
  Serial.begin(115200);
  Serial.println("Setup");

  // Flag
  this->btnPressed = false;
  this->motionDetected = false;
  this->cntUpdated = false;

  // Counter
  this->nettCounter = 0;
  
  // Instantiate 2 Bounce object
  this->debouncerA = Bounce(); 
  this->debouncerB = Bounce(); 
  this->debouncerSWITCH = Bounce();

  // Setup the buttons
  pinMode(pinA,INPUT_PULLUP);
  pinMode(pinB,INPUT_PULLUP);
  pinMode(pinBtnSwitch,INPUT_PULLUP);
  
  // After setting up the button, setup debouncer
  this->debouncerA.attach(pinA);
  this->debouncerA.interval(5);
  this->debouncerB.attach(pinB);
  this->debouncerB.interval(5);
  this->debouncerSWITCH.attach(pinBtnSwitch);
  this->debouncerSWITCH.interval(5);
}

void RotBtn::rotBtnRefresh() {
  doDebounce();
  doEncoderRead();
  doUpdateCounter();
  doUpdateBtnUpdate();
}

bool RotBtn::isBtnPushed(){
  if(this->btnPressed){
    
    this->btnPressed = false;
    return true;
  }
  return false;
}

bool RotBtn::isCntUpdated(){
  if(this->cntUpdated){
    this->cntUpdated = false;
    return true;
  }
  return false;
}

uint8_t RotBtn::getValueRot(){
  return this->nettCounter;
}

bool RotBtn::getValueBtn(){
  return this->valueP;
}

void RotBtn::doDebounce(){
  this->debouncerA.update();
  this->debouncerB.update();
  this->debouncerSWITCH.update();
}

void RotBtn::doEncoderRead(){
  this->valueA = this->debouncerA.read();
  this->valueB = this->debouncerB.read();
  this->valueP = !this->debouncerSWITCH.read();
}

void RotBtn::doUpdateCounter(){
    if (this->valueA && this->valueB && this->motionDetected ){ //in a detent and just arrived
      if (this->CW) this->nettCounter++; //cw +ve
      else this->nettCounter--; //ccw -ve
      this->motionDetected = false;
      this->cntUpdated = true;
    }

    if(!this->motionDetected) {
      if (!this->valueA && this->valueB){ // valueA XOR valueB
         this->CW = true; // Define if CW or CCW
         this->motionDetected=true; // just started CW  
      } 
      if (this->valueA && !this->valueB){ // valueA XOR valueB
         this->CW = false; // Define if CW or CCW
         this->motionDetected=true; // just started CW  
      } 
    }    
}

void RotBtn::doUpdateBtnUpdate(){
  if(this->valueP) this->btnPressed = true;
}
