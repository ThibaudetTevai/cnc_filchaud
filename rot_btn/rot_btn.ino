#include <Bounce2.h>

/* 
 DESCRIPTION
 ====================
 Reads the 2 switches in an encoder, 
 determines direction,
 updates counter.
 No interrupts.
 Switches debounced, didn't test first, just did it anyway.
 Pushing the knob in, resuses A and B as D and E
 D is paired with A; E with B
 */

const byte ENCODER_PINA= 33;  //A&C
const byte ENCODER_PINB= 31; //B&D
const byte SWITCH_PIN= 35;

bool valueA;
bool valueB;
bool btnPressed = false;
bool motionDetected = false;
bool CW;

// not pressed
int grossCounter = 0; // total steps
int nettCounter = 0;  // cw-ccw

// pressed; suffix P
int grossCounterP = 0; // total steps
int nettCounterP = 0;  // cw-ccw

// Instantiate 2 Bounce object
Bounce debouncerA = Bounce(); 
Bounce debouncerB = Bounce(); 
Bounce debouncerSWITCH = Bounce();

void setup() {
  setupBtn();
}
void loop(){
  rotBtnRefresh();  
}

void rotBtnRefresh() {
  // Update the debouncers
  doDebounce();
  // Read the encoder switches
  doEncoderRead();
  //determine direction and update counter
  updateCounter();
}

bool getPushedStatus(){
  return !btnPressed;
}

int getValueRot(){
  return nettCounter;
}


void setupBtn() {
  Serial.begin(115200);
  Serial.println("Setup");
  // Setup the buttons
  pinMode(ENCODER_PINA,INPUT_PULLUP);
  pinMode(ENCODER_PINB,INPUT_PULLUP);
  pinMode(SWITCH_PIN,INPUT_PULLUP);

  // After setting up the button, setup debouncer
  debouncerA.attach(ENCODER_PINA);
  debouncerA.interval(5);
  debouncerB.attach(ENCODER_PINB);
  debouncerB.interval(5);
  debouncerSWITCH.attach(SWITCH_PIN);
  debouncerSWITCH.interval(5);
  Serial.println("Setup done");
  Serial.println();
}

// my functions **************************************************
void doDebounce(){
  debouncerA.update();
  debouncerB.update();
  debouncerSWITCH.update();
} //doDebounce

void doEncoderRead(){
  valueA = debouncerA.read();
  valueB = debouncerB.read();
  btnPressed = !debouncerSWITCH.read();
} //doEncoderRead

void updateCounter(){
    if (valueA && valueB && motionDetected ) //in a detent and just arrived
    {
      if (CW) nettCounter++; //cw +ve
      else nettCounter--; //ccw -ve
      motionDetected = false;
      Serial.println("Pressed");
      Serial.println(nettCounter);
    }

    if (valueA && !valueB && !motionDetected ){ // just started CW
      CW= true;
      motionDetected=true;
    }

    if (!valueA && valueB && !motionDetected )  //just started CCW
    {
      CW= false;
      motionDetected=true;
    }
} //updateCounter
