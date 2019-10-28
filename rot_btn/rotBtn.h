#include <Bounce2.h>
#include "rotBtn.h"

#ifndef ROTBTN_H_
#define ROTBTN_H_

void   setupBtn(byte pinA, byte pinb, byte pinPush) 
void  rotBtnRefresh();

bool  getPushedStatus();
bool  getCntUpdated();
uint8_t getValueRot();

void  doDebounce();
void  doEncoderRead();
void  doUpdateCounter();

#endif
