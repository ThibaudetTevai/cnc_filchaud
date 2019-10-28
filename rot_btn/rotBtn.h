#include <Bounce2.h>

#ifndef ROTBTN_H_
#define ROTBTN_H_

void  	setupBtn(byte, byte, byte);
void  	rotBtnRefresh();

bool  	getPushedStatus();
bool  	getCntUpdated();
uint8_t getValueRot();

void  	doDebounce();
void  	doEncoderRead();
void  	doUpdateCounter();

#endif
