#include "../bounce2/Bounce2.h"

#ifndef ROTBTN_H_
#define ROTBTN_H_

class RotBtn
{
  public:
    RotBtn (uint8_t, uint8_t, uint8_t);
    void    rotBtnRefresh();
    bool    getCntUpdated();
    uint8_t getValueRot();
    bool    getValueBtn();
    bool    isCntUpdated();
    bool    isBtnPushed();
  void  resetValueRot();
    
  private:
    void doDebounce();
    void doEncoderRead();
    void doUpdateCounter();
    void doUpdateBtnUpdate();

    bool valueA;
    bool valueB;
    bool valueP;
    bool btnPressed;
    bool motionDetected;
    bool cntUpdated;
    bool CW;

    int8_t nettCounter;

    Bounce debouncerA;
    Bounce debouncerB;
    Bounce debouncerSWITCH;
};

#endif