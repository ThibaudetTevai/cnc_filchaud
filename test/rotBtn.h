#include <Bounce2.h>

#ifndef ROTBTN_H_
#define ROTBTN_H_

class RotBtn
{
  public:
    RotBtn (uint8_t, uint8_t, uint8_t);
    void    rotBtnRefresh();
    bool    getPushedStatus();
    bool    getCntUpdated();
    uint8_t getValueRot();
    bool    isUpdated();
    
	private:
		void doDebounce();
		void doEncoderRead();
		void doUpdateCounter();

		bool valueA;
		bool valueB;
		bool btnPressed;
		bool motionDetected;
		bool cntUpdated;
		bool CW;

		uint8_t nettCounter;

		Bounce debouncerA;
		Bounce debouncerB;
		Bounce debouncerSWITCH;
};

#endif
