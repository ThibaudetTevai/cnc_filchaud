#include "U8glib.h"

#ifndef LCDMATRIX_H_
#define LCDMATRIX_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "HardwareSerial.h"

#define COL 21
#define ROW 5

class LcdMatrix{
  public:
    LcdMatrix ();
    void setup(HardwareSerial &print );
    void printMatrix();
    void printLcd(uint8_t, uint8_t, char*);
    void clearLcd();
  private:
    void setParameter();
    
    HardwareSerial* printer;
    uint8_t WIDTH_FONT;
    uint8_t HEIGH_FONT;
    uint8_t WIDTH_SCREEN;
    uint8_t HEIGH_SCREEN;
    uint8_t NB_COL;
    uint8_t NB_ROW;
    U8GLIB_ST7920_128X64_1X *ptrU8g;
    char screenContent[COL*ROW];
};

#endif
