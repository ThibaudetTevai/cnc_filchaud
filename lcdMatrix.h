#include <U8g2lib.h>

#ifndef LCDMATRIX_H_
#define LCDMATRIX_H_

#define COL 21
#define ROW 5

class LcdMatrix{
  public:
    LcdMatrix ();
    void setup();
    void printMatrix();
    void printLcd(uint8_t, uint8_t, const char*);
    void clearLcd();
    bool isupdated();
  private:
    void setParameter();

    bool lcdUpdated; 

    uint8_t WIDTH_FONT;
    uint8_t HEIGH_FONT;
    uint8_t WIDTH_SCREEN;
    uint8_t HEIGH_SCREEN;
    uint8_t NB_COL;
    uint8_t NB_ROW;

    U8G2_ST7920_128X64_F_SW_SPI *ptrU8g;

    char screenContent[COL*ROW];
};

#endif