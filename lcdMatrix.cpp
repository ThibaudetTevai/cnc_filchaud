#include "lcdMatrix.h"

LcdMatrix::LcdMatrix(){};

void LcdMatrix::setup(HardwareSerial &print ){
  printer = &print; //operate on the adress of print
  U8GLIB_ST7920_128X64_1X u8gTemp(23,17,16);
  this->ptrU8g = &u8gTemp;
  this->ptrU8g->setFont(u8g_font_6x13);  
  setParameter();
}

void LcdMatrix::setParameter() { 
  this->WIDTH_SCREEN = this->ptrU8g->getWidth();
  this->HEIGH_SCREEN = this->ptrU8g->getHeight();
  this->HEIGH_FONT = this->ptrU8g->getFontLineSpacing();
  do {
      this->WIDTH_FONT = this->ptrU8g->getStrWidth("A");
  } while (this->ptrU8g->nextPage());
  
  this->NB_COL = this->WIDTH_SCREEN / this->WIDTH_FONT;
  this->NB_ROW = this->HEIGH_SCREEN / this->HEIGH_FONT;
}

void LcdMatrix::printMatrix() { 
    //for (uint8_t i = 0; i<(this->NB_ROW * this->NB_COL); i++) printer->println(this->screenContent[i]);
    this->ptrU8g->firstPage();
    do {
        for (uint8_t i = 0; i < (this->NB_ROW); i++) {
            char toPrint[(this->NB_COL) + 1];
            for (uint8_t j = 0; j < this->NB_COL; j++){
              toPrint[j] = this->screenContent[(i * this->NB_COL) + j];
            }
            toPrint[this->NB_COL] = '\0';
            this->ptrU8g->setPrintPos(0, this->HEIGH_FONT * (i + 1));
            this->ptrU8g->print(toPrint);
        }
    } while (this->ptrU8g->nextPage());
}

void LcdMatrix::printLcd(uint8_t col, uint8_t row, const char * s) {
    uint8_t startPos = col + (row * this->NB_COL);
    uint8_t i = 0;
    
    if (startPos > (((this->NB_COL) * (this->NB_ROW)) - 1))return;
    while (s[i] != '\0' && i <= this->NB_COL) {
        this->screenContent[startPos + i] = s[i++];
    }
}

void LcdMatrix::clearLcd() {
    this->ptrU8g->firstPage();
    memset( this->screenContent,' ' , this->NB_ROW * this->NB_COL );
    do {} while (this->ptrU8g->nextPage());
}
