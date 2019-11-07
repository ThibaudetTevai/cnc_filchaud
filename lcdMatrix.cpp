#include "lcdMatrix.h"

LcdMatrix::LcdMatrix(){};

void LcdMatrix::setup(){
    U8G2_ST7920_128X64_F_SW_SPI u8gTemp(U8G2_R0, 23,  17, 16, U8X8_PIN_NONE);
    this->ptrU8g = &u8gTemp;
    this->ptrU8g->begin();
    this->ptrU8g->setFont(u8g_font_6x13); 

    this->lcdUpdated = false; 
    setParameter();
}

void LcdMatrix::setParameter() { 
  this->WIDTH_SCREEN = this->ptrU8g->getWidth();
  this->HEIGH_SCREEN = this->ptrU8g->getHeight();
  this->HEIGH_FONT = this->ptrU8g->getMaxCharHeight();
  do {
      this->WIDTH_FONT = this->ptrU8g->getStrWidth("A");
  } while (this->ptrU8g->nextPage());
  
  this->NB_COL = this->WIDTH_SCREEN / this->WIDTH_FONT;
  this->NB_ROW = this->HEIGH_SCREEN / this->HEIGH_FONT;
}

void LcdMatrix::printMatrix() { 
    this->ptrU8g->clearBuffer();
        for (uint8_t i = 0; i < (this->NB_ROW); i++) {
            char toPrint[(this->NB_COL) + 1];
            for (uint8_t j = 0; j < this->NB_COL; j++){
              toPrint[j] = this->screenContent[(i * this->NB_COL) + j];
            }
            toPrint[this->NB_COL] = '\0';
            this->ptrU8g->setCursor(0, this->HEIGH_FONT * (i + 1));
            this->ptrU8g->print(toPrint);
        }
    this->ptrU8g->sendBuffer();
    lcdUpdated = false;
}

void LcdMatrix::printLcd(uint8_t col, uint8_t row, const char * s) {
    uint8_t startPos = col + (row * this->NB_COL);
    uint8_t i = 0;
    
    if (startPos > (((this->NB_COL) * (this->NB_ROW)) - 1))return;
    while (s[i] != '\0' && i <= this->NB_COL) {
        this->screenContent[startPos + i] = s[i];
        i++;
    }

    lcdUpdated = true;
}

void LcdMatrix::clearLcd() {
    this->ptrU8g->firstPage();
    memset( this->screenContent,' ' , this->NB_ROW * this->NB_COL );
    do {} while (this->ptrU8g->nextPage());
}

bool LcdMatrix::isupdated() {
    return this->lcdUpdated;
}