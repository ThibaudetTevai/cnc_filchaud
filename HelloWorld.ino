/*

  HelloWorld.pde
  
  "Hello World!" example code.
  
  >>> Before compiling: Please remove comment from the constructor of the 
  >>> connected graphics display (see below).
  
  Universal 8bit Graphics Library, http://code.google.com/p/u8glib/
  
  Copyright (c) 2012, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
*/


#include "U8glib.h"
#define COUNT 20
#define WIDTH_FONT 6
#define HEIGH_FONT 13

U8GLIB_ST7920_128X64_1X u8g(23, 17, 16); // SPI Com: SCK = en = 23, MOSI = rw = 17, CS = di = 16  RepRap Discount Full Graphic Smart Controller - RAMPS

void draw(const char * txt, int i) {
 u8g.firstPage();  
      do {
        u8g.setFont(u8g_font_6x13);
        u8g.drawStr( 0+i, 13, txt);
      } while( u8g.nextPage() );
}

void matrixPrint (uint8_t col, uint8_t row, const char *s)
{
	col = (col * WIDTH_FONT) + WIDTH_FONT;
	row = (row * HEIGH_FONT) + HEIGH_FONT;

  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_6x13);
    u8g.drawStr(col, row, s);
  } while( u8g.nextPage() );
}

void setup(void) {
  
  
}

void loop(void) {
  int i;  
  for (i=0; i < COUNT; i++)
    {  

      matrixPrint(0,0,"SUCE");
      delay(1000);
      matrixPrint(0,1,"TOI");
      delay(1000);
    }
}
