#include "U8glib.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3);  // RX, TX
#define COUNT 20
static int WIDTH_FONT;
static int HEIGH_FONT;
static int WIDTH_SCREEN;
static int HEIGH_SCREEN;
static int NB_COL;
static int NB_ROW;
static char *ptrContent;

U8GLIB_ST7920_128X64_1X u8g(23, 17, 16);  // SPI Com: SCK = en = 23, MOSI = rw = 17, CS = di = 16  RepRap Discount Full Graphic Smart Controller - RAMPS

void matrixPrint()
{
  Serial.println("---------------");

  Serial.print("GO PRINT: ");
  Serial.println(NB_ROW);
  Serial.println(NB_COL);

  u8g.firstPage();
  do {
    for (uint8_t i = 0; i < NB_ROW; i++)
    {
      int indexRow = (i *NB_COL);
      char toPrint[NB_COL + 1];
      for (uint8_t j = 0; j < NB_COL; j++)
      {
        int offset = indexRow + j;
        toPrint[j] = *(ptrContent + offset);
      }
      toPrint[NB_COL] = '\0';
      Serial.println(toPrint);
      u8g.setPrintPos(0, HEIGH_FONT *(i + 1));
      u8g.print(toPrint);
    }
  } while (u8g.nextPage());
  //for (uint8_t i = 0; i<(NB_ROW*NB_COL); i++) Serial.println((char) *(ptrContent+i));
}

void printLcd(uint8_t col, uint8_t row, const char *s)
{
  uint8_t startPos = col + (row *NB_COL);
  uint8_t i = 0;
  if (startPos > (((NB_COL) *(NB_ROW)) - 1))
  {
    Serial.println("Err start pos");
    return;
  }
  while (*(s + i) != '\0' && i <= NB_COL)
  {
    int index = startPos + i;
    *(ptrContent + index) = *(s + i++);
  }
}

void clearLcd()
{
  u8g.firstPage();

  for (uint8_t i = 0; i < (NB_ROW *NB_COL); i++) *(ptrContent + i) = (char)" ";
  do {} while (u8g.nextPage());
}

void setup(void)
{
  u8g.setFont(u8g_font_6x13);
  Serial.begin(115200);
  while (!Serial);
  Serial.println("start");
  WIDTH_SCREEN = u8g.getWidth();
  HEIGH_SCREEN = u8g.getHeight();

  HEIGH_FONT = u8g.getFontLineSpacing();
  do {
    WIDTH_FONT = u8g.getStrWidth("A");
  } while (u8g.nextPage());

  NB_COL = WIDTH_SCREEN / WIDTH_FONT;
  NB_ROW = HEIGH_SCREEN / HEIGH_FONT;
  char screenContent[NB_ROW][NB_COL];
  ptrContent = (char*) screenContent;
  Serial.println((int)&ptrContent);
  for (uint8_t i = 0; i < (NB_ROW *NB_COL); i++) *(ptrContent + i) = (char)" ";

  Serial.println((int)&ptrContent);
  printLcd(0, 0, "azertyuiopqsdfghjklmw");
  printLcd(0, 1, "AZERTYUIOPQSDFGHJKLMW");
  printLcd(0, 2, "123456789NBVCXWMLKJHG");
  printLcd(0, 3, "nbvcxwmlkjhg987654321");
  printLcd(0, 4, "NBVCXWMLKJHG123456789");

  matrixPrint();
  Serial.println((int)&ptrContent);
  delay(2000);
  clearLcd();
  delay(2000);
  printLcd(0,2,"For that you will need dynamic allocation, which works on Arduino, but is generally not a");
  matrixPrint();

}

void loop(void)
{
  int i;
  for (i = 0; i < COUNT; i++)
  {
    delay(2000);
  }
}
