#include "U8glib.h"
#include "lcdMatrix.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX
LcdMatrix lcdMatrix;

void setup(void) {
    ///u8g.setFont(u8g_font_6x13);
    Serial.begin(115200);
    while (!Serial);
    Serial.println("start");
    lcdMatrix.setup(Serial);
}

void loop(void) {
    lcdMatrix.printLcd(0, 0, "azertyuiopqsdfghjklmw");
    lcdMatrix.printLcd(0, 1, "AZERTYUIOPQSDFGHJKLMW");
    lcdMatrix.printLcd(0, 2, "123456789NBVCXWMLKJHG");
    lcdMatrix.printLcd(0, 3, "nbvcxwmlkjhg987654321");
    lcdMatrix.printLcd(0, 4, "NBVCXWMLKJHG123456789");
    lcdMatrix.printMatrix();
    delay(1000); 
    lcdMatrix.clearLcd();    
    lcdMatrix.printLcd(0, 0, "azertyuiopqsdfghjklmw");
    lcdMatrix.printLcd(0, 1, "AZERTYUIOPQSDFGHJKLMW");
    lcdMatrix.printLcd(0, 2, "For that you will ");
    lcdMatrix.printLcd(0, 3, "nbvcxwmlkjhg987654321");
    lcdMatrix.printLcd(0, 4, "NBVCXWMLKJHG123456789");
    lcdMatrix.printMatrix();
    delay(1000); 
    lcdMatrix.clearLcd();
}
