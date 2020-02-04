#ifndef _HEADERS_H
#define _HEADERS_H

/* External library */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/iom2560.h>
#include <Wire.h> // For INA219

/* Internal header */
#include "conf.h"
#include "lang.h"
#include "defines.h"
#include "structs.h"
#include "enums.h"
#include "lcd/128x64/lcdMatrix.h"
#include "lcd/20x04/LiquidCrystal.h"
#include "rotBtn/rotBtn.h"
#include "utils/utility.h"
#include "ina219/Adafruit_INA219.h"

#endif