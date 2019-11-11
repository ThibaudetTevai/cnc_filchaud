#ifndef _DEFINES_H
#define _DEFINES_H

/* DEFINE */
#if BAUDRATE == 9600
#define UBRR0_BAUDRATE_VALUE 207 // Value for USART clock register for 9600 Baud with Fosc = 16MHz (cf. Atmega2560 datasheet)
#elif BAUDRATE == 14400
#define UBRR0_BAUDRATE_VALUE 138 // Value for USART clock register for 14400 Baud with Fosc = 16MHz (cf. Atmega2560 datasheet)
#elif BAUDRATE == 19200
#define UBRR0_BAUDRATE_VALUE 103 // Value of USART clock register for 19200 Baud with Fosc = 16MHz (cf. Atmega2560 datasheet)
#elif BAUDRATE == 38400
#define UBRR0_BAUDRATE_VALUE 51 // Value of USART clock register for 38400 Baud with Fosc = 16MHz (cf. Atmega2560 datasheet)
#elif BAUDRATE == 57600
#define UBRR0_BAUDRATE_VALUE 34 // Value of USART clock register for 57600 Baud with Fosc = 16MHz (cf. Atmega2560 datasheet)
#elif BAUDRATE == 115200
#define UBRR0_BAUDRATE_VALUE 16 // Value of USART clock register for 115200 Baud with Fosc = 16MHz (cf. Atmega2560 datasheet)
#elif BAUDRATE == 250000
#define UBRR0_BAUDRATE_VALUE 7 // Value of USART clock register for 250000 Baud with Fosc = 16MHz (cf. Atmega2560 datasheet)
#else
#error "Missing or invalide Baudrate value in Conf.h" // Baud rate value configured with wrong value or missing declaration in Conf.h
#endif

#define BAUDRATE_R ((F_CPU) / (BAUDRATE * 8UL) - 1)

#define PIN_DEBUG1 13
#define PIN_DEBUG2 19
#define PIN_DEBUG3 6
#define PIN_DEBUG4 7

// stepper driver
#define DRIVER_X1_DIR_MASK 0x10 // Direction mask
#define DRIVER_X2_DIR_MASK 0x20
#define DRIVER_Y1_DIR_MASK 0x40
#define DRIVER_Y2_DIR_MASK 0x80

#define DRIVER_X1_STEP_MASK 0x01 // Step Mask
#define DRIVER_X2_STEP_MASK 0x02
#define DRIVER_Y1_STEP_MASK 0x04
#define DRIVER_Y2_STEP_MASK 0x08

#define PIN_X1_DIR A1 // PF1, X1 Direction
#define PIN_X2_DIR A7 // PF7, X2 Direction
#define PIN_Y1_DIR 28 // PA6, Y1 Direction.
#define PIN_Y2_DIR 34 // PC3, Y2 Direction

#define PIN_X1_STEP A0   // PF0, X1 Step
#define PIN_X2_STEP A6   // PF6, X2 Step
#define PIN_Y1_STEP 26   // PA4, Y1 Step
#define PIN_Y2_STEP 36   // PC1, Y2 Step
#define PIN_TEST_STEP 11 // Interrupt frequency test pin

#define PIN_X1_EN 38 // PD7, X1 Enable
#define PIN_X2_EN A2 // PF2, X2 Enable
#define PIN_Y1_EN 24 // PA2, Y1 Enable
#define PIN_Y2_EN 30 // PC7, Y2 Enable

#define MAX_FEEDRATE_TIMER_VALUE 32767 // Maximum Timer Period for low speed
#define MIN_FEEDRATE_TIMER_VALUE 25    // Minimum Timer Period for high speed
#define MAX_PAUSE_VALUE 65535          // Maximum de temps de pause
#define MIN_PAUSE_VALUE 0              // Minimum de temps de pause

#define MM_PER_SECONDE 62500.0 * MM_PER_STEP

// LCD
#define PIN_LCD_RS 16                 // LCD control and is connected into GADGETS3D  shield LCDRS
#define PIN_LCD_E 17                  // LCD enable pin and is connected into GADGETS3D shield LCDE
#define PIN_LCD_D4 23                 // LCD signal pin, connected to Gadgets3D shield LCD4
#define PIN_LCD_D5 25                 // LCD signal pin, connected to Gadgets3D shield LCD5
#define PIN_LCD_D6 27                 // LCD signal pin, connected to Gadgets3D shield LCD6
#define PIN_LCD_D7 29                 // LCD signal pin, connected to Gadgets3D shield LCD7
#define LCD_COLUMN_COUNT 20           // 20 character per line
#define LCD_LINE_COUNT 4              // 4 lines
#define LCD_NEW_DIGIT_DISPLAY_COUNT 6 // Number of timer 2 interrupt to generate a new LCD display

// PWM
#define PIN_WIRE_PWM 8   //Sortie de cde de chauffage di fil
#define PIN_CUTTER_PWM 9 //Sortie de cde de chauffage di fil

//PIN
#define PIN_SWITCH_CONTROL_MODE 40    //Interrupteur de selection PC ou Manu "0" PC
#define PIN_SWITCH_MOTOR A9           //Interrupteur des moteur PAP "0" ON
#define PIN_SWITCH_HEATING_PC 44      //Interrupteur de Chauffage fil "0" PC
#define PIN_SWITCH_HEATING_MANU A10   //Interrupteur de Chauffage Manuel fil "0" M
#define PIN_SWITCH_HEATING_CUTTER A12 //Interrupteur de Chauffage Dutter "0" ON
//Al>
//#define PIN_PUSHBUTTON_SHUNT_ENDSTOP 42 // Bouton poussoir de shunt fin de course pour redémarrer
//<
#define PIN_RELAY_HEATING 10 //Interrupteur de Chauffage fil "0" ON
//Al>
//#define PIN_ENDSTOP_MINI 14 //Somme des 4 fdc mini en série "0" fdc non sollicité
//<
#define PIN_POT_WIRE A5    //potentiomètre chauffage fil
#define PIN_POT_CUTTER A11 //potentiomètre chauffage cutter electrique

#define PIN_ROTARY_ENCODER_A 31          // Rotary Encoder Channel A
#define PIN_ROTARY_ENCODER_B 33          // Rotary Encoder Channel B
#define PIN_ROTARY_ENCODER_PUSHBUTTON 35 // Rotary Encoder Channel Push Button
#define PIN_BUZZER 37                    // Beeper is Connected into GADGETS3D shield MEGA_18BEEPER

#define PIN_X1_LIMIT 3  // "0" fdc X1 non sollicité ; "1" avec inversion fdc
#define PIN_Y1_LIMIT 2  // "0" fdc Y1 non sollicité ; "1" avec inversion fdc
#define PIN_X2_LIMIT 15 // "0" fdc X2 non sollicité ; "1" avec inversion fdc
#define PIN_Y2_LIMIT 14 // "0" fdc Y2 non sollicité ; "1" avec inversion fdc

#define PIN_BP_HOMING 42 // Bouton poussoir de homing

// Command Buffer
#define CMD_DATA_SIZE 4     // Size of the Command data in byte (= 4 byte) !! Shall be even number (2, 4, 6, etc..) !!
#define CMD_BUFFER_SIZE 260 // Number of Command the Buffer can stock
#define CMD_BUFFER_MAX 255

//Communication Buffer
#define COM_BUFFER_SIZE 260
#define COM_BUFFER_OVERFLOW_TRIGGER 175 //
#define COM_BUFFER_UNDERFLOW_TRIGGER 75 //

#define TWO_BYTE_CMD 2
#define THREE_BYTE_CMD 3

// Switch

#define SWITCH_STATUS_CONTROL_MODE 0
#define SWITCH_STATUS_MOTOR_ENABLE 1
#define SWITCH_STATUS_HEAT_PC 2
#define SWITCH_STATUS_HEAT_MANU 3
#define SWITCH_STATUS_CUTTER_ENABLE 4
//#define SWITCH_STATUS_ENDSTOP_SHUNT 5
#define SWITCH_STATUS_HOMING_OK 5
#define SWITCH_STATUS_ENDSTOP 6

// Mode

#define IsSwitchNotInitialized(x) ((x & 0x1F) != 0x1F)

#define MODE_SWITCH_BIT_CONTROL_MODE 0
#define MODE_SWITCH_BIT_MOTOR_MODE 1
#define MODE_SWITCH_BIT_HEAT_PC 2
#define MODE_SWITCH_BIT_HEAT_MANU 3
#define MODE_SWITCH_BIT_CUTTER 4
//#define MODE_SWITCH_BIT_ENDSTOP_SHUNT 5
#define MODE_SWITCH_BIT_HOMING_OK 5
#define MODE_SWITCH_BIT_ENDSTOP 6

#define MODE_SWITCH_PC(x) ((x & 0x01) == 0x00)
#define MODE_INIT(x) ((x & 0x1F) != 0x1F)
#define HEAT_MODE_MANU(x) ((x & 0x08) == 0x00)
#define HEAT_MODE_PC(x) ((x & 0x04) == 0x00)
#define HEAT_MODE_CUTTER(x) ((x & 0x10) == 0x00)
#define ENDSTOP_ACTIVE(x) ((x & 0x60) == 0x60)
#define MOTOR_DEACTIVATED(x) (x & 0x02)

// Utility
#define ON true
#define OFF false
#define HIGH_BYTE 1
#define LOW_BYTE 0

#define ENABLE_RX_ISR() UCSR0B |= 0x90
#define ENABLE_T1_ISR() TIMSK1 = 0x02
#define ENABLE_T5_ISR() TIMSK5 = 0x02
#define ENABLE_T2_ISR() TIMSK2 = 0x02

#define DISABLE_RX_ISR() UCSR0B &= 0x6F
#define DISABLE_T1_ISR() TIMSK1 = 0x00
#define DISABLE_T5_ISR() TIMSK5 = 0x00
#define DISABLE_T2_ISR() TIMSK2 = 0x00

#define TX_WRITE(x) UDR0 = x
#define RX_READ(x) x = UDR0

#define ENABLE_T2_COMP_OUTPUT_B() TCCR2A |= 0x20
#define DISABLE_T2_COMP_OUTPUT_B() TCCR2A &= 0xCF
//#define ENABLE_T4_COMP_OUTPUT_C() TCCR4A |= 0x08
//#define DISABLE_T4_COMP_OUTPUT_C() TCCR4A &= 0xF3

const byte heatPercentToPWMConvTab[101] = 
{
    0, 3, 6, 8, 11, 13, 16, 18, 21, 23, 26, 29, 31, 34, 36, 39, 41,
    44, 46, 49, 51, 54, 57, 59, 62, 64, 67, 69, 72, 74, 77, 80, 82,
    85, 87, 90, 92, 95, 97, 100, 102, 105, 108, 110, 113, 115, 118,
    120, 123, 125, 128, 131, 133, 136, 138, 141, 143, 146, 148, 151,
    153, 156, 159, 161, 164, 166, 169, 171, 174, 176, 179, 182, 184,
    187, 189, 192, 194, 197, 199, 202, 204, 207, 210, 212, 215, 217,
    220, 222, 225, 227, 230, 233, 235, 238, 240, 243, 245, 248, 250,
    253, 255
};

const bool StepperDriverEnableTab[2][2] = 
{
    {ON, ON},
    {OFF, ON}
};

const bool StepperDriverEnableState[2] = 
{
    STEPPER_DRIVER_ENABLE_LOW_LEVEL,
    STEPPER_DRIVER_ENABLE_HIGH_LEVEL
};

const byte SwitchStatusTab[7][2] =
{
    {0, 1},
    {0, 2},
    {0, 4},
    {0, 8},
    {0, 16},
    {0, 32},
    {0, 128}
};

#endif