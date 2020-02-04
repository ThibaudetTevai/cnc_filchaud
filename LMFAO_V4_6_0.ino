/*  Copyright 2013 Martin
    Complété en 2016 par A. DENIS
    Complété en 2017 par Vincent
    Complete en 2018 par Olivier
    Complete en 2019 par Alain
    This file is part of jedicutplugin.

    fcifmdlcnc is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    fcifmdlcnc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

  This version of the code is for the letmathe mdlcnc board. For
  other boards this code must be adapted accordingly !!!
*/
/*
  The jedicut communication configuration for Clock and Direction must be fixed
  set to the below values,
  only the wires to the specific board may be different -- wire to -->

  Le 18/09/2016 commencé assai avec carte arduino mega 2560 modifier le pins,
  pris le port C au lieu du port D
  Le 20/09/2016 modification du code pour avoir les bonnes bornes de sorties
  pour Ramps1.4
  Le 21/09/2016 Mise en place de L'afficheur LCD
  Le 28/10/2016 changement de TIMER 1 vers 5
  Le 29/10 2016 Ajout choix baudrate dans conf.h
  Le 2/01/2017 Modification Calcul de frequence
  Le 3/01/2017 Modification du mode de timer mis en CTC mode 4
  Le 4/01/2017 Mise en place affichage configuration
  le 30/11/2017 Adaptation du sketch suit à nouveau Plugin USBSerial_2.dll compatible
  avec les entraînements à courroie passer 16 bits la commande de vitesse pour
  élargissement de la plage de vitesse dans la config de Jedicut.
  Le 11/02/2019 Separation des fins de course et mise en place de l'écran 
  d'aide à la mise en service.
  le 12/02/2019 Mise en place du homing
  Le 13/02/2019 Recherche Bug sur fdc X2 un debug utilisait l'entrée fdc X2
  Le 14/02/2019 Mise en place chauffe Dynamique.
  Version au 11/04/2019 LMFO_V4_6_0 ajout traitement des pauses
  
  Function               Number in jedicut configuration   Arduino Pin
                       (fixed !!)
  EngineX1 Clock         2                                 53  (PB0)
  EngineX2 Clock         3                                 52  (PB1)
  EngineY1 Clock         4                                 51  (PB2)
  EngineY2 Clock         5                                 50  (PB3)
  EngineX1 Direction     6                                 33  (PC4)
  EngineX2 Direction     7                                 32  (PC5)
  EngineY1 Direction     8                                 31  (PC6)
  EngineY2 Direction     9                                 30  (PC7)

  All Engines On/Off     -                                 10  (PB4)
  Heating On/off         -                                 35  (PC2)
  Heating PWM            -                                  7  (PH4)
*/
#include "headers.h"

// Mot de la lecture des fins de course 0b 0,0,0,0,fdcY2,fdcY1,fdcX2,fdcX1
byte mask_limits;  // Résultat de la lecture des fins de courses
byte Val_X1_Limit; // "0" fdc X1 non sollicité
byte Val_Y1_Limit; // "0" fdc Y1 non sollicité
byte Val_X2_Limit; // "0" fdc X2 non sollicité
byte Val_Y2_Limit; // "0" fdc Y2 non sollicité

#ifdef MATRIX_LCD    
    LcdMatrix lcdMatrix; // MATRIX declaration
#else
    //LCD declaration
    LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
#endif

#ifdef INA219
    //INA 219 declaration for the prob current
    Adafruit_INA219 ina219;
#endif

#ifdef HEAT_CONSIGN_ROTARY_ENCODER
    // Instance object rotary button encoder
    RotBtn rotBtn(PIN_ROTARY_ENCODER_A, PIN_ROTARY_ENCODER_B, PIN_ROTARY_ENCODER_PUSHBUTTON);
#endif

volatile bool toogle = true;
bool StepperDriverEnableMode = STEPPER_DRIVER_ENABLE_HIGH_LEVEL;

volatile TBuffer CommandBuffer[CMD_BUFFER_SIZE];
volatile byte CommandIndexRead = 0;
volatile byte CommandIndexWrite = 0;

volatile byte CommandCounter = 0;

byte ComOverflow = 0;

volatile byte ParserState = PARSER_STATE_CMD;

#ifdef MACHINE_NAME
    const char MachineName[sizeof(STRINGIFY(MACHINE_NAME))] = MACHINE_NAME;
#else
    const char *MachineName = TITLE_JEDICUT;
#endif

unsigned char MachineNameChar[LCD_COLUMN_COUNT];

volatile bool buzzerToogle = false;

static byte modeState;

// Pour Homing
byte Mot_Dir = 0; // Mot de commande des direction des moteurs
byte Mot_Ck = 0;  // Mot de commande des steps des moteurs

// Mot de commande des steps des moteurs 0b dY2,dY1,dX2,dX1,sY2,sY1,sX2,sX1
byte h_motCkDir = 0; // pour le homing

//Pour chauffe dynamique du fil
//int refChaufDyn ; // consigne avec correction par les steps
bool WireVitesse = 0; //ordre de correction de vitesse

// Pour Pause
bool ActivePause = 0;
unsigned long kPause = 0;  // millis  + pause en début de pause
unsigned long RePause = 0; // calcul de la pause restante
unsigned long Cad_Aff = 0; //cadense affichage pause

// Variable init timer4
int maRaz;
int maPreset;

/**********************************************************************************/
void setup(void)
{
    #ifdef USART_ARDUINO
        Serial.begin(115200);
    #else
        //serial com Initialization
        UCSR0A |= (1 << U2X0);                                 // double transmission speed
        UCSR0B |= (1 << RXCIE0) | (1 << TXEN0) | (1 << RXEN0); //enable transmitter, receiver, enable receive complete interrupt
        UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);                /* 8 data bits, 1 stop bit */
        UBRR0H = (BAUDRATE_R >> 8);                            // shift the register right by 8 bits to get the upper 8 bits
        UBRR0L = BAUDRATE_R;                                   // Configure USART Clock register
        SREG |= 0x80;                                          //set global interrupt enable bit
    #endif

    // pins Initialization

    //LCD
    #ifdef MATRIX_LCD
        lcdMatrix.setup();
    #endif

    // INA219
    #ifdef INA219
        ina219.begin(); // Init I2C and calibration 32V 2A
    #endif


    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    // Heating
    pinMode(PIN_RELAY_HEATING, OUTPUT);   // PD2, Heat Relay on/off low active for letmathe mdlcnc
    digitalWrite(PIN_RELAY_HEATING, LOW); // Off

    pinMode(PIN_WIRE_PWM, OUTPUT);   // PWM pin for wire heating
    pinMode(PIN_CUTTER_PWM, OUTPUT); //$PWM pin for the cutter

    pinMode(PIN_POT_WIRE, INPUT);   // Analog input for wire heat consign potentiometer
    pinMode(PIN_POT_CUTTER, INPUT); // Analog input for Cutter hea[ consign potentiometer

    // Switch
    pinMode(PIN_SWITCH_CONTROL_MODE, INPUT);     //Switch control mode PC/Manu ->  low state = mode PC / high state = mode Manu
    digitalWrite(PIN_SWITCH_CONTROL_MODE, HIGH); // Atmega internal Pullup Activated

    pinMode(PIN_SWITCH_MOTOR, INPUT);     //Switch Stepper Driver -> low state = Enabled / high state = OFF or PC control
    digitalWrite(PIN_SWITCH_MOTOR, HIGH); // Atmega internal Pullup Activated

    pinMode(PIN_SWITCH_HEATING_PC, INPUT);     //Switch Wire heat mode PC -> Low state = control by PC / high state = OFF or Manu
    digitalWrite(PIN_SWITCH_HEATING_PC, HIGH); // Atmega internal Pullup Activated

    pinMode(PIN_SWITCH_HEATING_MANU, INPUT);     //Swipch Wire heat mode manu -> low state = manu / high state = OFF or PC
    digitalWrite(PIN_SWITCH_HEATING_MANU, HIGH); // Atmega internal Pullup Activated

    pinMode(PIN_SWITCH_HEATING_CUTTER, INPUT);     //Switch Cutter heat mode -> disable if switch control mode is on PC mode else low state = ON / high state = OFF
    digitalWrite(PIN_SWITCH_HEATING_CUTTER, HIGH); // Atmega internal Pullup Activated

    pinMode(PIN_X1_LIMIT, INPUT);     //Pin sfcd X1 "0" non sollicité
    digitalWrite(PIN_X1_LIMIT, HIGH); // Mise en servive Pullup

    pinMode(PIN_Y1_LIMIT, INPUT);     //Pin sfcd Y1 "0" non sollicité
    digitalWrite(PIN_Y1_LIMIT, HIGH); // Mise en servive Pullup

    pinMode(PIN_X2_LIMIT, INPUT);     //Pin sfcd X2 "0" non sollicité
    digitalWrite(PIN_X2_LIMIT, HIGH); // Mise en servive Pullup

    pinMode(PIN_Y2_LIMIT, INPUT);     //Pin sfcd Y2 "0" non sollicité
    digitalWrite(PIN_Y2_LIMIT, HIGH); // Mise en servive Pullup
    //Al>
    pinMode(PIN_BP_HOMING, INPUT);     //Pin BP Homing
    digitalWrite(PIN_BP_HOMING, HIGH); // Mise en servive Pullup

    //  pinMode (PIN_ENDSTOP_MINI, INPUT); // state of the 4 endstops -> low state = All Endstops not activate / high state = at least one Endstop is activate
    //  digitalWrite(PIN_ENDSTOP_MINI, HIGH); // Atmega internal Pullup Activated
    //  pinMode (PIN_PUSHBUTTON_SHUNT_ENDSTOP, INPUT); // push button shunt endstops -> low state = depressed = Endstops shunted / high state = Endstops not shunted
    //  digitalWrite(PIN_PUSHBUTTON_SHUNT_ENDSTOP, HIGH); // Atmega internal Pullup Activated
    //<
    pinMode(PIN_DEBUG1, OUTPUT); // Led Pin used to indicate an underflow condition
    pinMode(PIN_DEBUG2, OUTPUT); // Led Pin used to indicate an underflow condition
    pinMode(PIN_DEBUG3, OUTPUT); // Led Pin used to indicate an underflow condition
    pinMode(PIN_DEBUG4, OUTPUT); // Led Pin used to indicate an underflow condition

    // Driver Motor Pins

    pinMode(PIN_X1_EN, OUTPUT);
    digitalWrite(PIN_X1_EN, STEPPER_DRIVER_ENABLE_LOW_LEVEL); // Disable Driver X1
    pinMode(PIN_X2_EN, OUTPUT);
    digitalWrite(PIN_X2_EN, STEPPER_DRIVER_ENABLE_LOW_LEVEL); // Disable Driver X2
    pinMode(PIN_Y1_EN, OUTPUT);
    digitalWrite(PIN_Y1_EN, STEPPER_DRIVER_ENABLE_LOW_LEVEL); // Disable Driver Y1
    pinMode(PIN_Y2_EN, OUTPUT);
    digitalWrite(PIN_Y2_EN, STEPPER_DRIVER_ENABLE_LOW_LEVEL); // Disable Driver Y2

    pinMode(PIN_X1_STEP, OUTPUT);
    pinMode(PIN_X2_STEP, OUTPUT);
    pinMode(PIN_Y1_STEP, OUTPUT);
    pinMode(PIN_Y2_STEP, OUTPUT);
    pinMode(PIN_TEST_STEP, OUTPUT);
    digitalWrite(PIN_TEST_STEP, LOW);

    pinMode(PIN_X1_DIR, OUTPUT);
    pinMode(PIN_X2_DIR, OUTPUT);
    pinMode(PIN_Y1_DIR, OUTPUT);
    pinMode(PIN_Y2_DIR, OUTPUT);

    // Timer 1 init, used to parse the communication buffer and fill the command buffer
    TIMSK1 = 0x00; // Reset interrupt flag
    TIFR1 = 0x00;  // Interrupt disabled, enabled only when there is command to execute
    TCCR1A = 0x00; // configure timer in Clear Timer on Compare mode, disable Output Compare pins
    TCCR1B = 0x0C; // configure timer in Clear Timer on Compare mode and set prescaler to 256
    TCCR1C = 0x00;
    OCR1A = 1; // Set Top value => Fosc = 16MHz (62.5 ns), Prescaler = 256, Compare value = 1 -> Fpwm = 1 / ((1 + 1) * 256 * 62.5e-9) ~=  KHz

    // Timer 2 init, used for Cutter Heat PWM as hardware PWM generator and as Buzzer sound generator on timer counter overflow
    TIMSK2 = 0x00; // Reset interrupt flag
    TIFR2 = 0x00;  // disable interrupt
    TCCR2A = 0x03; // configure timer in Fast PWM mode, disable Output Compare pin A and enable Output Compare pin B
    TCCR2B = 0x0E; // configure timer in Fast PWM mode and set prescaler to 256
    OCR2A = 100;   // Set Top value => Fosc = 16MHz, Prescaler = 256, Compare value = 100 -> Fpwm = 1 / (100 * 256 * 62.5e-9) = 625 Hz
    OCR2B = 0;     // Set compare value => Compare value between 0 = 0% PWM and 200 = 100% PWM

    /*
    // Timer 4 init, used for Wire Heat PWM as Hardware PWM generator
    TIMSK4 = 0x00;  // Reset interrupt flag
    TIFR4 = 0x00; // disable interrupt
    TCCR4A = 0x0A;  // configure timer in Fast PWM mode, disable Output Compare pins A and B and enable Output Compare pin C
    TCCR4B = 0x1C;  // conbigure timer in Fast PWM mode and set prescaler to 256
    TCCR4C = 0x00;  // Disable Input capture
    ICR4 = 100;   // Set Top value => Fosc = 16MHz, Prescaler = 256, Compare value = 200 -> Fpwm = 1 / (100 * 256 * 62.5e-9) = 10 KHz
    OCR4C = 0;    // Set compare value => Compare value between 0 = 0% PWM and 200 = 100% PWM
    */

    // modification frequence PWM des pin 6,)7, 8, par le timer 4  (chauffe)
    maRaz = 7;    // 111 pour CS02, CS01, CS00
    maPreset = 2; //010 pour 7800 Hz ; 001 pour 62000Hz
    TCCR4B &= ~maRaz;
    TCCR4B |= maPreset;

    // Timer 5 init, used to generate steps at a constant rate
    TIMSK5 = 0x00; // Reset interrupt flag
    TIFR5 = 0x00;  // Interrupt disabled, enabled only when there is command to execute
    TCCR5A = 0x00; // configure timer in Clear Timer on Compare mode, disable Output Compare pins
    TCCR5B = 0x0C; // configure timer in Clear Timer on Compare mode and set prescaler to 256
    TCCR5C = 0x00;
    OCR5A = 255; // Set Top value => Fosc = 16MHz (62.5 ns), Prescaler = 256, Compare value = 255 -> Fpwm = 1 / (255 * 256 * 62.5e-9) =  245 Hz

    // MachineName.getBytes(MachineNameChar, LCD_COLUMN_COUNT);
    // Switch.HomingOk = false ;  // Etat du Homing
    #ifdef DEBUG
        digitalWrite(13, HIGH);
    #endif
    //Al>
    AideMiseServiceFdc(); // aide à la mise en service des fins de course
    //<
    printSerialLn("End Setup");
}

//==============================================================================
//  Test  des fins de course
//==============================================================================
void Test_fdc()
{
    do
    {
        limits_Lect();
        Aff_Test_Fdc();

        Heat.WireConsign = map(analogRead(PIN_POT_WIRE), 0, 1023, 0, MAX_PERCENTAGE_WIRE);
    } while (Heat.WireConsign > 10);
    limits_Lect();
}
//-----------------------------------------------------------------------------

/**********************************************************************************/
//==============================================================================
// Aide à la mise en service des fins de course
//==============================================================================
void AideMiseServiceFdc(void)
{
    Heat.WireConsign = map(analogRead(PIN_POT_WIRE), 0, 1023, 0, MAX_PERCENTAGE_WIRE);
    if (Heat.WireConsign > 50)
    {
        Affic_Trame_fdc();
        Test_fdc();
    }
}
//------------------------------------------------------------------------------

//==============================================================================
// Affichage de la Trame Etat Fins de course
//==============================================================================
void Affic_Trame_fdc()
{
#ifndef MATRIX_LCD
    lcd.begin(LCD_COLUMN_COUNT, LCD_LINE_COUNT);
#endif
    clearLCD();
    printLCD(0, 0, TEXT17); // " Etat fin de course"
    printLCD(0, 1, TITLE_FDC);
    printLCD(0, 3, TEXT18); // "Fin -> Pot Ch < 10%"
}
//------------------------------------------------------------------------------

//==============================================================================
// Lecture des fins de course et constitution d'un mot état des fins de course
//==============================================================================

void limits_Lect()
{
    mask_limits = (((digitalRead(PIN_X1_LIMIT) ^ INV_FDC_X1) << 0) | ((digitalRead(PIN_Y1_LIMIT) ^ INV_FDC_Y1) << 2) | ((digitalRead(PIN_X2_LIMIT) ^ INV_FDC_X2) << 1) | ((digitalRead(PIN_Y2_LIMIT) ^ INV_FDC_Y2) << 3));

    Val_X1_Limit = bitRead(mask_limits, 0);
    Val_X2_Limit = bitRead(mask_limits, 1);
    Val_Y1_Limit = bitRead(mask_limits, 2);
    Val_Y2_Limit = bitRead(mask_limits, 3);
}
//------------------------------------------------------------------------------

//==============================================================================
// Affichage de l'etat des fins de course
//==============================================================================
void Aff_Test_Fdc()
{
    char val[5];
    sprintf(val, "%d", Val_X1_Limit);
    printLCD(2, 2, val);
    sprintf(val, "%d", Val_Y1_Limit);
    printLCD(7, 2, val);
    sprintf(val, "%d", Val_X2_Limit);
    printLCD(12, 2, val);
    sprintf(val, "%d", Val_Y2_Limit);
    printLCD_I(17, 2, val);
    delay(100); // Attendre 100ms
}
//------------------------------------------------------------------------------

//==============================================================================
// Homing Manage
//==============================================================================
void HomingManage()
{
    if (Switch.HomingOk == false)
    {
        if ((digitalRead(PIN_SWITCH_MOTOR) == LOW))
        {
            StepperDriverEnable(ON);
            Arm_Homing();
            printLCD(0, 1, TEXT14);
            GetSwitchStatus();
            HMI_ModeScreen();
        }
    }
    else
    {
        if (Switch.MotorEnable == 1)
            Desar_Homing();
    }
}
//------------------------------------------------------------------------------
//==============================================================================
// Armement Homing
//==============================================================================

void Arm_Homing()
{
    if (SEQ_HOMING == 1)
    {
        printLCD(0, 1, TEXT19);
        printLCD(0, 1, TEXT19);
        printLCD(0, 2, TITLE_ENDSTOP);
        printLCD_I(0, 3, CLEAN20);

        while ((digitalRead(PIN_BP_HOMING) == HIGH))
            ;
        homing_set();
        if (PREPOS == 1)
        {
            prepos_set();
        }

        printLCD(0, 1, TEXT20);
        delay(1000);

        if (MOTEUR_ON_ASSERVI == 1)
        {
            StepperDriverEnable(ON);
        }
        Switch.HomingOk = true;
    }

    // Si on ne veut pas de Homing par la config, on vient la
    // pour faire le premier passage
    else
    {
        Switch.HomingOk = true;
    }
}
//------------------------------------------------------------------------------

//==============================================================================
// Desarmement Homing
//==============================================================================
void Desar_Homing()
{
    Switch.HomingOk = false;
}

//------------------------------------------------------------------------------

//==============================================================================
// Séquence Homing
//==============================================================================

void homing_set()
{
    printLCD(0, 1, TEXT13);

    cycle_1();
    cycle_2();
    cycle_3();
    cycle_4();
}
//------------------------------------------------------------------------------

//==============================================================================
// Cycle 1 Remontée des axe Y de Xmm si vous avez choisi l'option
// Retour à zéro X1 et X2 en premier, Y1 et Y2 en second
//==============================================================================
void cycle_1()
{
    byte tr_mask_limits = 0;
    //calcul de la tempo entre chaque step pour la vitesse d'approche
    unsigned int tempo_step = MM_PER_STEP / VIT_RECH_FDC * 1000000; //555,555
    if (POS_SECU_Y == 1)
    {
        unsigned int Nbre_pas = MM_POS_SECU_Y / MM_PER_STEP;

        printLCD_I(0, 3, "===");

        for (unsigned int i = Nbre_pas; i > 0; i--)
        {
            h_motCkDir = 0x0C; // 00001100
            ProcessStep(h_motCkDir);
            delayMicroseconds(tempo_step);
        }
    }

    // Recherche fdc X1 et X2
    printLCD_I(3, 3, "==");

    do
    {
        limits_Lect();
        tr_mask_limits = mask_limits ^ 0xFF;
        h_motCkDir = 0xF3; // 11110011
        h_motCkDir = h_motCkDir & tr_mask_limits;
        ProcessStep(h_motCkDir);
        delayMicroseconds(tempo_step);
    } while (mask_limits < 0x03);

    // Recherche fdc Y1 et Y2
    printLCD_I(5, 3, "==");
    do
    {
        limits_Lect();
        tr_mask_limits = mask_limits ^ 0xFF;
        h_motCkDir = 0xFC; //11111100
        h_motCkDir = h_motCkDir & tr_mask_limits;
        ProcessStep(h_motCkDir);
        delayMicroseconds(tempo_step);
    } while (mask_limits < 0x0F);
}
//------------------------------------------------------------------------------

//==============================================================================
//Cycle 2 retour hors fin de course les 4 axes en même temps
//==============================================================================
void cycle_2()
{

    printLCD_I(7, 3, "==");

    byte tr_mask_limits = 0;

    //calcul de la tempo entre chaque step pour la vitesse lente (ajuste)
    unsigned int tempo_step = MM_PER_STEP / VIT_AJUST_FDC * 1000000;

    // Avance des 4 axes jusqu'à ce que les fins de course soient sollicités
    do
    {
        limits_Lect();
        tr_mask_limits = mask_limits ^ 0x00;
        h_motCkDir = 0x0F; //00001111
        h_motCkDir = h_motCkDir & tr_mask_limits;
        ProcessStep(h_motCkDir);
        delayMicroseconds(tempo_step);
    } while (mask_limits > 0x00); // Sortie de la boucle les 4 fdc non sollicités.
}
//------------------------------------------------------------------------------
//==============================================================================
// Cycle 3 : retour vers fins de course les 4 axes en même temps
//==============================================================================

void cycle_3()
{

    printLCD_I(9, 3, "===");

    byte tr_mask_limits = 0;

    //calcul de la tempo entre chaque step pour la vitesse lente (ajuste)
    unsigned int tempo_step = MM_PER_STEP / VIT_AJUST_FDC * 1000000;
    do
    {
        limits_Lect();
        tr_mask_limits = mask_limits ^ 0xFF;
        h_motCkDir = 0xFF; //11111111
        h_motCkDir = h_motCkDir & tr_mask_limits;
        ProcessStep(h_motCkDir);
        delayMicroseconds(tempo_step);
    } while (mask_limits < 0x0F); // Sortie de la boucle les 4 fdc sont sollicités.
}
//------------------------------------------------------------------------------

//==============================================================================
// Cycle 4 : retour hors fin de course les 4 axes en même temps
//==============================================================================
void cycle_4()
{

    printLCD_I(12, 3, "====");

    byte tr_mask_limits = 0;

    //calcul de la tempo entre chaque step pour la vitesse lente (ajuste)
    unsigned int tempo_step = MM_PER_STEP / VIT_AJUST_FDC * 1000000;

    do
    {
        limits_Lect();
        tr_mask_limits = mask_limits ^ 0x00;
        h_motCkDir = 0x0F; // 00001111
        h_motCkDir = h_motCkDir & tr_mask_limits;
        ProcessStep(h_motCkDir);
        delayMicroseconds(tempo_step);
    } while (mask_limits > 0x00); // Sortie de la boucle les 4 fdc non sollicités.
}
//------------------------------------------------------------------------------

//==============================================================================
// Si option "Prépositionnement" Les axes Y rejoignent les mm demandés en option
// puis  les axes X rejoignent les mm demandés
//==============================================================================
void prepos_set()
{
    printLCD_I(16, 3, "====");

    //calcul de la tempo pour la vitesse de déplacement (Grande Vitesse).
    unsigned int tempo_step = MM_PER_STEP / VIT_RECH_FDC * 1000000;

    // Prepositionnement des axes Y
    unsigned int Nbre_pas = MM_PREPOS_Y / MM_PER_STEP;
    for (unsigned int i = Nbre_pas; i > 0; i--)
    {
        h_motCkDir = 0x0C; // 00001100
        ProcessStep(h_motCkDir);
        delayMicroseconds(tempo_step);
    }
    // Prépositionnement des axes X
    Nbre_pas = MM_PREPOS_X / MM_PER_STEP;
    for (unsigned int i = Nbre_pas; i > 0; i--)
    {
        h_motCkDir = 0x03; // 00000011
        ProcessStep(h_motCkDir);
        delayMicroseconds(tempo_step);
    }
}
//------------------------------------------------------------------------------

//=====================================9========================================
// End Stop Manage  sur une detection de End Stop en Mode Auto
//==============================================================================
void EndStopManage()
{
    if (Switch.HomingOk == true)
    {
        //arretPropreMachine();

        limits_Lect();
        if (mask_limits > 0x00)
        {
            Trait_Arr_fdc();
            Desar_Homing();
        }
    }
}

//==============================================================================
// Traitement d'un arrêt par fin de course
//==============================================================================
void Trait_Arr_fdc()
{
    clearLCD();
    printLCD(0, 0, TEXT11);
    printLCD(0, 1, TITLE_FDC);
    printLCD(0, 3, TEXT12);
    Aff_Test_Fdc(); // Inside this methode we print the matrix
    while (digitalRead(PIN_BP_HOMING) == HIGH)
        ; //alarmSonor();
    SoundAlarm(OFF);
    while (digitalRead(PIN_BP_HOMING) == LOW)
        ;

    clearLCD();
    testPosIntDem();
    clearLCD();
    printLCD(0, 0, MachineName);
    printLCD(0, 1, TEXT14);
    printLCD(0, 3, TITLE_VAR);
}

//------------------------------------------------------------------------------

//==============================================================================
// Au demarrage test de la position des interrupteurs ou sur arret fdc
//==============================================================================

void testPosIntDem()
{
    printLCD(0, 2, TEXT6);
    if (digitalRead(PIN_SWITCH_CONTROL_MODE) == LOW) // test si l'inter Mode est sur Manu
    {
        printLCD_I(0, 3, TEXT7);
        while (digitalRead(PIN_SWITCH_CONTROL_MODE) == LOW)
            ;
    }

    if (digitalRead(PIN_SWITCH_MOTOR) == LOW) // test si l'inter Moteur est sur OFF
    {
        printLCD_I(0, 3, TEXT8); // "Mettre Mot. sur OFF" attente inter Moteur sur OFF
        while (digitalRead(PIN_SWITCH_MOTOR) == LOW)
            ;
    }
    if ((digitalRead(PIN_SWITCH_HEATING_PC == LOW) | (digitalRead(PIN_SWITCH_HEATING_MANU) == LOW))) // test si l'inter Chauffe est sur OFF
    {
        printLCD_I(0, 3, TEXT9);
        while ((digitalRead(PIN_SWITCH_HEATING_PC) == LOW) | (digitalRead(PIN_SWITCH_HEATING_MANU) == LOW))
            ;
    }

    if (digitalRead(PIN_SWITCH_HEATING_CUTTER) == LOW) // test si l'inter Moteur est sur OFF
    {
        printLCD_I(0, 3, TEXT10); // "Mettre Cut sur OFF" attente inter Moteur sur OFF
        while (digitalRead(PIN_SWITCH_HEATING_CUTTER) == LOW)
            ;
    }
}
//------------------------------------------------------------------------------

//==============================================================================
// Pause Manage  sur une detection de End Stop en Mode Auto
//==============================================================================
inline void PauseManage(void)
{
    if (ActivePause == 1)
    {
        if (millis() > Cad_Aff)
        {
            RePause = (kPause - millis()) / 1000;
            Cad_Aff = millis() + 250;

            printLCD(7, 0, CLEAN3);
            printLCD(7, 0, ftostr62rj(RePause));
        }

        if (millis() > kPause)
        {
            ActivePause = 0;
            kPause = 0;
            printLCD(0, 0, CLEAN19);
            printLCD(0, 0, MachineName);
            ENABLE_T5_ISR();
        }
    }
}

//------------------------------------------------------------------------------

inline void StepperDriverDir(byte dir)
{
    //AL >
    dir = (dir ^ INV_DIR_MASK);
    //<
    bitWrite(PORTF, 1, (dir & DRIVER_X1_DIR_MASK));
    bitWrite(PORTF, 7, (dir & DRIVER_X2_DIR_MASK));
    bitWrite(PORTA, 6, (dir & DRIVER_Y1_DIR_MASK));
    bitWrite(PORTC, 3, (dir & DRIVER_Y2_DIR_MASK));
}

/**********************************************************************************/

inline void StepperDriverStep(byte step)
{
    bitWrite(PORTF, 0, (step & DRIVER_X1_STEP_MASK));
    bitWrite(PORTF, 6, (step & DRIVER_X2_STEP_MASK));
    bitWrite(PORTA, 4, (step & DRIVER_Y1_STEP_MASK));
    bitWrite(PORTC, 1, (step & DRIVER_Y2_STEP_MASK));
    bitWrite(PORTB, 5, (step & DRIVER_X1_STEP_MASK));
}

/**********************************************************************************/
inline void ProcessStep(byte cmd)
{
    StepperDriverDir(cmd);
    //  delayMicroseconds(1);     // wait 1 us before sending the steps
    StepperDriverStep(cmd);

    //Calcul compensation Chauffe artifice vitesse
    if (CHAUFFE_ASSERV == 1)
    {
        WireVitesse = (((cmd & DRIVER_X1_STEP_MASK) && (cmd & DRIVER_Y1_STEP_MASK)) || ((cmd & DRIVER_X2_STEP_MASK) && (cmd & DRIVER_Y2_STEP_MASK)));
    }
    //  delayMicroseconds(3);     // wait 3 us before sending falling pulse for step
    StepperDriverStep(0);
}

/**********************************************************************************/
inline void StepperDriverEnable(bool en)
{
    bool enable = StepperDriverEnableState[en];

    StepperDriverEnableMode = en;

    digitalWrite(PIN_X1_EN, enable); // Cde Driver X1
    digitalWrite(PIN_X2_EN, enable); // Cde Driver X2
    digitalWrite(PIN_Y1_EN, enable); // Cde Driver Y1
    digitalWrite(PIN_Y2_EN, enable); // Cde Driver Y2
}

/**********************************************************************************/

//==============================================================================
//Gestion de la marche asservie des Drivers
//==============================================================================
void StepperDriverManage(void)
{
    if ((MOTEUR_ON_ASSERVI == 1))
    {
        if ((!Switch.ControlMode) && (!Switch.MotorEnable) && (StepperDriverEnableMode != PC.Motor))
        {
            StepperDriverEnable(PC.Motor);
        }
        else
        {
            if (Switch.MotorEnable)
                Desar_Homing();
        }
    }
    else
    {
        if ((Switch.MotorEnable) && (StepperDriverEnableMode != OFF))
        {
            StepperDriverEnable(OFF); // blocage des drivers
            Desar_Homing();
        }
        else
        {
            if ((!Switch.MotorEnable) && (StepperDriverEnableMode != ON))
                StepperDriverEnable(ON);
        }
    }
}
//------------------------------------------------------------------------------
/**********************************************************************************/
void CleanMotorHalt(void)
{
    PC.Motor = OFF;
    StepperDriverStep(OFF);
}

/**********************************************************************************/

inline void HeatingRelay(byte en)
{
    digitalWrite(PIN_RELAY_HEATING, en);
}

/**********************************************************************************/

inline void ComputeRotaryEncoderHeatConsign(void)
{
    #ifdef HEAT_CONSIGN_ROTARY_ENCODER
        bool bntPressed = rotBtn.isBtnPushed();  // Get state and reset flag
        bool cntUpdated = rotBtn.isCntUpdated(); // Get state and reset flag
        if (cntUpdated || bntPressed)
        {
            if (bntPressed)
                Heat.WireConsign += 10 * rotBtn.getValueRot(); // x 10 the value
            else if (cntUpdated)
                Heat.WireConsign += rotBtn.getValueRot(); // Value <- +1 or -1
            rotBtn.resetValueRot();
            if (Heat.WireConsign < 0)
                Heat.WireConsign = 0;
            else if (Heat.WireConsign > MAX_PERCENTAGE_WIRE)
                Heat.WireConsign = MAX_PERCENTAGE_WIRE;
        }
    #endif
}

/**************************}*******************************************************/

inline void HeatingManage(byte mode)
{
    if (mode == MODE_MANU)
    {
        if (!Switch.HeatManu)
        {
#ifdef HEAT_CONSIGN_ROTARY_ENCODER
            ComputeRotaryEncoderHeatConsign();
#else
            Heat.WireConsign = map(analogRead(PIN_POT_WIRE), 0, 1023, 0, 100);
            if (Heat.WireConsign > MAX_PERCENTAGE_WIRE)
                Heat.WireConsign = MAX_PERCENTAGE_WIRE;
#endif
        }
        else
        {
            Heat.WireConsign = 0;
        }

        if (Switch.CutterEnable)
        {
            DISABLE_T2_COMP_OUTPUT_B();
            Heat.CutterConsign = 0;
            OCR2B = 0;
        }
        else
        {
            ENABLE_T2_COMP_OUTPUT_B();
            Heat.CutterConsign = map(analogRead(PIN_POT_CUTTER), 0, 1023, 0, 100);
            if (Heat.CutterConsign > MAX_PERCENTAGE_CUTTER)
                Heat.CutterConsign = MAX_PERCENTAGE_CUTTER;
            OCR2B = Heat.CutterConsign;
        }
    }
    else
    {
        if (!Switch.HeatPC)
        {
            Heat.WireConsign = PC.Heat;
        }
        else if (!Switch.HeatManu)
        {
#ifdef HEAT_CONSIGN_ROTARY_ENCODER
            ComputeRotaryEncoderHeatConsign();
#else
            Heat.WireConsign = map(analogRead(PIN_POT_WIRE), 0, 1023, 0, 100);
            if (Heat.WireConsign > MAX_PERCENTAGE_WIRE)
                Heat.WireConsign = MAX_PERCENTAGE_WIRE;
#endif
        }
        else
            Heat.WireConsign = 0;
    }

    if (Heat.WireConsign > 0)
    {
        HeatingRelay(ON);
        //ENABLE_T4_COMP_OUTPUT_C();
    }
    else
    {
        HeatingRelay(OFF);
        //DISABLE_T4_COMP_OUTPUT_C();
    }
    if ((CHAUFFE_ASSERV == 1) & (mode == MODE_PC))
    {
        if (WireVitesse == 1) //Correction de la chauffe en fonction des steps
        {
            Heat.WireDynamique = (Heat.WireConsign * CORRECT_CHAUFFE);
            if (Heat.WireDynamique > MAX_PERCENTAGE_WIRE)
                Heat.WireDynamique = MAX_PERCENTAGE_WIRE;
        }
        else
            Heat.WireDynamique = Heat.WireConsign;

        analogWrite(PIN_WIRE_PWM, (Heat.WireDynamique * 2.55));
    }
    else
    {
        //OCR4C = Heat.WireConsign; // PWM for wire heating (stretch 0-100% to a range of 0-254)*/
        analogWrite(PIN_WIRE_PWM, (Heat.WireConsign * 2.55));
    }
}

/**********************************************************************************/

void ResetHeat(void)
{
    Heat.WireConsign = 0;
    PC.Heat = 0;
    HeatingRelay(OFF);
    //OCR4C = 0; // PWM for wire heating (stretch 0-100% to a range of 0-255)*/
    analogWrite(PIN_WIRE_PWM, 0);
    Heat.CutterConsign = 0;
    OCR2B = 0;
}

/**********************************************************************************/

inline void SoundAlarm(bool en)
{
    if(en)
      ENABLE_T2_ISR();
    else
    {
      DISABLE_T2_ISR();
      digitalWrite(PIN_BUZZER, LOW);
    }
}

/*********************************************************************************/

inline void ProcessCommand(void)
{
    switch (CommandBuffer[CommandIndexRead].Cmd)
    {
    case 'M':
        ProcessStep(CommandBuffer[CommandIndexRead].Data.ui8[HIGH_BYTE]);
        break;

    case 'A': // All Motors on/off
        digitalWrite(PIN_DEBUG1, toogle);
        toogle = !toogle;
        PC.Motor = CommandBuffer[CommandIndexRead].Data.ui8[HIGH_BYTE];
        if (MOTEUR_ON_ASSERVI)
            StepperDriverEnable(StepperDriverEnableTab[Switch.MotorEnable][PC.Motor]);

        break;

    case 'F': // Change the stepper Feedrate
        // OCR5A values 255 = 250Hz 190 = 328Hz 127 = 500Hz 63 = 1 kHz 31 = 2KHz 15 = 4 kHz
        PC.Feedrate = CommandBuffer[CommandIndexRead].Data.ui16;
        OCR5A = PC.Feedrate;
        TCNT5 = 0;
        break;

    case 'P': // Pause
        DISABLE_T5_ISR();
        ActivePause = 1;
        PC.Pause = CommandBuffer[CommandIndexRead].Data.ui16;
        /*unsigned int H_Pause = 0;
        unsigned int L_Pause = 0;
        H_Pause = PC.Pause;
        L_Pause = PC.Pause;
        H_Pause << 8 ;
        L_Pause >> 8 ;
        PC.Pause = H_Pause | L_Pause ;
        */
        kPause = 0;
        kPause = millis() + PC.Pause;
        Cad_Aff = millis() + 250;

        printLCD_I(0, 0, "Pause      secondes ");
        printLCD_I(7, 0, ftostr62rj(PC.Pause / 1000));

        break;

    case 'H': // Wire Heat ON/OFF (may be programmed as PWM (analog out))
        PC.Heat = CommandBuffer[CommandIndexRead].Data.ui8[HIGH_BYTE];
        break;

    default:
        break;
    }
}

/**********************************************************************************/

inline void IsrProcessBuffer(void)
{
    while (CommandCounter != 0)
    {
        ProcessCommand();
        CommandIndexRead++;
        CommandCounter--;
        CheckComBufferUnderflow();
        if (CommandBuffer[CommandIndexRead].Cmd == 'M')
            break;
    }
}

/**********************************************************************************/

ISR(TIMER1_COMPA_vect)
{
}

/**********************************************************************************/

ISR(TIMER2_COMPA_vect)
{
    buzzerToogle = !buzzerToogle;
    digitalWrite(PIN_BUZZER, buzzerToogle);
}

/**********************************************************************************/

ISR(TIMER5_COMPA_vect)
{
    digitalWrite(PIN_DEBUG3, HIGH);
    IsrProcessBuffer();
    digitalWrite(PIN_DEBUG3, LOW);
}

/**********************************************************************************/
#ifndef USART_ARDUINO
    ISR(USART0_RX_vect)
    {
        digitalWrite(PIN_DEBUG4, HIGH);
        ComParse();
        digitalWrite(PIN_DEBUG4, LOW);
    }
#endif

/**********************************************************************************/

void BufferFlush(void)
{
    CommandCounter = 0;
    ComOverflow = false;
    TX_WRITE('C');
    CommandIndexRead = 0;
    CommandIndexWrite = 0;
    ParserState = PARSER_STATE_CMD;
}

/**********************************************************************************/

inline void CmdBufferWrite(unsigned char *Data)
{
    CommandBuffer[CommandIndexWrite].Cmd = Data[0];
    CommandBuffer[CommandIndexWrite].Data.ui8[HIGH_BYTE] = Data[1];
    CommandBuffer[CommandIndexWrite].Data.ui8[LOW_BYTE] = Data[2];
    CommandIndexWrite++;
    if (CommandCounter < 255)
        CommandCounter++;
    CheckComBufferOverflow();
}

/**********************************************************************************/

inline void CheckComBufferOverflow(void)
{
    if ((CommandCounter >= COM_BUFFER_OVERFLOW_TRIGGER) && (!ComOverflow))
    {
        ComOverflow = true;
        digitalWrite(PIN_DEBUG2, HIGH);
        TX_WRITE('S');
    }
}

/**********************************************************************************/

inline void CheckComBufferUnderflow(void)
{
    if ((CommandCounter <= COM_BUFFER_UNDERFLOW_TRIGGER) && (ComOverflow))
    {
        ComOverflow = false;
        digitalWrite(PIN_DEBUG2, LOW);
        TX_WRITE('C');
    }
}

/**********************************************************************************/

inline void DataProcess(unsigned char *data)
{
    CmdData i;

    switch (data[0])
    {
    case 'A':
        if (data[1] == '1')
            data[1] = ON;
        else
            data[1] = OFF;
        break;

    case 'F':
        i.ui8[HIGH_BYTE] = data[1];
        i.ui8[LOW_BYTE] = data[2];
        if (i.ui16 > MAX_FEEDRATE_TIMER_VALUE)
            i.ui16 = MAX_FEEDRATE_TIMER_VALUE;
        else if (i.ui16 < MIN_FEEDRATE_TIMER_VALUE)
            i.ui16 = MIN_FEEDRATE_TIMER_VALUE;

        data[1] = i.ui8[HIGH_BYTE];
        data[2] = i.ui8[LOW_BYTE];
        break;

    case 'P':
        /* char H_Pause = 0;
         char L_Pause = 0;

         L_Pause = data[1];
         H_Pause = data[2];
         data[1] = H_Pause;
         data[2] = L_Pause;
         */
        i.ui8[HIGH_BYTE] = data[1];
        i.ui8[LOW_BYTE] = data[2];

        if (i.ui16 > MAX_PAUSE_VALUE)
            i.ui16 = MAX_PAUSE_VALUE;
        else if (i.ui16 < MIN_PAUSE_VALUE)
            i.ui16 = MIN_PAUSE_VALUE;

        data[1] = i.ui8[HIGH_BYTE];
        data[2] = i.ui8[LOW_BYTE];

        break;

    case 'H':
        if (data[1] > MAX_PERCENTAGE_WIRE)
            data[1] = MAX_PERCENTAGE_WIRE;
        break;

    default:
        break;
    }
}

/**********************************************************************************/

inline void ComParse(void)
{
    static byte i = 0;
    static unsigned char Cmd[CMD_DATA_SIZE];
    static byte CmdSize = 0;
    unsigned char data;

    RX_READ(data);
    switch (ParserState)
    {
    case PARSER_STATE_CMD:
        if ((data == 'A') || (data == 'H') || (data == 'M'))
        {
            Cmd[0] = data;
            i = 1;
            CmdSize = TWO_BYTE_CMD;
            ParserState = PARSER_STATE_DATA;
        }
        else if ((data == 'F') || (data == 'P'))
        {
            Cmd[0] = data;
            i = 1;
            CmdSize = THREE_BYTE_CMD;
            ParserState = PARSER_STATE_DATA;
        }
        break;

    case PARSER_STATE_DATA:
        Cmd[i++] = data;
        if (i >= CmdSize)
        {
            DataProcess(Cmd);
            CmdBufferWrite(Cmd);
            i = 0;
            ParserState = PARSER_STATE_CMD;
        }
        break;

    default:
        break;
    }
}

/**********************************************************************************/

void GetSwitchStatus(void)
{
    Switch.Status = 0;

    Switch.ControlMode = digitalRead(PIN_SWITCH_CONTROL_MODE);
    Switch.Status += SwitchStatusTab[SWITCH_STATUS_CONTROL_MODE][Switch.ControlMode];
    Switch.MotorEnable = digitalRead(PIN_SWITCH_MOTOR);
    Switch.Status += SwitchStatusTab[SWITCH_STATUS_MOTOR_ENABLE][Switch.MotorEnable];
    Switch.HeatPC = digitalRead(PIN_SWITCH_HEATING_PC);
    Switch.Status += SwitchStatusTab[SWITCH_STATUS_HEAT_PC][Switch.HeatPC];
    Switch.HeatManu = digitalRead(PIN_SWITCH_HEATING_MANU);
    Switch.Status += SwitchStatusTab[SWITCH_STATUS_HEAT_MANU][Switch.HeatManu];
    Switch.CutterEnable = digitalRead(PIN_SWITCH_HEATING_CUTTER);
    Switch.Status += SwitchStatusTab[SWITCH_STATUS_CUTTER_ENABLE][Switch.CutterEnable];
    //Al>
    //Switch.EndStopShunt = digitalRead(PIN_BP_HOMING);
    //Switch.Status += SwitchStatusTab[SWITCH_STATUS_ENDSTOP_SHUNT][Switch.EndStopShunt];
    //Switch.HomingOk = Switch.HomingOk;
    Switch.Status += SwitchStatusTab[SWITCH_STATUS_HOMING_OK][Switch.HomingOk];
    //<
    //Al>
    byte SomEndStop = (((digitalRead(PIN_X1_LIMIT) ^ INV_FDC_X1) << 0) | ((digitalRead(PIN_Y1_LIMIT) ^ INV_FDC_Y1) << 2) | ((digitalRead(PIN_X2_LIMIT) ^ INV_FDC_X2) << 1) | ((digitalRead(PIN_Y2_LIMIT) ^ INV_FDC_Y2) << 3));
    if (SomEndStop == 0)
        Switch.EndStop = 0;
    else
        Switch.EndStop = 1;

    //<
    Switch.Status += SwitchStatusTab[SWITCH_STATUS_ENDSTOP][Switch.EndStop];

#ifdef DEBUG
    Switch.ControlMode = false;
    Switch.MotorEnable = false;
    Switch.Status &= 0xFE;
#endif
}

/*********************************************************************************/
/************************ PRINTING STRING ON SERIAL ******************************/
/*********************************************************************************/
void printSerial(char const *str){
    #ifdef USART_ARDUINO
        Serial.print(str);
    #else
        for (uint8_t i = 0; i < strlen(str); i++){ 
            while (( UCSR0A & (1<<UDRE0))  == 0){};
            UDR0 = str[i]; 
        }
    #endif
}

void printSerialLn(char const *str){
    #ifdef USART_ARDUINO
        Serial.println(str);
    #else
        printSerial(str);
        printSerial("\n");
    #endif
}

void printSerial(double value){
    #ifdef USART_ARDUINO
        Serial.print(value);
    #else
        char str[20];
        sprintf(str, "%lf", value); 
        printSerial(str);
    #endif
}

/*********************************************************************************/

void printLCD(uint8_t col, uint8_t row,
              const char *s)
{

#ifdef MATRIX_LCD
    lcdMatrix.printLcd(col, row, s);
#else
    lcd.setCursor(col, row);
    lcd.print(s);
#endif
}

/*********************************************************************************/

void printLCD_I(uint8_t col, uint8_t row, const char *s)
{
    printLCD(col, row, s);
#ifdef MATRIX_LCD
    // Print imidialty -> You must use this methode
    // before a Do/While
    printMatrix();
#endif
}

/*********************************************************************************/

void printMatrix()
{
#ifdef MATRIX_LCD
    if (lcdMatrix.isupdated())
        lcdMatrix.printMatrix();
#endif
}

/*********************************************************************************/

void clearLCD()
{
#ifdef MATRIX_LCD
    lcdMatrix.clearLcd();
#else
    lcd.clear();
#endif
}

/*********************************************************************************/

void HMI_WriteModeManu(void)
{
    printLCD(0, 2, MANU);
    printLCD(0, 3, CLEAN10);
}

/*********************************************************************************/

void HMI_WriteModePC(void)
{
    printLCD(0, 2, TITLE_CAT);
    printLCD(6, 3, MM_S);
}

/*********************************************************************************/

inline void HMI_InitScreen(void)
{
// Welcome text
#ifndef MATRIX_LCD
    lcd.begin(LCD_COLUMN_COUNT, LCD_LINE_COUNT);
#endif
    clearLCD();
    printLCD(0, 0, TITLE_JEDICUT);
    printLCD(7, 1, VERSION);
    printLCD(7, 2, STRINGIFY(BAUDRATE));

#ifdef BUZZER_ON
    printLCD(5, 3, BUZZ_ON);
    SoundAlarm(ON);
    delay(500);
    SoundAlarm(OFF);
#else
    printLCD(5, 3, BUZZ_OFF);
#endif
}

/*********************************************************************************/

inline void HMI_ParamsScreen(void)
{
    clearLCD();

    static char temp0[sizeof(MM_STEP) + 10] = {' '}; // +10 because sizeof(MM_PER_STEP) make somme crash
    strcat(temp0, MM_STEP);
    strcat(temp0, STRINGIFY(MM_PER_STEP));
    printLCD(0, 0, temp0);

    static char temp1[sizeof(TEXT1) + sizeof(MAX_PERCENTAGE_WIRE) + 1] = {
        ' '};
    strcat(temp1, TEXT1);
    strcat(temp1, itostr3left(MAX_PERCENTAGE_WIRE));
    strcat(temp1, "%");
    printLCD(0, 1, temp1);

    static char temp2[sizeof(TEXT2) + sizeof(MAX_PERCENTAGE_CUTTER) + 1] = {
        ' '};
    strcat(temp2, TEXT2);
    strcat(temp2, itostr3left((MAX_PERCENTAGE_CUTTER)));
    strcat(temp2, "%");
    printLCD(0, 2, temp2);
#ifdef HEAT_CONSIGN_ROTARY_ENCODER
    printLCD(0, 3, TEXT4);
#else
    printLCD(0, 3, TEXT3);
#endif
}

/*********************************************************************************/

inline void HMI_InitSwitchScreen(void)
{
    clearLCD();
    printLCD(0, 1, TEXT6);
}

/*********************************************************************************/

inline bool HMI_SwitchInitScreen(void)
{
    static byte old = 0;
    byte status = Switch.Status;

#ifdef DEBUG
    status = 0x1F;
#endif

    if (IsSwitchNotInitialized(status))
    {
        if (old != status)
        {
            if (!Switch.ControlMode)   // test si l'inter Mode est sur Manu
                printLCD(0, 2, TEXT7); // attente inter Mode sur Manu
            else if (!Switch.MotorEnable)
                printLCD(0, 2, TEXT8); // attente inter Moteur sur OFF
            else if (!Switch.HeatPC || !Switch.HeatManu)
                printLCD(0, 2, TEXT9); // attente inter Chauffe sur OFF
            else if (!Switch.CutterEnable)
                printLCD(0, 2, TEXT10); // attente inter Moteur sur OFF
            old = status;
        }
        return false;
    }

    clearLCD();
    printLCD(0, 0, MachineName);
    printLCD(0, 1, TEXT14);
    printLCD(0, 3, TITLE_VAR);

#ifdef DEBUG
    digitalWrite(13, LOW);
#endif

    return true;
}

/*********************************************************************************/

inline void HMI_ModeScreen(void)
{
    static byte old = 0;

    if (old != Switch.Status)
    {
        printLCD(0, 2, CLEAN20);
        if (Switch.ControlMode)
        {
            printLCD(0, 2, MAN);

            if (Switch.MotorEnable)
                printLCD(7, 2, OFF_STATUS);
            else
                printLCD(7, 2, ON_STATUS);

            if (!Switch.HeatManu)
                printLCD(12, 2, MAN);
            else if (!Switch.HeatPC)
                printLCD(12, 2, DIS);
            else
                printLCD(12, 2, OFF_STATUS);

            if (Switch.CutterEnable)
                printLCD(17, 2, OFF_STATUS);
            else
                printLCD(17, 2, ON_STATUS);
        }
        else
        {
            printLCD(0, 2, PC_STATUS);

            if (Switch.MotorEnable)
                printLCD(7, 2, OFF_STATUS);
            else
                printLCD(8, 2, PC_STATUS);

            if (!Switch.HeatManu)
                printLCD(12, 2, MAN);
            else if (!Switch.HeatPC)
                printLCD(12, 2, PC_STATUS);
            else
                printLCD(12, 2, OFF_STATUS);

            printLCD(17, 2, DIS);
        }

        //  if(Switch.EndStop && Switch.EndStopShunt)
        if (Switch.EndStop && !Switch.ControlMode)
        {
            SoundAlarm(ON);
            printLCD(5, 2, "K");
        }
        else
        {
            SoundAlarm(OFF);
            printLCD(5, 2, "I");
        }

        HMI.ProcessDigit = true;
        old = Switch.Status;
    }
}

/*********************************************************************************/

inline void HMI_ManuDigitScreen(void)
{
    static char line[21] = {TITLE_PERCENT};
    if ((Heat.WireConsign != HMI.WireConsign) || (Heat.CutterConsign != HMI.CutterConsign) || (HMI.ProcessDigit))
    {
        HMI.WireConsign = Heat.WireConsign;
        HMI.CutterConsign = Heat.CutterConsign;

        line[11] = HMI.WireConsign >= 100 ? ('0' + (HMI.WireConsign / 100)) : ' ';
        line[12] = HMI.WireConsign >= 10 ? ('0' + ((HMI.WireConsign / 10) % 10)) : ' ';
        line[13] = '0' + (HMI.WireConsign % 10);

        line[16] = HMI.CutterConsign >= 100 ? ('0' + (HMI.CutterConsign / 100)) : ' ';
        line[17] = HMI.CutterConsign >= 10 ? ('0' + ((HMI.CutterConsign / 10) % 10)) : ' ';
        line[18] = '0' + (HMI.CutterConsign % 10);
        printLCD(0, 3, line);
        HMI.ProcessDigit = false;
    }
}

/*********************************************************************************/

inline void HMI_PcDigitScreen(void)
{
    static char line[21] = {TITLE_VAR2};
    float mmPerSec;
    byte val, valDec;

    if ((Heat.WireConsign != HMI.WireConsign) || (PC.Feedrate != HMI.Feedrate) || (HMI.ProcessDigit))
    {
        HMI.WireConsign = Heat.WireConsign;
        HMI.Feedrate = PC.Feedrate;

        if (HMI.Feedrate > 0)
        {
            mmPerSec = MM_PER_SECONDE / (float)(HMI.Feedrate);
            val = (byte)mmPerSec;
            valDec = ((unsigned int)(mmPerSec * 100.0) % 100);

            line[0] = val >= 100 ? ('0' + (val / 100)) : ' ';
            line[1] = val >= 10 ? ('0' + ((val / 10) % 10)) : ' ';
            line[2] = '0' + (val % 10);
            line[4] = valDec >= 10 ? ('0' + ((valDec / 10) % 10)) : '0';
            line[5] = '0' + (valDec % 10);
        }

        line[11] = HMI.WireConsign >= 100 ? ('0' + (HMI.WireConsign / 100)) : ' ';
        line[12] = HMI.WireConsign >= 10 ? ('0' + ((HMI.WireConsign / 10) % 10)) : ' ';
        line[13] = '0' + (HMI.WireConsign % 10);
        line[21] = '\0';
        printLCD(0, 3, line);
        HMI.ProcessDigit = false;
    }
}

/*********************************************************************************/

inline void HMI_DigitScreen(void)
{
    if (Switch.ControlMode)
        HMI_ManuDigitScreen();
    else
        HMI_PcDigitScreen();
}

/*********************************************************************************/

inline void HMI_Manage(void)
{
    static unsigned long i = 0;
    rotBtn.rotBtnRefresh();
    switch (HMI.State)
    {
    case HMI_MODE_SCREEN:
        HMI_ModeScreen();
        HMI_DigitScreen();
        break;

    case HMI_INIT_SCREEN:
        HMI_InitScreen();
        i = millis() + 3000;
        HMI.State = HMI_INIT_DELAY;
        break;

    case HMI_INIT_DELAY:
        if (i < millis())
            HMI.State = HMI_PARAMS_SCREEN;
        break;

    case HMI_PARAMS_SCREEN:
        HMI_ParamsScreen();
        i = millis() + 3000;
        HMI.State = HMI_PARAMS_DELAY;
        break;

    case HMI_PARAMS_DELAY:
        if (i < millis())
        {
            HMI_InitSwitchScreen();
            HMI.State = HMI_SWITCH_SCREEN;
        }
        break;

    case HMI_SWITCH_SCREEN:
        if (HMI_SwitchInitScreen())
            HMI.State = HMI_MODE_SCREEN;
        break;
    }
}

/*********************************************************************************/

inline void ModeManage(void)
{
    switch (modeState)
    {
    case MODE_INIT:
        if (HMI.State == HMI_MODE_SCREEN)
            modeState = MODE_MANU;
        break;

    case MODE_MANU:
        if (!Switch.ControlMode && !Switch.EndStop && !Switch.MotorEnable && Switch.HomingOk)
        {
            BufferFlush();
            ENABLE_T5_ISR();
            ENABLE_T1_ISR();
            modeState = MODE_PC;
        }
        else
        {
            EndStopManage();
            if (Switch.MotorEnable == 0)
            {
                HomingManage();
            }
            HeatingManage(MODE_MANU);
        }
        break;

    case MODE_PC:
        if (Switch.ControlMode || Switch.EndStop || Switch.MotorEnable)
        {
            DISABLE_T1_ISR();
            DISABLE_T5_ISR();

            OCR5A = 255;

            BufferFlush();
            CleanMotorHalt();
            ResetHeat();

            modeState = MODE_MANU;
        }
        else
            HeatingManage(MODE_PC);
        break;
    }
}

void getCurrentVoltage(){
    
    #ifdef INA219
        probCurrent.current_mA = ina219.getCurrent_mA();
        probCurrent.power_mW = ina219.getPower_mW();
        if(probCurrent.power_mW < 0.0) probCurrent.power_mW = probCurrent.power_mW * -1.0;
        probCurrent.voltage_V = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000);

        #ifdef D_INA219
            printSerial(probCurrent.voltage_V); printSerial(" V");
            printSerial("\t"); printSerial(probCurrent.current_mA); printSerial(" mA");
            printSerial("\t"); printSerial(probCurrent.power_mW); printSerialLn(" mW");
        #endif
    
    #endif
}



/**********************************************************************************/
/**** The main loop                                                           *****/
/**********************************************************************************/
void loop(void)
{
    GetSwitchStatus();
    StepperDriverManage();
    PauseManage();
    ModeManage();
    //HMI_Manage();

    
    //If the current probe INA219 is implemented.
    #ifdef INA219
        getCurrentVoltage();
    #endif

    //If some text should be updated, also printMatrix() refresh the display.
    #ifdef MATRIX_LCD
        printMatrix();
    #endif
}