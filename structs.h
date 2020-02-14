#ifndef _STRUCTS_H
#define _STRUCTS_H
#include "headers.h"
#include "enums.h"

typedef struct TypeBuffer
{
    unsigned char Cmd;
    union CmdData Data;
} TBuffer;

volatile struct StructPC
{
    bool Motor = OFF;
    byte Heat = 0;
    unsigned int Feedrate = 0;
    unsigned int Pause = 0;
} PC;

struct StructHeating
{
    byte WireConsign = 0;
    byte CutterConsign = 0;
    byte WireDynamique = 0;
    uint8_t PidConsign = 0;
} Heat;


struct StructHMI
{
    bool ProcessDigit = false;
    byte State = HMI_INIT_SCREEN;
    byte WireConsign = 0;
    byte CutterConsign = 0;
    unsigned int Feedrate = 0;
} HMI;

struct StructSwitch
{
    bool ControlMode;
    bool MotorEnable;
    bool HeatPC;
    bool HeatManu;
    bool CutterEnable;
    bool HomingOk = 0;
    bool EndStop;
    byte Status;
} Switch;


struct ProbCurrent
{
    float voltage_V = 0;
    float current_mA = 0;
    float power_mW = 0;
    int nbEch = 0;
} probCurrent;


#endif