#ifndef _STRUCTS_H
#define _STRUCTS_H
#include "headers.h"

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

#endif