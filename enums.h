#ifndef _ENUMS_H
#define _ENUMS_H

enum Mode
{
    MODE_INIT = 0,
    MODE_MANU,
    MODE_PC
};

enum ParseState
{
    PARSER_STATE_CMD = 0,
    PARSER_STATE_DATA,
    PARSER_STATE_WRITE_CMD_BUFFER
};

union CmdData {
    unsigned int ui16;
    unsigned char ui8[2];
};

enum HMI_State
{
    HMI_INIT_SCREEN = 0,
    HMI_INIT_DELAY,
    HMI_PARAMS_SCREEN,
    HMI_PARAMS_DELAY,
    HMI_SWITCH_SCREEN,
    HMI_MODE_SCREEN
};

#endif