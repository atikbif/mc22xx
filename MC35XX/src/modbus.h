#ifndef MODBUS_H_INCLUDED
#define MODBUS_H_INCLUDED

#define InputRegistersLimit     (32+1)
#define HoldingRegistersLimit   32
#define CoilsLimit              32
#define DiscreteInputsLimit     32

#include "data.h"

typedef struct
{
    unsigned char netAddr;      // network address
    unsigned char code;         // command's code
    unsigned short memAddr;     // memory address
    unsigned short cnt;         // count of data
    unsigned char* ptr;         // pointer to data
    char isCmd;                 // test received data (is it modbus command?)
    unsigned char err;          // error code
}modbusCmd;

modbusCmd* searchCmd(buf* rx_data);   // search modbus command in data
void sendAnswer(modbusCmd* cmd, buf* tx_data);

#endif /* MODBUS_H_INCLUDED */
