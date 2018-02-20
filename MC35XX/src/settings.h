#ifndef SETTINGS_H_INCLUDED
#define SETTINGS_H_INCLUDED

#define MOD_ADDRESS     0
#define DIN_FILTER      1
#define AIN_FILTER      2
#define MOD_BAUDRATE  	3
#define LINK_BREAK_TIME 4
#define STOP_BITS		5
#define PARITY_STATE	6

unsigned char getVarCnt(void);
void init_settings_memory(unsigned char cnt);
unsigned short readVar(unsigned short address);
char writeVar(unsigned short address, unsigned short value);

#endif /* SETTINGS_H_INCLUDED */
