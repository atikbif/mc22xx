#include "settings.h"
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#define MAX_VAR_CNT             100
#define FLASH_PAGE_SIZE         ((uint16_t)0x800)
#define BANK1_WRITE_START_ADDR  ((uint32_t)0x08010000)

static unsigned short varBuf[MAX_VAR_CNT];

static unsigned char varCnt=0;

unsigned short wr_conf_flag = 0;
unsigned short mAddress;
unsigned short dinFilter;
unsigned short ainFilter;
unsigned long baudRate;
unsigned short twoStopBits;
unsigned short parityState;
unsigned short linkBreakTime;

void init_settings_memory(unsigned char cnt)
{
    if(cnt>MAX_VAR_CNT) cnt=MAX_VAR_CNT;
    varCnt = cnt;
    if((readVar(MOD_ADDRESS)==0)||(readVar(MOD_ADDRESS)>255)) writeVar(MOD_ADDRESS,1);
    if(readVar(DIN_FILTER)>600) writeVar(DIN_FILTER,2);
    if(readVar(AIN_FILTER)>600) writeVar(AIN_FILTER,2);
    if((readVar(LINK_BREAK_TIME)==0)||(readVar(LINK_BREAK_TIME)>100)) writeVar(LINK_BREAK_TIME,5);
    if(readVar(MOD_BAUDRATE)>4) writeVar(MOD_BAUDRATE,0);
    if(readVar(STOP_BITS)>1) writeVar(STOP_BITS,0);
    if(readVar(PARITY_STATE)>2) writeVar(PARITY_STATE,0);
    mAddress = readVar(MOD_ADDRESS);
    dinFilter = readVar(DIN_FILTER);
    ainFilter = readVar(AIN_FILTER);
    linkBreakTime = readVar(LINK_BREAK_TIME);
    baudRate = readVar(MOD_BAUDRATE);
    switch(baudRate) {
    	case 0:baudRate=115200;break;
    	case 1:baudRate=57600;break;
    	case 2:baudRate=38400;break;
    	case 3:baudRate=19200;break;
    	case 4:baudRate=9600;break;
    	default:baudRate=115200;
    }
    twoStopBits = readVar(STOP_BITS);
    parityState = readVar(PARITY_STATE);
    if(linkBreakTime) linkBreakTime++;
}

unsigned short readVar(unsigned short address)
{
    if(address>=varCnt) return 0;
    return  *(uint16_t *)(BANK1_WRITE_START_ADDR + address*2);
}

char writeVar(unsigned short address, unsigned short value)
{
    unsigned short tmp;
    if(address>varCnt) return 0;
    wr_conf_flag = 1;
    for(tmp=0;tmp<varCnt;tmp++) varBuf[tmp] = readVar(tmp);
    varBuf[address] = value;
    //FLASH_UnlockBank1();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(BANK1_WRITE_START_ADDR);
    for(tmp=0;tmp<varCnt;tmp++)
        FLASH_ProgramHalfWord(BANK1_WRITE_START_ADDR + tmp*2,varBuf[tmp]);
    FLASH_Lock();
    wr_conf_flag = 0;
    return 1;
}

unsigned char getVarCnt(void)
{
    return varCnt;
}
