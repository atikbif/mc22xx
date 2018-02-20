#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ain.h"
#include "stm32f10x_conf.h"
#include "modbus.h"
#include "pcf8575.h"

#define TRANSMIT	0
#define RECEIVE		1
#define ClockSpeed              100000		// I2C clock

static void init_ain(void);
static void read_ain(unsigned char value);
static void adc_write_set(unsigned char num);
static void get_ext_adc(void);


static volatile unsigned short adc_data[16];
static volatile unsigned long tmp_data[16]={0};
static unsigned long filterCnt=0;
static volatile short ext_adc;
static volatile short _EA[16];
static unsigned short adc_led = 0x0000;	// analog inputs led indication

extern unsigned short wr_conf_flag;
extern unsigned short ainFilter;
extern unsigned short inpReg[InputRegistersLimit];

struct
{
	unsigned char direction;
	unsigned char tx[8];
	unsigned char rx[8];
	unsigned char send_amount;
	unsigned char rcv_amount;
	unsigned char cnt;
	unsigned char stat;
	unsigned char addr;
	unsigned short err;
}static volatile __attribute__((aligned(4))) i2c;

static unsigned char ext_cnt=0;
static unsigned char anum=0;

void ain16Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    init_ain();
    xLastExecutionTime = xTaskGetTickCount();
    for(;;)
    {
        read_ain(16);
        vTaskDelayUntil( &xLastExecutionTime, AIN_DELAY );
    }
}

void ain8Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    init_ain();
    xLastExecutionTime = xTaskGetTickCount();
    for(;;)
    {
        read_ain(8);
        vTaskDelayUntil( &xLastExecutionTime, AIN_DELAY );
    }
}

void init_ain(void) // изменён порядок
{
    GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef   I2C_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8); // delay;

	if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)==Bit_RESET) || (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)==Bit_RESET)) return;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	I2C_DeInit(I2C1);
	//I2C_SoftwareResetCmd(I2C1,ENABLE);
	i2c.err=0;

	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//GPIO_Mode_AF_PP;//GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

	I2C_Cmd(I2C1, ENABLE);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x30;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
	I2C_Init(I2C1, &I2C_InitStructure);
}

void adc_write_set(unsigned char num)
{
	if(num>15) num=0;
	i2c.direction = TRANSMIT;
	i2c.send_amount=3;
	i2c.tx[0] = 0x01;
	switch(num)
	{
		case 0:i2c.addr = 0x92;i2c.tx[1] = 0xF4;break;
		case 1:i2c.addr = 0x92;i2c.tx[1] = 0xE4;break;
		case 2:i2c.addr = 0x92;i2c.tx[1] = 0xD4;break;
		case 3:i2c.addr = 0x92;i2c.tx[1] = 0xC4;break;
		case 4:i2c.addr = 0x90;i2c.tx[1] = 0xF4;break;
		case 5:i2c.addr = 0x90;i2c.tx[1] = 0xE4;break;
		case 6:i2c.addr = 0x90;i2c.tx[1] = 0xD4;break;
		case 7:i2c.addr = 0x90;i2c.tx[1] = 0xC4;break;
		case 8:i2c.addr = 0x96;i2c.tx[1] = 0xC4;break;
		case 9:i2c.addr = 0x96;i2c.tx[1] = 0xD4;break;
		case 10:i2c.addr = 0x96;i2c.tx[1] = 0xE4;break;
		case 11:i2c.addr = 0x96;i2c.tx[1] = 0xF4;break;
		case 12:i2c.addr = 0x94;i2c.tx[1] = 0xC4;break;
		case 13:i2c.addr = 0x94;i2c.tx[1] = 0xD4;break;
		case 14:i2c.addr = 0x94;i2c.tx[1] = 0xE4;break;
		case 15:i2c.addr = 0x94;i2c.tx[1] = 0xF4;break;
	}
	i2c.tx[2] = 0xD3;
	if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)==Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)==Bit_SET))
	{
		I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
		I2C_GenerateSTART(I2C1, ENABLE);
	}
	else
	{
		i2c.err++;
		if(i2c.err>=1000){i2c.err=0;init_ain();}
	}
}

void get_ext_adc(void)
{
	i2c.direction = RECEIVE;
	i2c.tx[0]=0x00;
	i2c.stat=0;
	if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)==Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)==Bit_SET))
	{
	    if(wr_conf_flag==0) {
    	    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
            I2C_GenerateSTART(I2C1, ENABLE);
	    }
		i2c.err++;
		if(i2c.err>=100){i2c.err=0;init_ain();}
	}
}

void read_ain(unsigned char value)
{
    unsigned char tmp;
    //static double ain;

    switch(ext_cnt)
    {
        case 0:
            if(ext_adc<0) _EA[anum]=0;
            else {
                _EA[anum] = (double)ext_adc * 1.08/16;
            }
            anum++;
            if(anum>=value) anum=0;
            adc_write_set(anum);
            break;
        case 1:
            get_ext_adc();
            break;
    }
    ext_cnt++;
    if(ext_cnt>=2) ext_cnt=0;

    filterCnt++;
    for(tmp=0;tmp<value;tmp++)
    {
        adc_data[tmp] = (_EA[tmp]>0?_EA[tmp]*2:0);
        tmp_data[tmp]+=adc_data[tmp];
    }
    if((filterCnt)>=ainFilter)
    {
    	adc_led = 0;
        for(tmp=0;tmp<value;tmp++)
        {
            inpReg[tmp] = ((double)tmp_data[tmp]/filterCnt)+0.5; tmp_data[tmp]=0;
			if(inpReg[tmp]>=700) adc_led |= 0x01 << tmp;
            inpReg[tmp+value] = inpReg[tmp]<4095?inpReg[tmp]*16:65535;
        }
        if(value==16) set_led_state(adc_led);
		else if(value==8) set_first8bits(adc_led);
        filterCnt=0;
    }
}

void I2C1_EV_IRQHandler(void)
{
	switch(i2c.direction)
	{
		case TRANSMIT:
			switch (I2C_GetLastEvent(I2C1))
			{
				case I2C_EVENT_MASTER_MODE_SELECT:
					I2C_Send7bitAddress(I2C1, i2c.addr, I2C_Direction_Transmitter);
					break;
				case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
					I2C_SendData(I2C1, i2c.tx[0]);
					i2c.cnt=1;
					break;
				case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
					if(i2c.cnt < i2c.send_amount)
					{
						I2C_SendData(I2C1, i2c.tx[i2c.cnt++]);
					}
					else
					{
						I2C_GenerateSTOP(I2C1, ENABLE);
						I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
					}
					break;
			}
			break;
		case RECEIVE:
			switch (I2C_GetLastEvent(I2C1))
			{
				case I2C_EVENT_MASTER_MODE_SELECT:
					if(i2c.stat == 0)
					{
						I2C_Send7bitAddress(I2C1, i2c.addr, I2C_Direction_Transmitter);
					}
					else
					{
						I2C_Send7bitAddress(I2C1, i2c.addr, I2C_Direction_Receiver);
						i2c.stat=0;
					}

					break;
				case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
					I2C_SendData(I2C1, i2c.tx[0]);
					break;
				case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
					i2c.stat=1;
					I2C_GenerateSTART(I2C1, ENABLE);
					i2c.cnt=0;
					break;
				case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:

					break;
				case I2C_EVENT_MASTER_BYTE_RECEIVED:
					i2c.rx[i2c.cnt++] = I2C_ReceiveData(I2C1);
					if(i2c.cnt==1)
					{
						I2C_AcknowledgeConfig(I2C1, DISABLE);
						I2C_GenerateSTOP(I2C1, ENABLE);
					}
					if(i2c.cnt==2)
					{
						I2C_AcknowledgeConfig(I2C1, ENABLE);
						((unsigned char*)&ext_adc)[0]=i2c.rx[1];
						((unsigned char*)&ext_adc)[1]=i2c.rx[0];
						i2c.err=0;
						I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
					}
					break;
			}
			break;
	}
}

void I2C1_ER_IRQHandler(void)
{
	/* Check on I2C1 AF flag and clear it */
	if (I2C_GetITStatus(I2C1, I2C_IT_AF))
	{
		I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
		I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
	}
}
