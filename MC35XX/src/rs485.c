#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "rs485.h"
#include "stm32f10x_conf.h"
#include <string.h>
#include "crc.h"
#include "modbus.h"

#define RX_SIZE 512
#define TX_SIZE 512

static unsigned short rxBreakTime=10;	// delay between a request and an answer

static volatile unsigned char rx_buf[RX_SIZE];
static unsigned char tx_buf[TX_SIZE];
static volatile unsigned short rx_cnt=0;

static unsigned short tmp_tmr=0;
unsigned short linkTmr=0;

extern unsigned short mAddress;
extern unsigned long baudRate;
extern unsigned short twoStopBits;
extern unsigned short parityState;


static buf rx;
static buf tx;

static unsigned short led_tmr = 0;

static buf hello;
const char* helloString = "\r\nMC35 MODULE ver 1.0\r\n";


static void initRS485(void);            //  initialization of uart
static char checkRxData(void);          //  return non zero if data has been received
void write_data(buf* data);             //  send data to uart

// different pause between a request and an answer for different baudrates
static unsigned short getRxBreakTime() {
	unsigned short res = 10;
	switch(baudRate) {
		case 115200:res = 10;break;
		case 57600:res = 20;break;
		case 38400:res = 40;break;
		case 19200:res = 80;break;
		case 9600:res=160;break;
	}
	return res;
}

void initRS485(void)
{
    USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	rxBreakTime = getRxBreakTime();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baudRate;
	if(twoStopBits) USART_InitStructure.USART_StopBits = USART_StopBits_2;
	else USART_InitStructure.USART_StopBits = USART_StopBits_1;
	if(parityState==0) {
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_Parity = USART_Parity_No;
	}else if(parityState==1) {
		USART_InitStructure.USART_Parity = USART_Parity_Odd;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	}
	else if(parityState==2) {
		USART_InitStructure.USART_Parity = USART_Parity_Even;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	}
	else {
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	}

	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_DeInit( TIM5 );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
	TIM_TimeBaseStructure.TIM_Prescaler = 0xFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ( unsigned short ) 0xFFFF;
	TIM_TimeBaseInit( TIM5, &TIM_TimeBaseStructure );
	TIM_ARRPreloadConfig( TIM5, ENABLE );
	TIM_Cmd( TIM5, ENABLE );

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART1, ENABLE);

	GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);

	hello.cnt = strlen(helloString);
	hello.ptr = (unsigned char*)helloString;
	write_data(&hello);
}

void rs485Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    initRS485();
    xLastExecutionTime = xTaskGetTickCount();
    for(;;)
    {
        tmp_tmr++;
        if(tmp_tmr>=1000) {tmp_tmr = 0; linkTmr++;}
        if(checkRxData())
        {
            if(GetCRC16((unsigned char*)rx_buf,rx_cnt)==0)  // check CRC
            {
                if((rx_buf[0]==0)||(rx_buf[0]==255)||(rx_buf[0]==mAddress))
                {
                    linkTmr = 0;
                    rx.cnt = rx_cnt;
                    rx.ptr = (unsigned char*)rx_buf;
                    modbusCmd* res = searchCmd(&rx);    // search modbus command
                    if(res->isCmd)  // if command has been found
                    {
                        tx.ptr = tx_buf;
                        tx.cnt = TX_SIZE;   // max count of data
                        sendAnswer(res,&tx);
                        if(++led_tmr&0x01) GPIOA->ODR ^= GPIO_Pin_0;
                    }
                }
            }
            rx_cnt = 0;
        }
        vTaskDelayUntil( &xLastExecutionTime, RS485_DELAY );
    }
}

void write_data(buf* data)
{
    DMA_InitTypeDef DMA_InitStructure;
    if(data == 0) return;
    if(data->cnt == 0) return;
	GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);
	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_BASE + 4;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)data->ptr;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = data->cnt;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	DMA_Cmd(DMA1_Channel4, ENABLE);
}



char checkRxData(void)
{
    if((rx_cnt)&&(TIM5->CNT >= rxBreakTime)) return 1;
    return 0;
}

void USART1_IRQHandler(void)
{
   if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
   {
     USART_ClearITPendingBit(USART1, USART_IT_TC);
     if(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=RESET)
     {
        GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_RESET);
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
     }
   }
   if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
   {
     TIM5->CNT=0;
	 rx_buf[rx_cnt++] = USART_ReceiveData(USART1);
	 if(rx_cnt>=RX_SIZE) rx_cnt=0;
     USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}


void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC4) != RESET)
	{
		DMA_Cmd(DMA1_Channel4, DISABLE);
		DMA_ClearITPendingBit(DMA1_IT_GL4);
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	}
}
