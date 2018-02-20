/*
 * pcf8575.c
 *
 *  Created on: 19 февр. 2018 г.
 *      Author: Roman
 */

#include "pcf8575.h"
#include "stm32f10x_conf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define ClockSpeed              10000

struct
{
	unsigned char tx[8];
	unsigned char send_amount;
	unsigned char cnt;
	unsigned char stat;
	unsigned char addr;
	unsigned short err;
}static volatile __attribute__((aligned(4))) i2c2;

static void update_outputs(void);
static void init_pcf8575(void);

static unsigned short led_state = 0xFFFF; // corrected data (pcf8575 pins)
static unsigned short cur_value = 0x0000; // input bits

extern unsigned short wr_conf_flag;

void pcf8575Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    init_pcf8575();
    xLastExecutionTime = xTaskGetTickCount();
    for(;;)
    {
        update_outputs();
        vTaskDelayUntil( &xLastExecutionTime, PCF8575_DELAY );
    }
}

void update_outputs(void) {
	i2c2.send_amount=2;
	i2c2.addr = 0x40;
	i2c2.tx[0] = led_state & 0xFF;
	i2c2.tx[1] = led_state >> 8;
	if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)==Bit_SET))
	{
		if(wr_conf_flag==0) {
			I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
			I2C_GenerateSTART(I2C2, ENABLE);
		}
	}

}

void set_led_state(unsigned short value) {
	unsigned short out_state = 0x0000;
	cur_value = value;

	// low byte
	if(value & 0x8000) out_state|= 1<<4;
	if(value & 0x4000) out_state|= 1<<5;
	if(value & 0x2000) out_state|= 1<<6;
	if(value & 0x1000) out_state|= 1<<7;
	if(value & 0x0800) out_state|= 1<<3;
	if(value & 0x0400) out_state|= 1<<2;
	if(value & 0x0200) out_state|= 1<<1;
	if(value & 0x0100) out_state|= 1<<0;

	// high byte
	if(value & 0x80) out_state|= 1<<15;
	if(value & 0x40) out_state|= 1<<14;
	if(value & 0x20) out_state|= 1<<13;
	if(value & 0x10) out_state|= 1<<12;
	if(value & 0x08) out_state|= 1<<11;
	if(value & 0x04) out_state|= 1<<10;
	if(value & 0x02) out_state|= 1<<9;
	if(value & 0x01) out_state|= 1<<8;

	led_state = ~out_state;
}

void set_first8bits(unsigned char value) {
	unsigned short out_state = cur_value & 0xFF00;
	out_state |= value;
	set_led_state(out_state);
}

void set_second8bits(unsigned char value) {
	unsigned short out_state = cur_value & 0x00FF;
	out_state |= (unsigned short)value<<8;
	set_led_state(out_state);
}

void init_pcf8575(void) // изменён порядок
{
    GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef   I2C_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10); // delay;

	if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==Bit_RESET) || (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)==Bit_RESET)) return;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	I2C_DeInit(I2C2);
	//I2C_SoftwareResetCmd(I2C2,ENABLE);
	i2c2.err=0;

	NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_Cmd(I2C2, ENABLE);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x30;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
	I2C_Init(I2C2, &I2C_InitStructure);
}

void I2C2_EV_IRQHandler(void)
{
	switch (I2C_GetLastEvent(I2C2))
	{
		case I2C_EVENT_MASTER_MODE_SELECT:
			I2C_Send7bitAddress(I2C2, i2c2.addr, I2C_Direction_Transmitter);
			break;
		case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
			I2C_SendData(I2C2, i2c2.tx[0]);
			i2c2.cnt=1;
			break;
		case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
			if(i2c2.cnt < i2c2.send_amount)
			{
				I2C_SendData(I2C2, i2c2.tx[i2c2.cnt++]);
			}
			else
			{
				I2C_GenerateSTOP(I2C2, ENABLE);
				I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
			}
			break;
	}
}

void I2C2_ER_IRQHandler(void)
{
	/* Check on I2C1 AF flag and clear it */
	if (I2C_GetITStatus(I2C2, I2C_IT_AF))
	{
		I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
		I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
	}
}

