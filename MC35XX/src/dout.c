/*
 * dout.c
 *
 *  Created on: 20 февр. 2018 г.
 *      Author: Roman
 */

#include "dout.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "modbus.h"
#include "stm32f10x_conf.h"
#include "pcf8575.h"

static unsigned short do_hold_reg_addr = 0x00;
static unsigned short hold_reg_value = 0;

extern unsigned short linkBreakTime;
extern unsigned char coils[CoilsLimit];
extern unsigned short holdReg[HoldingRegistersLimit];
extern unsigned short linkTmr;

static void dout16Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);


	/*Configure GPIO pins : PA4 PA5 PA6 PA7 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

	/*Configure GPIO pins : PC4 PC5 PC6 PC7
						   PC8 PC9 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	GPIO_SetBits(GPIOC,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);

	/*Configure GPIO pins : PB0 PB1 PB12 PB13
						   PB14 PB15 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
}

static void dout8Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);


	/*Configure GPIO pins : PA4 PA5 PA6 PA7 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

	/*Configure GPIO pins : PC4 PC5 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	GPIO_SetBits(GPIOC,GPIO_Pin_4|GPIO_Pin_5);

	/*Configure GPIO pins : PB0 PB1 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1);
}

static void setSafeState(unsigned short cnt)
{
    unsigned char tmp=0;
    if(cnt>=CoilsLimit) cnt = CoilsLimit-1;
    for(tmp=0;tmp<cnt;tmp++) coils[tmp] = 0;
}

static void update_dout16(void)
{
	unsigned short tmp = 0x0000;
	if(hold_reg_value != holdReg[do_hold_reg_addr]) {
		hold_reg_value = holdReg[do_hold_reg_addr];
		for(tmp=0;tmp<16;++tmp) {
			if(hold_reg_value & (0x01 << tmp)) coils[tmp]=1;else coils[tmp]=0;
		}
		tmp=0;
	}
    if(coils[0]) {GPIO_ResetBits(GPIOB,GPIO_Pin_12);tmp|=0x0001;} else GPIO_SetBits(GPIOB,GPIO_Pin_12);
    if(coils[1]) {GPIO_ResetBits(GPIOB,GPIO_Pin_13);tmp|=0x0002;} else GPIO_SetBits(GPIOB,GPIO_Pin_13);
    if(coils[2]) {GPIO_ResetBits(GPIOB,GPIO_Pin_14);tmp|=0x0004;} else GPIO_SetBits(GPIOB,GPIO_Pin_14);
    if(coils[3]) {GPIO_ResetBits(GPIOB,GPIO_Pin_15);tmp|=0x0008;} else GPIO_SetBits(GPIOB,GPIO_Pin_15);
    if(coils[4]) {GPIO_ResetBits(GPIOC,GPIO_Pin_6);tmp|=0x0010;} else GPIO_SetBits(GPIOC,GPIO_Pin_6);
    if(coils[5]) {GPIO_ResetBits(GPIOC,GPIO_Pin_7);tmp|=0x0020;} else GPIO_SetBits(GPIOC,GPIO_Pin_7);
    if(coils[6]) {GPIO_ResetBits(GPIOC,GPIO_Pin_8);tmp|=0x0040;} else GPIO_SetBits(GPIOC,GPIO_Pin_8);
    if(coils[7]) {GPIO_ResetBits(GPIOC,GPIO_Pin_9);tmp|=0x0080;} else GPIO_SetBits(GPIOC,GPIO_Pin_9);
    if(coils[8]) {GPIO_ResetBits(GPIOB,GPIO_Pin_1);tmp|=0x0100;} else GPIO_SetBits(GPIOB,GPIO_Pin_1);
    if(coils[9]) {GPIO_ResetBits(GPIOB,GPIO_Pin_0);tmp|=0x0200;} else GPIO_SetBits(GPIOB,GPIO_Pin_0);
    if(coils[10]) {GPIO_ResetBits(GPIOC,GPIO_Pin_5);tmp|=0x0400;} else GPIO_SetBits(GPIOC,GPIO_Pin_5);
    if(coils[11]) {GPIO_ResetBits(GPIOC,GPIO_Pin_4);tmp|=0x0800;} else GPIO_SetBits(GPIOC,GPIO_Pin_4);
    if(coils[12]) {GPIO_ResetBits(GPIOA,GPIO_Pin_7);tmp|=0x1000;} else GPIO_SetBits(GPIOA,GPIO_Pin_7);
    if(coils[13]) {GPIO_ResetBits(GPIOA,GPIO_Pin_6);tmp|=0x2000;} else GPIO_SetBits(GPIOA,GPIO_Pin_6);
    if(coils[14]) {GPIO_ResetBits(GPIOA,GPIO_Pin_5);tmp|=0x4000;} else GPIO_SetBits(GPIOA,GPIO_Pin_5);
    if(coils[15]) {GPIO_ResetBits(GPIOA,GPIO_Pin_4);tmp|=0x8000;} else GPIO_SetBits(GPIOA,GPIO_Pin_4);
    set_led_state(tmp);
    holdReg[do_hold_reg_addr] = hold_reg_value = tmp;
}

static void update_dout8(void)
{
	unsigned short tmp = 0x0000;
	if(hold_reg_value != holdReg[do_hold_reg_addr]) {
		hold_reg_value = holdReg[do_hold_reg_addr];
		for(tmp=0;tmp<8;++tmp) {
			if(hold_reg_value & (0x01 << tmp)) coils[tmp]=1;else coils[tmp]=0;
		}
		tmp=0;
	}
    if(coils[0]) {GPIO_ResetBits(GPIOB,GPIO_Pin_1);tmp|=0x01;} else GPIO_SetBits(GPIOB,GPIO_Pin_1);
    if(coils[1]) {GPIO_ResetBits(GPIOB,GPIO_Pin_0);tmp|=0x02;} else GPIO_SetBits(GPIOB,GPIO_Pin_0);
    if(coils[2]) {GPIO_ResetBits(GPIOC,GPIO_Pin_5);tmp|=0x04;} else GPIO_SetBits(GPIOC,GPIO_Pin_5);
    if(coils[3]) {GPIO_ResetBits(GPIOC,GPIO_Pin_4);tmp|=0x08;} else GPIO_SetBits(GPIOC,GPIO_Pin_4);
    if(coils[4]) {GPIO_ResetBits(GPIOA,GPIO_Pin_7);tmp|=0x10;} else GPIO_SetBits(GPIOA,GPIO_Pin_7);
    if(coils[5]) {GPIO_ResetBits(GPIOA,GPIO_Pin_6);tmp|=0x20;} else GPIO_SetBits(GPIOA,GPIO_Pin_6);
    if(coils[6]) {GPIO_ResetBits(GPIOA,GPIO_Pin_5);tmp|=0x40;} else GPIO_SetBits(GPIOA,GPIO_Pin_5);
    if(coils[7]) {GPIO_ResetBits(GPIOA,GPIO_Pin_4);tmp|=0x80;} else GPIO_SetBits(GPIOA,GPIO_Pin_4);
    set_second8bits(tmp);
    holdReg[do_hold_reg_addr] = hold_reg_value = tmp;
}

void dout16Task( void *pvParameters ) {
	portTickType xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
	dout16Init();
	hold_reg_value = holdReg[do_hold_reg_addr];
	for(;;)
	{
		if(linkTmr>=linkBreakTime) setSafeState(16);
		update_dout16();
		vTaskDelayUntil( &xLastExecutionTime, DOUT_DELAY );
	}
}

void dout8Task( void *pvParameters ) {
	portTickType xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
	dout8Init();
	hold_reg_value = holdReg[do_hold_reg_addr];
	for(;;)
	{
		if(linkTmr>=linkBreakTime) setSafeState(8);
		update_dout8();
		vTaskDelayUntil( &xLastExecutionTime, DOUT_DELAY );
	}
}

void set_do16_hold_reg_addr(unsigned short value) {
	if(value>=HoldingRegistersLimit) value = 0;
	do_hold_reg_addr = value;
}

void set_do8_hold_reg_addr(unsigned short value) {
	set_do16_hold_reg_addr(value);
}
