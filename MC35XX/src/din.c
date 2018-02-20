/*
 * din.c
 *
 *  Created on: 19 февр. 2018 г.
 *      Author: Roman
 */

#include "din.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "modbus.h"
#include "stm32f10x_conf.h"
#include "pcf8575.h"

extern unsigned char discrInp[DiscreteInputsLimit];
extern unsigned short dinFilter;
extern unsigned short inpReg[InputRegistersLimit];

static unsigned char tmpInp[DiscreteInputsLimit]={0};
static unsigned short filterCnt[DiscreteInputsLimit]={0};
static unsigned short di_inp_reg_addr = 0x00;
static unsigned char di8_set = 0; // different sets of gpio for di8

void set_di16_inp_reg_addr(unsigned short value) {
	if(value>=InputRegistersLimit) value = 0;
	di_inp_reg_addr = value;
}

void set_di8_inp_reg_addr(unsigned short value) {
	set_di16_inp_reg_addr(value);
}

void set_di8_set(unsigned char value) {
	di8_set = value;
}

static void useFilter(unsigned char curValue,unsigned char inpNum)
{
    if(curValue==0)
    {
        if(tmpInp[inpNum]==0) filterCnt[inpNum]++;else filterCnt[inpNum]=0;
    }
    else
    {
        if(tmpInp[inpNum]) filterCnt[inpNum]++;else filterCnt[inpNum]=0;
    }
    tmpInp[inpNum] = curValue;
    if(filterCnt[inpNum]>=dinFilter) {discrInp[inpNum] = curValue;filterCnt[inpNum]=0;}
}

static void readDin8(void) {
	unsigned short res = 0;
	unsigned short tmp=0;
	if(di8_set==0) {
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==Bit_SET) useFilter(0,0); else {useFilter(1,0);res|=0x0001;}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==Bit_SET) useFilter(0,1); else {useFilter(1,1);res|=0x0002;}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==Bit_SET) useFilter(0,2); else {useFilter(1,2);res|=0x0004;}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==Bit_SET) useFilter(0,3); else {useFilter(1,3);res|=0x0008;}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==Bit_SET) useFilter(0,4); else {useFilter(1,4);res|=0x0010;}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)==Bit_SET) useFilter(0,5); else {useFilter(1,5);res|=0x0020;}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)==Bit_SET) useFilter(0,6); else {useFilter(1,6);res|=0x0040;}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)==Bit_SET) useFilter(0,7); else {useFilter(1,7);res|=0x0080;}
		set_first8bits(res);
	}else {
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==Bit_SET) useFilter(0,0); else {useFilter(1,0);res|=0x0001;}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==Bit_SET) useFilter(0,1); else {useFilter(1,1);res|=0x0002;}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==Bit_SET) useFilter(0,2); else {useFilter(1,2);res|=0x0004;}
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==Bit_SET) useFilter(0,3); else {useFilter(1,3);res|=0x0008;}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==Bit_SET) useFilter(0,4); else {useFilter(1,4);res|=0x0010;}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==Bit_SET) useFilter(0,5); else {useFilter(1,5);res|=0x0020;}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==Bit_SET) useFilter(0,6); else {useFilter(1,6);res|=0x0040;}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==Bit_SET) useFilter(0,7); else {useFilter(1,7);res|=0x0080;}
		set_second8bits(res);
	}
	res = 0;
	for(tmp=0;tmp<8;tmp++) if(discrInp[tmp]) res|=1<<tmp;
	inpReg[di_inp_reg_addr] = res;
}

static void readDin16(void)
{
	unsigned short res = 0;
	unsigned short tmp=0;
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==Bit_SET) useFilter(0,0); else {useFilter(1,0);res|=0x0001;}
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==Bit_SET) useFilter(0,1); else {useFilter(1,1);res|=0x0002;}
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==Bit_SET) useFilter(0,2); else {useFilter(1,2);res|=0x0004;}
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==Bit_SET) useFilter(0,3); else {useFilter(1,3);res|=0x0008;}
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==Bit_SET) useFilter(0,4); else {useFilter(1,4);res|=0x0010;}
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)==Bit_SET) useFilter(0,5); else {useFilter(1,5);res|=0x0020;}
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)==Bit_SET) useFilter(0,6); else {useFilter(1,6);res|=0x0040;}
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)==Bit_SET) useFilter(0,7); else {useFilter(1,7);res|=0x0080;}
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==Bit_SET) useFilter(0,8); else {useFilter(1,8);res|=0x0100;}
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==Bit_SET) useFilter(0,9); else {useFilter(1,9);res|=0x0200;}
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==Bit_SET) useFilter(0,10); else {useFilter(1,10);res|=0x0400;}
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==Bit_SET) useFilter(0,11); else {useFilter(1,11);res|=0x0800;}
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==Bit_SET) useFilter(0,12); else {useFilter(1,12);res|=0x1000;}
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==Bit_SET) useFilter(0,13); else {useFilter(1,13);res|=0x2000;}
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==Bit_SET) useFilter(0,14); else {useFilter(1,14);res|=0x4000;}
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==Bit_SET) useFilter(0,15); else {useFilter(1,15);res|=0x8000;}
    set_led_state(res);
    res = 0;
    for(tmp=0;tmp<16;tmp++) if(discrInp[tmp]) res|=1<<tmp;
    inpReg[di_inp_reg_addr] = res;

}

static void din16Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);


	/*Configure GPIO pins : PA4 PA5 PA6 PA7 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	/*Configure GPIO pins : PC4 PC5 PC6 PC7
						   PC8 PC9 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC,&GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB12 PB13
						   PB14 PB15 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
}

static void din8Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);

	if(di8_set==0) {
		/*Configure GPIO pins : PB12 PB13 B14 PB15 */
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB,&GPIO_InitStruct);

		/*Configure GPIO pins : PC6 PC7 PC8 PC9 */
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOC,&GPIO_InitStruct);
	}else {
		/*Configure GPIO pins : PB0 PB1 */
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB,&GPIO_InitStruct);

		/*Configure GPIO pins : PC4 PC5 */
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOC,&GPIO_InitStruct);

		/*Configure GPIO pins : PA4 PA5 PA6 PA7 */
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA,&GPIO_InitStruct);
	}
}

void din16Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    xLastExecutionTime = xTaskGetTickCount();
    din16Init();
    for(;;)
    {
    	readDin16();
        vTaskDelayUntil( &xLastExecutionTime, DIN_DELAY );
    }
}

void din8Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    xLastExecutionTime = xTaskGetTickCount();
    din8Init();
    for(;;)
    {
    	readDin8();
        vTaskDelayUntil( &xLastExecutionTime, DIN_DELAY );
    }
}
