/*
 * watchdog.c
 *
 *  Created on: 15 ����. 2018 �.
 *      Author: Roman
 */

#include "stm32f10x.h"
#include "watchdog.h"

void iwdg_init(void) {
	// �������� LSI
	RCC_LSICmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
	// ����������� ������ � ��������� IWDG
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	// ������������� ������������
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	// �������� ��� ������������
	IWDG_SetReload(0xEA);
	// ������������ ��������
	IWDG_ReloadCounter();
	// LSI ������ ���� �������
	IWDG_Enable();
}
