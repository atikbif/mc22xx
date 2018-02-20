/*
 * watchdog.c
 *
 *  Created on: 15 февр. 2018 г.
 *      Author: Roman
 */

#include "stm32f10x.h"
#include "watchdog.h"

void iwdg_init(void) {
	// включаем LSI
	RCC_LSICmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
	// разрешается доступ к регистрам IWDG
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	// устанавливаем предделитель
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	// значение для перезагрузки
	IWDG_SetReload(0xEA);
	// перезагрузим значение
	IWDG_ReloadCounter();
	// LSI должен быть включен
	IWDG_Enable();
}
