/*
 * dout.h
 *
 *  Created on: 20 февр. 2018 г.
 *      Author: Roman
 */

#ifndef DOUT_H_
#define DOUT_H_

#define DOUT_TASK_PRIORITY                  ( tskIDLE_PRIORITY + 1 )
#define DOUT_DELAY						   ( ( portTickType ) 10 / portTICK_RATE_MS )

void dout16Task( void *pvParameters );
void dout8Task( void *pvParameters );
void set_do16_hold_reg_addr(unsigned short value);
void set_do8_hold_reg_addr(unsigned short value);

#endif /* DOUT_H_ */
