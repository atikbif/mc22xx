/*
 * din.h
 *
 *  Created on: 19 февр. 2018 г.
 *      Author: Roman
 */

#ifndef DIN_H_
#define DIN_H_

#define DIN_TASK_PRIORITY                  ( tskIDLE_PRIORITY + 1 )
#define DIN_DELAY						   ( ( portTickType ) 10 / portTICK_RATE_MS )

void din16Task( void *pvParameters );
void din8Task( void *pvParameters );
void set_di16_inp_reg_addr(unsigned short value);
void set_di8_inp_reg_addr(unsigned short value);
void set_di8_set(unsigned char value);

#endif /* DIN_H_ */
