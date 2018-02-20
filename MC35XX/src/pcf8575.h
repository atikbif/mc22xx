/*
 * pcf8575.h
 *
 *  Created on: 19 февр. 2018 г.
 *      Author: Roman
 */

#ifndef PCF8575_H_
#define PCF8575_H_

#define PCF8575_TASK_PRIORITY                  ( tskIDLE_PRIORITY + 1 )
#define PCF8575_DELAY						   ( ( portTickType ) 100 / portTICK_RATE_MS )

void pcf8575Task( void *pvParameters );
void set_led_state(unsigned short value);
void set_first8bits(unsigned char value);
void set_second8bits(unsigned char value);

#endif /* PCF8575_H_ */
