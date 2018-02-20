#ifndef AIN_H_INCLUDED
#define AIN_H_INCLUDED

#define AIN_TASK_PRIORITY                  ( tskIDLE_PRIORITY + 1 )
#define AIN_DELAY						   ( ( portTickType ) 10 / portTICK_RATE_MS )

void ain16Task( void *pvParameters );
void ain8Task( void *pvParameters );

#endif /* AIN_H_INCLUDED */
