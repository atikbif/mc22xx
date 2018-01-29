#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>

uint16_t GetCRC16 (uint8_t* ptr,uint16_t cnt);
uint8_t CheckLRC(uint8_t* ptr,uint16_t lng);
uint8_t getLRC(uint8_t* ptr,uint16_t lng);

#endif /* CRC_H_ */