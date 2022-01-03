/*
 * DS18B20.h
 *
 *  Created on: Dec 15, 2021
 *      Author: rlalu
 */

#ifndef SRC_DS18B20_H_
#define SRC_DS18B20_H_

uint8_t DS18B20_Start (void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);
extern void delay_us (uint16_t time);

#endif /* SRC_DS18B20_H_ */
