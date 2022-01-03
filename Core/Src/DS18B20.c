/*
 * DS18B20.c
 *
 *  Created on: Dec 15, 2021
 *      Author: rlalu
 */
#include "main.h"

#define  DS18B20_PORT GPIOA
#define  DS18B20_PIN  GPIO_PIN_10


uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Mode = MODE_OUTPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // set the pin as output
	//Set_Pin_Output(GPIOC,GPIO_PIN_2, DS18B20_PIN);   // set the pin as output


	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay_us (480);   // delay according to datasheet

	GPIO_InitStruct.Mode = MODE_INPUT;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // set the pin as output
	//Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	delay_us (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay_us (400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Mode = MODE_OUTPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // set the pin as output
	//Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1
			GPIO_InitStruct.Mode = MODE_OUTPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // set the pin as output
		//	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay_us (1);  // wait for 1 us

			GPIO_InitStruct.Mode = MODE_INPUT;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // set the pin as output
			//Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay_us (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0
			GPIO_InitStruct.Mode = MODE_OUTPUT;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // set the pin as output
			//Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);

			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay_us (50);  // wait for 60 us

			GPIO_InitStruct.Mode = MODE_OUTPUT;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // set the pin as output
			//Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = MODE_OUTPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	for (int i=0;i<8;i++)
	{
		GPIO_InitStruct.Mode = MODE_OUTPUT;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // set the pin as output
		//Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set as output

		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the data pin LOW
		delay_us (1);  // wait for > 1us

		GPIO_InitStruct.Mode = MODE_INPUT;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // set the pin as output
		//Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay_us (50);  // wait for 60 us
	}
	return value;
}
