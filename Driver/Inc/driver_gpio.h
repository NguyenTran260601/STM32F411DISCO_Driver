/*
 * driver_gpio.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Nguyen Tran
 */

#ifndef DRIVER_GPIO_H_
#define DRIVER_GPIO_H_

#include "driver.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_Pin_Config_t;

/*********************Handle structure GPIO PIN*********************/
typedef struct
{
	GPIO_Reg_t 				*pGPIOx;
	GPIO_Pin_Config_t 		GPIO_PinConfig;
}GPIO_Handle_t;

/**************************GPIO pin NUMBER***************************/
#define GPIO_PIN_0 			0
#define GPIO_PIN_1 			1
#define GPIO_PIN_2 			2
#define GPIO_PIN_3 			3
#define GPIO_PIN_4 			4
#define GPIO_PIN_5 			5
#define GPIO_PIN_6 			6
#define GPIO_PIN_7 			7
#define GPIO_PIN_8 			8
#define GPIO_PIN_9 			9
#define GPIO_PIN_10 		10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/**************************GPIO pin Mode***************************/
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTFUNC	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/**********************GPIO pin OUTPUT Type***********************/
#define GPIO_OP_TYPE_PP 	0
#define GPIO_OP_TYPE_OD 	1

/**********************GPIO pin OUTPUT Speed***********************/
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/**********************GPIO pin PUP AND PDOW***********************/
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2



/**************************Peripheral Clock Setup*******************/
void GPIO_PeriClockControl(GPIO_Reg_t *pGPIOx, uint8_t En_or_Di);

/****************************Init and DeInit************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Reg_t *pGPIOx);

/**************************Data Read and Write*******************/

uint8_t GPIO_ReadFromInputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_Reg_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber);

/*******************************ISR*******************************/
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t En_or_Di);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* DRIVER_GPIO_H_ */
