/*
 * driver_gpio.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Nguyen Tran
 */

#ifndef DRIVER_GPIO_H_
#define DRIVER_GPIO_H_

#include "driver.h"

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/*!< 	possible values from @GPIO_PIN_NUMBERS 		>*/
	uint8_t GPIO_PinMode;			/*!< 	possible values from @GPIO_PIN_MODES 		>*/
	uint8_t GPIO_PinSpeed;			/*!< 	possible values from @GPIO_PIN_SPEED 		>*/
	uint8_t GPIO_PinPuPControl;		/*!< 	possible values from @GPIO_PUPD_CONTROL 	>*/
	uint8_t GPIO_PinOPType;			/*!< 	possible values from @GPIO_OP_TYPES 		>*/
	uint8_t GPIO_PinAltFunMode;		/*!< 	possible values from @ 1 to 15 				>*/
}GPIO_Pin_Config_t;

/*********************Handle structure GPIO PIN*********************/
typedef struct
{
	GPIO_Reg_t 				*pGPIOx;
	GPIO_Pin_Config_t 		GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
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

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTFUNC	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_OP_TYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP 	0
#define GPIO_OP_TYPE_OD 	1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PUPD_CONTROL
 * GPIO pin pull up AND pull down configuration macros
 */
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* DRIVER_GPIO_H_ */
