/*
 * gpio_IT_main.c
 *
 *  Created on: Jun 16, 2022
 *      Author: Nguyen Tran
 */

#include<string.h>
#include "driver.h"

void delay(void)
{
	// this will introduce ~200ms delay when system clock is 16MHz
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioButton;
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioButton, 0, sizeof(GpioLed));

    //configled
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_15;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPControl	= GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);

	//configbutton
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode 			= GPIO_MODE_IT_FT; //mode it
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPControl	= GPIO_PIN_PU;
	GPIO_Init(&GpioButton);

	GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
	//IQRconfigure
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PR15);
	GPIO_IRQITConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);
}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_15);
}
