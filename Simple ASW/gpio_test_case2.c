/*
 * gpio_test_case2.c
 *
 *  Created on: Mar 6, 2023
 *      Author: Nguyen Tran
 */


#include "driver.h"

#define HIGH_STATE		1
#define BUTTON_PRESS    HIGH_STATE

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main()
{
    GPIO_Handle_t GpioLed, GpioButton;
    GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber		    = GPIO_PIN_15;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		    = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed 		    = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType		    = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPControl	    = GPIO_NO_PUPD;
    GPIO_Init(&GpioLed);

    GpioButton.pGPIOx = GPIOA;
    GpioButton.GPIO_PinConfig.GPIO_PinNumber        = GPIO_PIN_0;
    GpioButton.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_INPUT;
    GpioButton.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPControl	= GPIO_NO_PUPD;
    GPIO_Init(&GpioButton);

    while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) == BUTTON_PRESS)
			{
				delay();
				GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_15);
			}
	}
	return 0;
}
