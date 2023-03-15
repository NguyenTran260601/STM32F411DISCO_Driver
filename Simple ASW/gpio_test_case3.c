/*
 * gpio_test_case3.c
 *
 *  Created on: Mar 6, 2023
 *      Author: Nguyen Tran
 */


#include "driver.h"

#define LOW_STATE		0
#define BUTTON_PRESS    LOW_STATE

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main()
{
    GPIO_Handle_t GpioLed, GpioButton;
    GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber		    = GPIO_PIN_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		    = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed 		    = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType		    = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPControl	    = GPIO_NO_PUPD;
    GPIO_Init(&GpioLed);

    GpioButton.pGPIOx = GPIOB;
    GpioButton.GPIO_PinConfig.GPIO_PinNumber        = GPIO_PIN_12;
    GpioButton.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_INPUT;
    GpioButton.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPControl	= GPIO_PIN_PU;
    GPIO_Init(&GpioButton);

    while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_12) == BUTTON_PRESS)
			{
				delay();
				GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_8);
			}
	}
	return 0;
}
