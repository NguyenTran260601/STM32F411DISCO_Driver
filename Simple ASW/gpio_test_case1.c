/*
 * gpio_test_case1.c
 *
 *  Created on: Mar 6, 2023
 *      Author: Nguyen Tran
 */


#include "driver.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}



int main(void)
{
    GPIO_Handle_t GpioLed;
    GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber		    = GPIO_PIN_15;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		    = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed 		    = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType		    = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPControl	    = GPIO_NO_PUPD;
    GPIO_Init(&GpioLed);

    /* Loop forever */
    while(1)
    {
    	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_15);
    	delay();

    }
    return 0;

}
