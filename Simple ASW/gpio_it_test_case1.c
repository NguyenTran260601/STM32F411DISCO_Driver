/*
 * gpio_it_test_case1.c
 *
 *  Created on: Mar 6, 2023
 *      Author: Nguyen Tran
 */

#include "driver.h"
#include<string.h>


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main()
{
    GPIO_Handle_t GpioLed, GpioButton;

    GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber		    = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		    = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed 		    = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType		    = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPControl	    = GPIO_NO_PUPD;
    GPIO_Init(&GpioLed);

    GpioButton.pGPIOx = GPIOA;
    GpioButton.GPIO_PinConfig.GPIO_PinNumber        = GPIO_PIN_0;
    GpioButton.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_IT_RT;
    GpioButton.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPControl	= GPIO_NO_PUPD;
    GPIO_Init(&GpioButton);


	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PR15);
	GPIO_IRQITConfig(IRQ_NO_EXTI0, ENABLE);

    while(1);
}



void EXTI0_IRQHandler(void)
{
    //delay(); //wait till button de-bouncing gets over
	GPIO_IRQHandling(GPIO_PIN_0); //clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_12);
}
