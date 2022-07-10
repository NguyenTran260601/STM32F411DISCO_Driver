/*
 * driver_gpio.c
 *
 *  Created on: Jun 16, 2022
 *      Author: Nguyen Tran
 */

#include "driver_gpio.h"


/**************************Peripheral Clock Setup*******************/
void GPIO_PeriClockControl(GPIO_Reg_t *pGPIOx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE )
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLOCK_ENABLE();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLOCK_ENABLE();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLOCK_ENABLE();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLOCK_ENABLE();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLOCK_ENABLE();
		}
	}else
	{
		//
	}
}

/****************************Init and DeInit************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    //enable peripheral clock control
    GPIO_PeriClockControl(pGPIOHandle ->pGPIOx, ENABLE);

	uint32_t temp = 0;
	//1. configure mode io pin
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non it mode
		temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle -> pGPIOx -> MODER &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clc
		pGPIOHandle -> pGPIOx -> MODER |= temp;//set
	}else
	{
		//it mode
		if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1 FTSR
			EXTI ->EXTI_FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			EXTI ->EXTI_RTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);//clc RTSR

		}else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1 RTSR
			EXTI ->EXTI_RTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			EXTI ->EXTI_FTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);//clc FTSR

		}else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1 FTSR & RTSR
			EXTI ->EXTI_RTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			EXTI ->EXTI_FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}

		//2 GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle -> pGPIOx);
		SYSCFG_PCLOCK_ENABLE();
		SYSCFG -> EXTICR[temp1] = portcode <<(temp2 * 4);

		//3. enable exti interupt delivery using IMR
		EXTI ->EXTI_IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}


	temp = 0;

	//2. configure speed
	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OSPEEDR &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clc
	pGPIOHandle -> pGPIOx -> OSPEEDR |= temp;

	temp = 0;
	//3. configure pupd setting
	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPControl << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> PUPDR &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clc
	pGPIOHandle -> pGPIOx -> PUPDR |= temp;

	temp = 0;
	//4. configure optype
	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OTYPER &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clc
	pGPIOHandle -> pGPIOx -> OTYPER |= temp;

	temp = 0;
	//5. configure alt func
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC)
	{
		//configure alt func
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber /8;
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber %8;
		pGPIOHandle -> pGPIOx -> AFR[temp1] &= ~(0xF << (4 * temp2)); //clc
		pGPIOHandle -> pGPIOx -> AFR[temp1] |= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

void GPIO_DeInit(GPIO_Reg_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/**************************Data Read and Write*******************/

uint8_t GPIO_ReadFromInputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t Value;
	Value = (uint8_t)((pGPIOx -> IDR >> PinNumber) & 0x00000001);
	return Value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_Reg_t *pGPIOx)
{
	uint16_t Value;
	Value = (uint16_t)pGPIOx -> IDR;
	return Value;
}
void GPIO_WriteToOutputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//w1
		pGPIOx -> ODR |= (1 << PinNumber);
	}else
	{
		//w0
		pGPIOx -> ODR &= ~(1 << PinNumber);
	}

}

void GPIO_WriteToOutputPort(GPIO_Reg_t *pGPIOx, uint16_t Value)
{
	pGPIOx -> ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx -> ODR ^= (1 << PinNumber);
}

/*******************************ISR*******************************/
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32) );

		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32) );

		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64) );
		}
	}
}

/***********************IQRPriorityConfig************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BIT_IMPLIMENT);

	*(NVIC_PR_BASEADDR + iprx) |=  (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear exti register
	if(EXTI -> EXTI_PR & (1 << PinNumber))
	{
		//clear
		EXTI -> EXTI_PR |= (1 << PinNumber);
	}
}

