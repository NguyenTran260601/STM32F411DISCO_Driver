/*
 * driver_gpio.c
 *
 *  Created on: Jun 16, 2022
 *      Author: Nguyen Tran
 */

#include "driver_gpio.h"


/**************************Peripheral Clock Setup*******************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
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
		if(En_or_Di == DISABLE )
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLOCK_DISABLE();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLOCK_DISABLE();
			}else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLOCK_DISABLE();
			}else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLOCK_DISABLE();
			}else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLOCK_DISABLE();
			}else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLOCK_DISABLE();
			}		
		}
	}
}

/****************************Init and DeInit************************/

/*
 * @fn      		  - GPIO_Init
 *
 * @brief             - 
 *
 * @param[in]         - base address of the gpio handle
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
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
		uint8_t temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber /4; //array
		uint8_t temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber %4; //position
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
	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle -> pGPIOx -> OTYPER &= ~(0x1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); //clc
	pGPIOHandle -> pGPIOx -> OTYPER |= temp;

	temp = 0;
	//5. configure alt func
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC)
	{
		//configure alt func
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber /8; //low and high AFR
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber %8; //position
		pGPIOHandle -> pGPIOx -> AFR[temp1] &= ~(0xF << (4 * temp2)); //clc
		pGPIOHandle -> pGPIOx -> AFR[temp1] |= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}


/*******************************************************************.
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function resets gpio peripheral
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
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


/*******************************************************************.
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads from pin gpio 
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - Pin number
 * @param[in]         -
 *
 * @return            - Value of PinNumber
 *
 * @Note              - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t Value;
	Value = (uint8_t)((pGPIOx -> IDR >> PinNumber) & 0x00000001);
	return Value;
}



/*******************************************************************.
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads from port gpio
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            - Value of Port
 *
 * @Note              - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_t *pGPIOx)
{
	uint16_t Value;
	Value = (uint16_t)pGPIOx -> IDR;
	return Value;
}


/*******************************************************************.
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes to pin gpio
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - Pin number
 * @param[in]         - Value
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1
		pGPIOx -> ODR |= (1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx -> ODR &= ~(1 << PinNumber);
	}

}


/*******************************************************************.
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes to port gpio
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - Value
 * @param[in]         - 
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPort(GPIO_Reg_t *pGPIOx, uint16_t Value)
{
	pGPIOx -> ODR = Value;
}



/*******************************************************************.
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles pin gpio
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - Pin number
 * @param[in]         - 
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_ToggleOutputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx -> ODR ^= (1 << PinNumber);
}



/*******************************ISR*******************************/

/*******************************************************************.
 * @fn      		  - GPIO_IRQITConfig
 *
 * @brief             - This function enables IRQ number
 *
 * @param[in]         - IRQ number
 * @param[in]         - ENABLE or DISABLE
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
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

/*******************************************************************.
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - This function configures IRQ priority
 *
 * @param[in]         - IRQ number
 * @param[in]         - IRQPriority
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BIT_IMPLIMENT);

	//set
	*(NVIC_PR_BASEADDR + iprx) |=  (IRQPriority << shift_amount);
}


/*******************************************************************.
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function handles IRQ
 *
 * @param[in]         - PinNumber
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear exti register
	if(EXTI -> EXTI_PR & (1 << PinNumber))
	{
		//clear
		EXTI -> EXTI_PR |= (1 << PinNumber);
	}
}
