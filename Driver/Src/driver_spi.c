/*
 * driver_spi.c
 *
 *  Created on: Jun 19, 2022
 *      Author: Nguyen Tran
 */
#include "driver_spi.h"

static void spi_txe_interupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interupt_handle(SPI_Handle_t *pSPIHandle);


/**************************Peripheral Clock Setup*******************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_PeriClockControl(SPI_Reg_t *pSPIx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE )
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLOCK_ENABLE();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLOCK_ENABLE();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLOCK_ENABLE();
			}else if(pSPIx == SPI4)
			{
				SPI4_PCLOCK_ENABLE();
			}else if(pSPIx == SPI5)
			{
				SPI5_PCLOCK_ENABLE();
			}
		}else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLOCK_DISABLE();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLOCK_DISABLE();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLOCK_DISABLE();
			}else if(pSPIx == SPI4)
			{
				SPI4_PCLOCK_DISABLE();
			}else if(pSPIx == SPI5)
			{
				SPI5_PCLOCK_DISABLE();
			}
		}
}


/* Init and De_init */

/************************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         - base address of the SPI handle
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//enable peripheral clock
	SPI_PeriClockControl(pSPIHandle ->pSPIx, ENABLE);

	/* config SPI_CR1 register */
	uint32_t temp = 0;

	//1.config device mode
	temp |= pSPIHandle -> SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2.config bus
	if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		temp |= (1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		temp |= (1 << SPI_CR1_RXONLY);
	}

	//3.config spi serial clock speed (baud rate)
	temp |= pSPIHandle -> SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.config DFF
	temp |= pSPIHandle -> SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5.config CPOL
	temp |= pSPIHandle -> SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6.config CPHA
	temp |= pSPIHandle -> SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. config SSM
	temp |= pSPIHandle -> SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle -> pSPIx -> SPI_CR1 = temp;
}


/*******************************************************************.
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function resets SPI peripheral
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_DeInit(SPI_Reg_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}else if(pSPIx == SPI5)
	{
		SPI5_REG_RESET();
	}
}


/*
 * Data send and receive
 */

/*******************************************************************.
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This is helper function get flag status
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - flag name
 * @param[in]         -
 *
 * @return            -  Flag set or reset
 *
 * @Note              -  none
 */
uint8_t SPI_GetFlagStatus(SPI_Reg_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx -> SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*******************************************************************.
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function transmits data
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - address of Tx buffer
 * @param[in]         - data length
 *
 * @return            -  none
 *
 * @Note              -  This is blocking call
 */
void SPI_SendData(SPI_Reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set (TX buffer is empty)
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the bit in CR1
		if( (pSPIx -> SPI_CR1 & (1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load data in to DR
			pSPIx -> SPI_DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx -> SPI_DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}




/*******************************************************************.
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function receives data
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - address of Tx buffer
 * @param[in]         - data length
 *
 * @return            -  none
 *
 * @Note              -  This is blocking call
 */
void SPI_ReceiveData(SPI_Reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXEN is set (Rx buffer is not empty)
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. check the bit in CR1
		if( (pSPIx -> SPI_CR1 & (1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load data from DR to Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx -> SPI_DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			*(pRxBuffer) = pSPIx -> SPI_DR;
			Len--;
			pRxBuffer++;
		}
	}
}


/*******************************************************************.
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - This function transmits data with interrupt
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - address of Tx buffer
 * @param[in]         - data length
 *
 * @return            -  state
 *
 * @Note              -  This is non blocking call
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle -> TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1. save Tx buffer address and len information in some global variables
		pSPIHandle -> pTxBuffer = pTxBuffer;
		pSPIHandle -> TxLen = Len;
		//2. mark the spi state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle -> TxState = SPI_BUSY_IN_TX;

		//3 enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle -> pSPIx -> SPI_CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

/*******************************************************************.
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - This function receives data
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - address of Tx buffer
 * @param[in]         - data length
 *
 * @return            -  state
 *
 * @Note              -  This is non blocking call
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle -> RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. save Tx buffer address and len information in some global variables
		pSPIHandle -> pRxBuffer = pRxBuffer;
		pSPIHandle -> RxLen = Len;
		//2. mark the spi state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle -> RxState = SPI_BUSY_IN_RX;

		//3 enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle -> pSPIx -> SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}




/*
 * IQR configuration and ISR handling
 */

/*******************************************************************.
 * @fn      		  - SPI_IRQITConfig
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
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}


/*******************************************************************.
 * @fn      		  - SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BIT_IMPLIMENT);

	*(NVIC_PR_BASEADDR + iprx) |=  (IRQPriority << shift_amount);

}


/*******************************************************************.
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function handles IRQ
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//check TXE
	temp1 = pSPIHandle -> pSPIx -> SPI_SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle -> pSPIx -> SPI_CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interupt_handle(pSPIHandle);
	}

	//check RXNE
	temp1 = pSPIHandle -> pSPIx -> SPI_SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle -> pSPIx -> SPI_CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interupt_handle(pSPIHandle);
	}

	//check OVR
	temp1 = pSPIHandle -> pSPIx -> SPI_SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle -> pSPIx -> SPI_CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle OVR
		spi_ovr_err_interupt_handle(pSPIHandle);
	}

}

/*
 *Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_Reg_t *pSPIx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		pSPIx -> SPI_CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx -> SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_Reg_t *pSPIx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		pSPIx -> SPI_CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx -> SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_Reg_t *pSPIx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		pSPIx -> SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx -> SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


static void spi_txe_interupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check DFF
	if( (pSPIHandle -> pSPIx -> SPI_CR1 & (1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load data in to DR
		pSPIHandle -> pSPIx -> SPI_DR = *((uint16_t*)pSPIHandle ->pTxBuffer);
		pSPIHandle -> TxLen--;
		pSPIHandle -> TxLen--;
		(uint16_t*)pSPIHandle ->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle -> pSPIx -> SPI_DR = *((uint16_t*)pSPIHandle ->pTxBuffer);
		pSPIHandle -> TxLen--;
		(uint16_t*)pSPIHandle ->pTxBuffer++;
	}

	if(! pSPIHandle ->TxLen)
	{
		//txlen = 0 close spi transmission and inform app
		//tx is over
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


static void spi_rxne_interupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check DFF
	if( (pSPIHandle -> pSPIx -> SPI_CR1 & (1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load data in to DR
		*((uint16_t*)pSPIHandle ->pRxBuffer) = (uint16_t)pSPIHandle -> pSPIx -> SPI_DR;
		pSPIHandle -> RxLen -= 2;
		pSPIHandle -> pRxBuffer--;
		pSPIHandle -> pRxBuffer--;

	}else
	{
		//8 bit DFF
		*((uint16_t*)pSPIHandle ->pRxBuffer) = (uint16_t)pSPIHandle -> pSPIx -> SPI_DR;
		pSPIHandle -> RxLen--;
		pSPIHandle -> pRxBuffer--;
	}

	if(! pSPIHandle ->RxLen)
	{
		//reception complete
		SPI_CloseRecception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


static void spi_ovr_err_interupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//clear ovr flag
	if(pSPIHandle ->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle -> pSPIx -> SPI_DR;
		temp = pSPIHandle -> pSPIx -> SPI_SR;
	}
	(void)temp;
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);

}


void SPI_ClearOVRFlag(SPI_Reg_t *pSPIx)
{
	uint8_t temp;
	temp =pSPIx -> SPI_DR;
	temp =pSPIx -> SPI_SR;
	(void)temp;

}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle -> pSPIx -> SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle -> pTxBuffer = NULL;
	pSPIHandle -> TxLen = 0;
	pSPIHandle -> TxState = SPI_READY;
}

void SPI_CloseRecception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle -> pSPIx -> SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle -> pRxBuffer = NULL;
	pSPIHandle -> RxLen = 0;
	pSPIHandle -> RxState = SPI_READY;
}

__attribute((weak)) void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t ApplicationEv)
{

}
