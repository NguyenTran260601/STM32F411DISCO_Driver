/*
 * driver_i2c.c
 *
 *  Created on: July 7, 2022
 *      Author: Nguyen Tran
 */

#include "driver_i2c.h"

static void I2C_GenerateStartCondition(I2C_Reg_t *pI2Cx);
static void I2C_ExcuteAddressPhaseWrite(I2C_Reg_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExcuteAddressPhaseRead(I2C_Reg_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);


static void I2C_GenerateStartCondition(I2C_Reg_t *pI2Cx)
{
	pI2Cx ->I2C_CR1 |= (1 << I2C_CR1_START);
}


static void I2C_ExcuteAddressPhaseWrite(I2C_Reg_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx ->I2C_DR = SlaveAddr;
}

static void I2C_ExcuteAddressPhaseRead(I2C_Reg_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= ~(1);
	pI2Cx ->I2C_DR = SlaveAddr;
}


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// disable ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// clear ADDR flag
				dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
				dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
				(void)dummy_read;
			}
		}else
		{
			// clear ADDR flag
			dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
			dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummy_read;
		}

	}else
	{
		//device is in slave mode
		// clear ADDR flag
		dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
		dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
		(void)dummy_read;
	}
}

void I2C_GenerateStopCondition(I2C_Reg_t *pI2Cx)
{
	pI2Cx ->I2C_CR1 |= (1 << I2C_CR1_STOP);
}


void I2C_SlaveEnableDisableCallbackEvents(I2C_Reg_t *pI2Cx,uint8_t En_or_Di)
{
	 if(En_or_Di == ENABLE)
	 {
			pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
			pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);
			pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
	 }else
	 {
			pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
			pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
			pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITERREN);
	 }

}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_SlaveSendData(I2C_Reg_t *pI2C,uint8_t data)
{
	pI2C->I2C_DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_Reg_t *pI2C)
{
    return (uint8_t) pI2C->I2C_DR;
}




/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_Reg_t *pI2Cx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE )
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLOCK_ENABLE();
			}
			else if(pI2Cx == I2C2)
			{
				I2C2_PCLOCK_ENABLE();
			}else if(pI2Cx == I2C3)
			{
				I2C3_PCLOCK_ENABLE();
			}
		}else
		{
			//
		}
}


/*
 * Init and De_init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp = 0;

	I2C_PeriClockControl(pI2CHandle -> pI2Cx, ENABLE);
	//ack
	temp |= pI2CHandle-> I2C_Config.I2C_ACKControl << 10;
	pI2CHandle -> pI2Cx ->I2C_CR1 = temp;

	//FREQ of CR2
	temp = 0;
	temp |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle -> pI2Cx -> I2C_CR2 = (temp & 0x3F);

	//program the device own address
	temp |= pI2CHandle ->I2C_Config.I2C_DeviceAddress << 1;
	temp |= (1 << 14);
	pI2CHandle -> pI2Cx -> I2C_OAR1 = temp;

	//CCR
	uint16_t ccr_val = 0;
	temp = 0;
	if(pI2CHandle ->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_val = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		temp |= (ccr_val & 0xFFF);
	}else
	{
		//mode is fast mode
		temp |= (1 << 15);
		temp |=	(pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_val = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else
		{
			ccr_val = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		temp |= (ccr_val & 0xFFF);
	}
	pI2CHandle ->pI2Cx ->I2C_CCR = temp;

	//TRISE
	if(pI2CHandle ->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		temp = (RCC_GetPCLK1Value() /1000000U) +1;
	}else
	{
		//mode is fast mode
		temp = ((RCC_GetPCLK1Value() * 300)/ 100000000U) + 1;
	}

	pI2CHandle ->pI2Cx ->I2C_TRISE = (temp & 0x3F);

}


void I2C_DeInit(I2C_Reg_t *pI2Cx)
{
    if(pI2Cx == I2C1)
    {
        I2C1_PCLOCK_DISABLE();
    }
    else if(pI2Cx == I2C2)
    {
        I2C2_PCLOCK_DISABLE();
    }else if(pI2Cx == I2C3)
    {
        I2C3_PCLOCK_DISABLE();
    }
}

/*
 * Data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1 generate the start condition
	I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

	//2 confirm that start generation is complete by checking the SB flag in SR1
	// SB clear SCL stretched (Low)
	while(! I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_SB));

	//3 send the address of slave with r/w bit set to w(0)
	I2C_ExcuteAddressPhaseWrite(pI2CHandle ->pI2Cx, SlaveAddr);

	//4 confirm that address phase is complete by checking the ADDR flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_ADDR));

	//5 clear ADDR
	I2C_ClearADDRFlag(pI2CHandle);

	//6 send the data until len becomes 0

	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TXE)); //wait TxE set
		pI2CHandle -> pI2Cx -> I2C_DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7 TxE =1 and BTF = 1 before generating the STOP condition
	//TxE =1 and BTF = 1 -> SR and DR empty
	//BTF = 1 SCL stretched
	while(! I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_BTF));


	//8 stop condition
	I2C_GenerateStopCondition(pI2CHandle ->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1 generate the start condition
	I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

	//2 confirm that start generation is complete by checking the SB flag in SR1
	// SB clear SCL stretched (Low)
	while(! I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_SB));

	//3 send the address of slave with r/w bit set to R(1)
	I2C_ExcuteAddressPhaseRead(pI2CHandle ->pI2Cx, SlaveAddr);

	//4 confirm that address phase is complete by checking the ADDR flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle ->pI2Cx, I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//disable ack
		I2C_ManageAcking(pI2CHandle ->pI2Cx, I2C_ACK_DISABLE);

		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_RXNE));

		//generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle ->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2CHandle ->pI2Cx ->I2C_DR;

		return;
	}

	//procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len become zero
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait until RXNE become 1
			while(! I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) //2bytes are remaining
			{
				//disable ack
				I2C_ManageAcking(pI2CHandle ->pI2Cx, I2C_ACK_DISABLE);
				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle ->pI2Cx);
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle ->pI2Cx ->I2C_DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}

}




void I2C_ManageAcking(I2C_Reg_t *pI2Cx, uint8_t En_Or_Di)
{
	if(En_Or_Di == I2C_ACK_ENABLE)
	{
		//enable
		pI2Cx ->I2C_CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
		pI2Cx ->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
	}

}


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle -> TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle -> pTxBuffer = pTxBuffer;
		pI2CHandle -> TxLen = Len;
		pI2CHandle -> TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle -> DevAddr = SlaveAddr;
		pI2CHandle -> Sr = Sr;

		// generate the start condition
		I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

		// enable ITBUFEN
		pI2CHandle ->pI2Cx -> I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

		// enable ITEVFEN
		pI2CHandle ->pI2Cx -> I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		// enable ITERREN
		pI2CHandle ->pI2Cx -> I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;

}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


		// enable ITBUFEN
		pI2CHandle ->pI2Cx -> I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

		// enable ITEVFEN
		pI2CHandle ->pI2Cx -> I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		// enable ITERREN
		pI2CHandle ->pI2Cx -> I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;

}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle -> pI2Cx ->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle -> pI2Cx ->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle -> pI2Cx -> I2C_SR1 & (1 << I2C_SR1_SB);

	//1. Handle For interrupt generated by SB event
	if(temp1 && temp3)
	{
		//SB flag is set

		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle ->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExcuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if (pI2CHandle ->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExcuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle -> pI2Cx -> I2C_SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	if(temp1 && temp3)
	{
		// ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle -> pI2Cx -> I2C_SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		// BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//TEX is also set
			if(pI2CHandle -> pI2Cx -> I2C_SR1 & (1 << I2C_SR1_TXE))
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0)
				{
					//1 generate stop condition
					if(pI2CHandle ->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
					}
					//2 reset all the member elements of the handler structure
					I2C_CloseSendData(pI2CHandle);

					//3 notify the application about transmission complete
					I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	temp3 = pI2CHandle -> pI2Cx -> I2C_SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	if(temp1 && temp3)
	{
		// STOPF flag is set
		//clear STOPF
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;

		//notify application the Stop is detected
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle -> pI2Cx -> I2C_SR1 & (1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//check for device mode
		if(pI2CHandle -> pI2Cx -> I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			// TXE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle ->TxLen > 0)
				{
					//load the data in DR
					pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);
					// decrement txlen
					pI2CHandle->TxLen--;
					// increment buffer address
					pI2CHandle->pTxBuffer++;
				}
			}
		}else
		{
			//slave
			//make sure that slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}

	}


	temp3 = pI2CHandle -> pI2Cx -> I2C_SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check device mode
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			//the device is master
			// RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxSize == 1)
				{
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
					pI2CHandle->RxLen--;
				}

				if(pI2CHandle->RxSize > 1)
				{
					if(pI2CHandle->RxLen == 2)
					{
						//clear ack bit
						I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
					}

					//read DR
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
					pI2CHandle ->pRxBuffer++;
					pI2CHandle ->RxLen--;
				}

				if(pI2CHandle->RxLen == 0)
				{
					//close the I2C data reception and notify the application
					//1 generate stop condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					//2 close the I2C rx
					I2C_CloseReceiveData(pI2CHandle);

					//3	notify application the Stop is detected
					I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_RX_CMPLT);
				}
			}
		}else
		{
			//slave
			//make sure that slave is really in receive mode
			if(!(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)))
			{
				//slave
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}



void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;
    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);

	//check for bus error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1 && temp2 )
	{
		//This is Bus error
		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_BERR);
	}

	// Check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1 && temp2)
	{
		//This is arbitration lost error
		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_ARLO);

	}

	//check for ACK failure  error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1 && temp2)
	{
		//This is ACK failure error
		//Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_AF);
	}

	// check for Overrun/underrun error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1 && temp2)
	{
		//This is Overrun/underrun
		//Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_OVR);
	}

	//check for time out error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2)
	{
		//This is Time out error
	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}

/*
 * IQR configuration and ISR handling
 */
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t En_or_Di)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BIT_IMPLIMENT);

	*(NVIC_PR_BASEADDR + iprx) |=  (IRQPriority << shift_amount);
}

/*
 *Other Peripheral Control APIs
 */
uint8_t I2C_GetFlagStatus(I2C_Reg_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx ->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_PeripheralControl(I2C_Reg_t *pI2Cx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);

	}else
	{
		pI2Cx->I2C_CR1 &= ~(1 << 0);
	}
}

__attribute((weak)) void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t ApplicationEv)
{

}
