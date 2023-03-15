/*
 * driver_spi.h
 *
 *  Created on: Jun 19, 2022
 *      Author: Nguyen Tran
 */

#ifndef DRIVER_SPI_H_
#define DRIVER_SPI_H_

#include "driver.h"
/*
 * Configure structure for SPI peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;		/*!< 	possible values from @SPI_DeviceMode 		>*/
	uint8_t SPI_BusConfig;		/*!< 	possible values from @SPI_BusConfig 		>*/
	uint8_t SPI_SclkSpeed;		/*!< 	possible values from @SPI_SclkSpeed 		>*/
	uint8_t SPI_DFF;			/*!< 	possible values from @SPI_DFF 				>*/
	uint8_t SPI_CPOL;			/*!< 	possible values from @SPI_CPOL 				>*/
	uint8_t SPI_CPHA;			/*!< 	possible values from @SPI_CPHA 				>*/
	uint8_t SPI_SSM;			/*!< 	possible values from @SPI_SSM 				>*/
}SPI_Config_t;


/*
 * Handle structure for SPI peripheral
 */
typedef struct
{
	SPI_Reg_t		*pSPIx;
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}SPI_Handle_t;


/*
 * SPI application state
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/*
 *Apllication event
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DIVECE_MODE_SLAVE	0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BIT	0
#define SPI_DFF_16BIT	1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN		1 //hw
#define SPI_SSM_DI		0 //sw


/*
 * SPI relate status flags definitions
 */
#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)




/********************************************
 * APIs SUPPORTED FOR THIS DRIVER           *
 ********************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_Reg_t *pSPIx, uint8_t En_or_Di);

/*
 * Init and De_init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Reg_t *pSPIx);

/*
 * Data send and receive
 */
uint8_t SPI_GetFlagStatus(SPI_Reg_t *pSPIx, uint32_t FlagName);
void SPI_SendData(SPI_Reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_Reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IQR configuration and ISR handling
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t En_or_Di);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 *Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_Reg_t *pSPIx, uint8_t En_or_Di);
void SPI_SSIConfig(SPI_Reg_t *pSPIx, uint8_t En_or_Di);
void SPI_SSOEConfig(SPI_Reg_t *pSPIx, uint8_t En_or_Di);
void SPI_ClearOVRFlag(SPI_Reg_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseRecception(SPI_Handle_t *pSPIHandle);

/*
 * application callback
 */
void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t ApplicationEv);



#endif /* DRIVER_SPI_H_ */
