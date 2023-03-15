/*
 * spi_senddata_testcase1.c
 *
 *  Created on: Mar 8, 2023
 *      Author: Nguyen Tran
 */


#include "driver.h"
#include <string.h>

/*
 * SPI1 -> AF5
 * PA15 -> NSS
 * PB3 -> SCKL
 * PB4 -> MISO
 * PB5 -> MOSI
 */

void SPI1_GPIO_Init(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUNC;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCKL
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_Init(&SPIPins);
}

void SPI1_Init(void)
{
	SPI_Handle_t SPI1Handle;

	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //sw slave management enable

	SPI_Init(&SPI1Handle);
}

int main()
{
	char user_data[] = "Hello world";

	SPI1_GPIO_Init();

	SPI1_Init();

	//NSS internally high and avoid MODF
	SPI_SSIConfig(SPI1, ENABLE);

	//enable peripheral
	SPI_PeripheralControl(SPI1, ENABLE);

	SPI_SendData(SPI1,(uint8_t*)user_data,strlen(user_data));

	SPI_PeripheralControl(SPI1, DISABLE);

	while(1);

	return 0;
}
