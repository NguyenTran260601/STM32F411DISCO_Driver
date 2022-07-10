/*
 * spi_senddata_main.c
 *
 *  Created on: Jun 19, 2022
 *      Author: Nguyen Tran
 */

/*
 *PB15 --> SPI2_MOSI
 *PB14 --> SPI2_MISO
 *PB13 --> SPI_SCKL
 *PB12 --> SPI_NSS
 *Alt SPI2 = 5
 */


#include "driver.h"

void SPI_GPIOInit(void)
{
    GPIO_Handle_t SPIPin;
    SPIPin.pGPIOx = GPIOB;
    SPIPin.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_ALTFUNC;
    SPIPin.GPIO_PinConfig.GPIO_PinAltFunMode    = 5;
    SPIPin.GPIO_PinConfig.GPIO_PinOPType        = GPIO_OP_TYPE_PP;
    SPIPin.GPIO_PinConfig.GPIO_PinPuPControl    = GPIO_NO_PUPD;

    //pin SCLK
    SPIPin.GPIO_PinConfig.GPIO_PinNumber        = GPIO_PIN_13;
    GPIO_Init(&SPIPin);

    //pin  MOSI
    SPIPin.GPIO_PinConfig.GPIO_PinNumber        = GPIO_PIN_15;
    GPIO_Init(&SPIPin);    
}

void SPI2_Init(void)
{
    SPI_Handle_t SPI2Handle;
    SPI2Handle.SPIConfig.SPI_BusConfig          = SPI_BUS_CONFIG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode         = SPI_DEVICE_MODE_MASTER;
    SPI2Handle.SPIConfig.SPI_SclkSpeed          = SPI_SCLK_SPEED_DIV2;  //8MHz
    SPI2Handle.SPIConfig.SPI_DFF                = SPI_DFF_8BIT;
    SPI2Handle.SPIConfig.SPI_CPOL               = SPI_CPOL_LOW;
    SPI2Handle.SPIConfig.SPI_CPHA               = SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM                = SPI_SSM_EN;   //software managerment
    SPI_Init(&SPI2Handle);
}

int main()
{
    char some_data[] = "This is progaram for SPI Senddata";
    SPI_GPIOInit();
    SPI2_Init();
    //enable SPI2

    SPI_PeripheralControl(SPI2, ENABLE);
    SPI_SendData(SPI2, (uint8_t*)some_data, 34);

    while (1);
}