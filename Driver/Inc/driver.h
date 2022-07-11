/*
 * driver.h
 *
 *  Created on: Jun 14, 2022
 *      Author: Nguyen Tran
 */

#ifndef DRIVER_H_
#define DRIVER_H_

#include <stdint.h>
#include <stddef.h>

/*************START: Processor Specific Detail**********************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register address
 */
#define NVIC_ISER0				((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1				((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2				((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3				((volatile uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register address
 */
#define NVIC_ICER0				((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1				((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2				((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3				((volatile uint32_t*)0xE000E18C)

/*
 * ARM Cortex Mx Processor priority register address
 */
#define NVIC_PR_BASEADDR		((volatile uint32_t*)0xE000E400)

#define NO_PR_BIT_IMPLIMENT		4


/*****************************SYSTEMS*******************************/
#define FLASH_BASEADDR              0x08000000U
#define SRAM                        0x20000000U
#define ROM                         0x1FFF0000U


/************Base address of peripheral hang on APB1****************/
#define TIM2_BASEADDR               0x40000000U
#define TIM3_BASEADDR               0x40000400U
#define TIM4_BASEADDR               0x40000800U
#define TIM5_BASEADDR               0x40000C00U
#define RTC_BKP_BASEADDR            0x40002800U
#define WWDG_BASEADDR               0x40002C00U
#define IWDG_BASEADDR               0x40003000U
#define I2S2EXT_BASEADDR            0x40003400U
#define SPI2_BASEADDR			    0x40003800U
#define SPI3_BASEADDR			    0x40003C00U
#define I2S3EXT_BASEADDR		    0x40004000U
#define USART2_BASEADDR			    0x40004400U
#define I2C1_BASEADDR			    0x40005400U
#define I2C2_BASEADDR			    0x40005800U
#define I2C3_BASEADDR			    0x40005C00U
#define PWR_BASEADDR			    0x40007000U

/************Base address of peripheral hang on APB2****************/
#define TIM1_BASEADDR			    0x40010000U
#define USART1_BASEADDR			    0x40011000U
#define USART6_BASEADDR			    0x40011400U
#define ADC_BASEADDR			    0x40012000U
#define SDIO_BASEADDR			    0x40012C00U
#define SPI1_BASEADDR			    0x40013000U
#define SPI4_BASEADDR			    0x40013400U
#define SYSCFG_BASEADDR			    0x40013800U
#define EXTI_BASEADDR			    0x40013C00U
#define TIM9_BASEADDR			    0x40014000U
#define TIM10_BASEADDR			    0x40014400U
#define TIM11_BASEADDR			    0x40014800U
#define SPI5_BASEADDR			    0x40015000U

/************Base address of peripheral hang on AHB1****************/
#define GPIOA_BASEADDR			    0x40020000U
#define GPIOB_BASEADDR			    0x40020400U
#define GPIOC_BASEADDR			    0x40020800U
#define GPIOD_BASEADDR			    0x40020C00U
#define GPIOE_BASEADDR			    0x40021000U
#define GPIOH_BASEADDR			    0x40021C00U
#define CRC_BASEADDR			    0x40023000U
#define RCC_BASEADDR			    0x40023800U
#define FLASH_INTER_BASEADDR	    0x40023C00U
#define DMA1_BASEADDR			    0x40026000U
#define DMA2_BASEADDR			    0x40026400U

/************Base address of peripheral hang on AHB2****************/
#define USB_OTG_FS_BASEADDR         0x50000000U

/*************************RCC_Register******************************/
typedef struct
{
	volatile uint32_t 	CR;					//0x00
	volatile uint32_t 	PLLCFGR;			//0x04
	volatile uint32_t 	CFGR;				//0x08
	volatile uint32_t 	CIR;				//0x0C
	volatile uint32_t 	AHB1RSTR;			//0x10
	volatile uint32_t 	AHB2RSTR;			//0x14
	volatile uint32_t 	AHB3RSTR;			//0x18
	uint32_t 			Reserved0;			//0x1C
	volatile uint32_t	APB1RSTR;			//0x20
	volatile uint32_t	APB2RSTR;			//0x24
	uint32_t 			Reserved1[2];		//0x28 - 0x2C
	volatile uint32_t	AHB1ENR;			//0x30
	volatile uint32_t	AHB2ENR;			//0x34
	volatile uint32_t	AHB3ENR;			//0x38
	uint32_t 			Reserved2;			//0x3C
	volatile uint32_t	APB1ENR;			//0x40
	volatile uint32_t	APB2ENR;			//0x44
	uint32_t 			Reserved3[2];		//0x48 - 0x4C
	volatile uint32_t	AHB1LPENR;			//0x50
	volatile uint32_t	AHB2LPENR;			//0x54
	volatile uint32_t	AHB3LPENR;			//0x58
	uint32_t 			Reserved4;			//0x5C
	volatile uint32_t	APB1LPENR;			//0x60
	volatile uint32_t	APB2LPENR;			//0x64
	uint32_t 			Reserved5[2];		//0x68 - 0x6C
	volatile uint32_t	BDCR;				//0x70
	volatile uint32_t	CSR;				//0x74
	uint32_t 			Reserved6[2];		//0x78 - 0x7C
	volatile uint32_t	SSCGR;				//0x80
	volatile uint32_t	PLLI2SCFGR;			//0x84
	volatile uint32_t	PLLSAICFGR;			//0x88
	volatile uint32_t	DCKCFGR;			//0x84C
}RCC_Reg_t;

/***************************EXTI_register***************************/
typedef struct
{
	volatile uint32_t	EXTI_IMR;			//0x00
	volatile uint32_t	EXTI_EMR;			//0x04
	volatile uint32_t	EXTI_RTSR;			//0x08
	volatile uint32_t	EXTI_FTSR;			//0x0C
	volatile uint32_t	EXTI_SWIER;			//0x10
	volatile uint32_t	EXTI_PR;			//0x14
}EXTI_Reg_t;

/***************************SYSCFG_register*************************/
typedef struct
{
	volatile uint32_t	MEMRMP;				//0x00
	volatile uint32_t	PCM;				//0x04
	volatile uint32_t	EXTICR[4];			//0x08 - 0x14
	uint32_t 			Reserved1[2];		//0x18 - 0x1C
	volatile uint32_t	CMPCR;				//0x20
	uint32_t 			Reserved2[2];		//0x24 -0x28
	volatile uint32_t 	CFGR;				//0x2C
}SYSCFG_Reg_t;


/************************GPIO_Register******************************/
typedef struct
{
	volatile uint32_t 	MODER;				//0x00
	volatile uint32_t 	OTYPER;				//0x04
	volatile uint32_t 	OSPEEDR;			//0x08
	volatile uint32_t 	PUPDR;				//0x0C
	volatile uint32_t 	IDR;				//0x10
	volatile uint32_t 	ODR;				//0x14
	volatile uint32_t 	BSRR;				//0x18
	volatile uint32_t 	LCKR;				//0x1C
	volatile uint32_t 	AFR[2];				//0x24 - 0x20
}GPIO_Reg_t;

/*************************SPI_Register******************************/
typedef struct 
{
	volatile uint32_t	SPI_CR1;			//0x00
	volatile uint32_t	SPI_CR2;			//0x04
	volatile uint32_t	SPI_SR;				//0x08
	volatile uint32_t 	SPI_DR;				//0x0C
	volatile uint32_t	SPI_CRCPR;			//0x10
	volatile uint32_t	SPI_RXCRCR;			//0x14
	volatile uint32_t	SPI_TXCRCR;			//0x18
	volatile uint32_t	SPI_I2SCFGR;		//0x1C
	volatile uint32_t	SPI_I2SPR;			//0x20
}SPI_Reg_t;

/*************************I2C_Register******************************/
typedef struct
{
	volatile uint32_t	I2C_CR1;			//0x00
	volatile uint32_t	I2C_CR2;			//0x04
	volatile uint32_t	I2C_OAR1;			//0x08
	volatile uint32_t	I2C_OAR2;			//0x0C
	volatile uint32_t	I2C_DR;				//0x10
	volatile uint32_t	I2C_SR1;			//0x14
	volatile uint32_t	I2C_SR2;			//0x18
	volatile uint32_t	I2C_CCR;			//0x1C
	volatile uint32_t	I2C_TRISE;			//0x20
	volatile uint32_t	I2C_FLTR;			//0x24
}I2C_Reg_t;

/*****************************USART_register*************************/
typedef struct
{
	volatile uint32_t USART_SR;						//0x00
	volatile uint32_t USART_DR;						//0x04
	volatile uint32_t USART_BRR;					//0x08
	volatile uint32_t USART_CR1;					//0x0C
	volatile uint32_t USART_CR2;					//0x10
	volatile uint32_t USART_CR3;					//0x14
	volatile uint32_t USART_GTPR;					//0x18
}USART_Reg_t;


/**********Peripheral BASE ADDRESS TYPECASTTED TO xxx_Reg_t*********/
#define RCC			((RCC_Reg_t*) RCC_BASEADDR)

#define EXTI		((EXTI_Reg_t*) EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_Reg_t*) SYSCFG_BASEADDR)

#define GPIOA		((GPIO_Reg_t*) GPIOA_BASEADDR)
#define GPIOB		((GPIO_Reg_t*) GPIOB_BASEADDR)
#define GPIOC		((GPIO_Reg_t*) GPIOC_BASEADDR)
#define GPIOD		((GPIO_Reg_t*) GPIOD_BASEADDR)
#define GPIOE		((GPIO_Reg_t*) GPIOE_BASEADDR)
#define GPIOH		((GPIO_Reg_t*) GPIOH_BASEADDR)

#define SPI1		((SPI_Reg_t*) SPI1_BASEADDR)
#define SPI2		((SPI_Reg_t*) SPI2_BASEADDR)
#define SPI3		((SPI_Reg_t*) SPI3_BASEADDR)
#define SPI4		((SPI_Reg_t*) SPI4_BASEADDR)
#define SPI5		((SPI_Reg_t*) SPI5_BASEADDR)

#define I2C1		((I2C_Reg_t*) I2C1_BASEADDR)
#define I2C2		((I2C_Reg_t*) I2C2_BASEADDR)
#define I2C3		((I2C_Reg_t*) I2C3_BASEADDR)

#define USART1		((USART_Reg_t*) USART1_BASEADDR)
#define USART2		((USART_Reg_t*) USART2_BASEADDR)
#define USART6		((USART_Reg_t*) USART6_BASEADDR)

/************************CLOCK ENABLE GPIOx*************************/
#define GPIOA_PCLOCK_ENABLE()  	(RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PCLOCK_ENABLE()  	(RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PCLOCK_ENABLE()  	(RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_PCLOCK_ENABLE()  	(RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PCLOCK_ENABLE()  	(RCC -> AHB1ENR |= (1 << 4))
#define GPIOH_PCLOCK_ENABLE()  	(RCC -> AHB1ENR |= (1 << 7))

/*************************CLOCK ENABLE SPIx**************************/
#define SPI1_PCLOCK_ENABLE()	(RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLOCK_ENABLE()	(RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLOCK_ENABLE()	(RCC -> APB1ENR |= (1 << 15))
#define SPI4_PCLOCK_ENABLE()	(RCC -> APB2ENR |= (1 << 13))
#define SPI5_PCLOCK_ENABLE()	(RCC -> APB2ENR |= (1 << 20))

/*************************CLOCK ENABLE I2Cx**************************/
#define I2C1_PCLOCK_ENABLE()	(RCC -> APB1ENR |= (1 << 21))
#define I2C2_PCLOCK_ENABLE()	(RCC -> APB1ENR |= (1 << 22))
#define I2C3_PCLOCK_ENABLE()	(RCC -> APB1ENR |= (1 << 23))

/************************CLOCK ENABLE USARTx*************************/
#define USART1_PCLOCK_ENABLE()	(RCC -> APB2ENR |= (1 << 4))
#define USART2_PCLOCK_ENABLE()	(RCC -> APB1ENR |= (1 << 17))
#define USART6_PCLOCK_ENABLE()	(RCC -> APB2ENR |= (1 << 5))

/**********************CLOCK ENABLE SYSCFG**************************/
#define SYSCFG_PCLOCK_ENABLE()	(RCC -> APB2ENR |= (1 << 14))

/************************CLOCK DISABLE GPIOx************************/
#define GPIOA_PCLOCK_DISABLE()  (RCC -> AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLOCK_DISABLE()  (RCC -> AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLOCK_DISABLE()  (RCC -> AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLOCK_DISABLE()  (RCC -> AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLOCK_DISABLE()  (RCC -> AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLOCK_DISABLE()  (RCC -> AHB1ENR &= ~(1 << 7))

/*************************CLOCK DISABLE SPIx************************/
#define SPI1_PCLOCK_DISABLE()	(RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLOCK_DISABLE()	(RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_PCLOCK_DISABLE()	(RCC -> APB1ENR &= ~(1 << 15))
#define SPI4_PCLOCK_DISABLE()	(RCC -> APB2ENR &= ~(1 << 13))
#define SPI5_PCLOCK_DISABLE()	(RCC -> APB2ENR &= ~(1 << 20))

/*************************CLOCK DISABLE I2Cx************************/
#define I2C1_PCLOCK_DISABLE()	(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PCLOCK_DISABLE()	(RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_PCLOCK_DISABLE()	(RCC -> APB1ENR &= ~(1 << 23))

/************************CLOCK DISABLE USARTx************************/
#define USART1_PCLOCK_DISABLE()	(RCC -> APB2ENR &= ~(1 << 4))
#define USART2_PCLOCK_DISABLE()	(RCC -> APB1ENR &= ~(1 << 17))
#define USART6_PCLOCK_DISABLE()	(RCC -> APB2ENR &= ~(1 << 5))

/*******************************RESET GPIO**************************/
#define GPIOA_REG_RESET() 		do{ (RCC -> AHB1RSTR |= (1 << 0)); (RCC -> AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET() 		do{ (RCC -> AHB1RSTR |= (1 << 1)); (RCC -> AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET() 		do{ (RCC -> AHB1RSTR |= (1 << 2)); (RCC -> AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET() 		do{ (RCC -> AHB1RSTR |= (1 << 3)); (RCC -> AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET() 		do{ (RCC -> AHB1RSTR |= (1 << 4)); (RCC -> AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET() 		do{ (RCC -> AHB1RSTR |= (1 << 7)); (RCC -> AHB1RSTR &= ~(1 << 5)); }while(0)

/*************Return port code for GPIOx base address***************/
#define GPIO_BASEADDR_TO_CODE(x)	   ((x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
										(x == GPIOE)?4:\
										(x == GPIOH)?7:0)		


/**********************************IRQ******************************/
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85


#define IRQ_NO_I2C1_EV     	31
#define IRQ_NO_I2C1_ER     	32
#define IRQ_NO_I2C2_EV     	33
#define IRQ_NO_I2C2_ER     	34
#define IRQ_NO_I2C3_EV     	72
#define IRQ_NO_I2C3_ER     	73

#define IRQ_NO_USART1	    37
#define IRQ_NO_USART3	    39
#define IRQ_NO_USART6	    71

/*******************Some macro possible priority********************/
#define NVIC_IRQ_PR0		0
#define NVIC_IRQ_PR1		1
#define NVIC_IRQ_PR2		2
#define NVIC_IRQ_PR3		3
#define NVIC_IRQ_PR4		4
#define NVIC_IRQ_PR5		5
#define NVIC_IRQ_PR6		6
#define NVIC_IRQ_PR7		7
#define NVIC_IRQ_PR8		8
#define NVIC_IRQ_PR9		9
#define NVIC_IRQ_PR10		10
#define NVIC_IRQ_PR11		11
#define NVIC_IRQ_PR12		12
#define NVIC_IRQ_PR13		13
#define NVIC_IRQ_PR14		14
#define NVIC_IRQ_PR15		15


/*
 *Macro for some SPI_Register
 */

/*********************some macro SPI_SR register********************/
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*****************Some macro SPI_CR2 register***********************/
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_Res			3
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/****************Some macro SPI_SR register*************************/
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSEDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/*
 *Macro for some I2C_Register
 */

/****************Some macro I2C_CR1 register*************************/
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

/****************Some macro I2C_CR2 register*************************/
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN 	10

/****************Some macro I2C_SR1 register*************************/
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_TIMEOUT		14

/****************Some macro I2C_SR2 register*************************/
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7


/*
 *Macro for some USART_Register
 */
/****************Some macro USART_SR register************************/
#define USART_SR_PE						0
#define USART_SR_FE						1
#define USART_SR_NF						2
#define USART_SR_ORE					3
#define USART_SR_IDLE					4
#define USART_SR_RXNE					5
#define USART_SR_TC						6
#define USART_SR_TXE					7
#define USART_SR_LBD					8
#define USART_SR_CTS					9

/****************Some macro USART_CR1 register***********************/
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/****************Some macro USART_CR2 register***********************/
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/****************Some macro USART_CR3 register***********************/
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/****************************Some generic****************************/
#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

#include "driver_gpio.h"
#include "driver_spi.h"
#include "driver_i2c.h"
#include "driver_rcc.h"
#include "driver_uart.h"

#endif /* DRIVER_H_ */
