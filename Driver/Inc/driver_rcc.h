/*
 * driver_rcc.h
 *
 *  Created on: July 7, 2022
 *      Author: Nguyen Tran
 */


#ifndef DRIVER_RCC_H_
#define DRIVER_RCC_H_

#include "driver.h"



//APB1
uint32_t RCC_GetPCLK1Value(void);

//APB2
uint32_t RCC_GetPCLK2Value(void);

uint32_t  RCC_GetPLLOutputClock(void);


#endif /* DRIVER_RCC_H_ */
