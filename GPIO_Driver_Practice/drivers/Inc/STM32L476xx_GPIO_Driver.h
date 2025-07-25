/*
 * STM32L476xx_GPIO_Driver.h
 *
 *  Created on: Jul 25, 2025
 *      Author: Ray Dai
 */

#ifndef INC_STM32L476XX_GPIO_DRIVER_H_
#define INC_STM32L476XX_GPIO_DRIVER_H_

#include "STM32L476xx.h"


//---------------------------------------------Start of Written Code--------------------------------------

typedef struct {
	uint8_t GPIO_PinNum;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinOutType;
	uint8_t GPIO_PinOutSpd;
	uint8_t GPIO_PinPupd;
	uint8_t GPIO_PinAltFunc;
} GPIO_PinConfig_t;

typedef struct {
	GPIOx_RegDef_t* pGPIOx_Base;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_PinHandle_t;

void GPIO_Init(GPIO_PinHandle_t* pGPIO_PinHandle);
void GPIO_DeInit(GPIOx_RegDef_t* pGPIOx);
void GPIO_Pclk_Ctrl(GPIOx_RegDef_t* pGPIOx, uint8_t ENorDI);

uint8_t GPIO_ReadPin(GPIOx_RegDef_t* pGPIOx, uint8_t pinNum);
uint16_t GPIO_ReadPort(GPIOx_RegDef_t* pGPIOx);
void GPIO_WritePin(GPIOx_RegDef_t* pGPIOx, uint8_t pinNum, uint8_t value);
void GPIO_WritePort(GPIOx_RegDef_t* pGPIOx, uint8_t value);
void GPIO_TogglePin(GPIOx_RegDef_t* pGPIOx, uint8_t pinNum);

void GPIO_IRQConfig(uint8_t IRQ_Num, uint8_t IRQ_Prio, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t pinNum);

//---------------------------------------------End of Written Code--------------------------------------


#endif /* INC_STM32L476XX_GPIO_DRIVER_H_ */
