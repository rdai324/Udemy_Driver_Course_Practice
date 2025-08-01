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

#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

#define GPIO_MODE_CLEARMASK	0b11
#define GPIO_MODE_BITWIDTH	2
#define GPIO_MODE_IN		0b00
#define GPIO_MODE_OUT		0b01
#define GPIO_MODE_ALT		0b10		// Alt Function Mode
#define GPIO_MODE_AN		0b11		// Analog Mode

#define GPIO_OUTTYPE_CLEARMASK	0b1
#define GPIO_OUTTYPE_BITWIDTH	1
#define GPIO_OUTTYPE_PP			0			// Push-pull output
#define GPIO_OUTTYPE_OD			1			// Open-drain output

#define GPIO_OUTSPD_CLEARMASK	0b11
#define GPIO_OUTSPD_BITWIDTH	2
#define GPIO_OUTSPD_LO			0b00		// 25-110ns output rise/fall time
#define GPIO_OUTSPD_MD			0b01		// 9-21ns output rise/fall time
#define GPIO_OUTSPD_HI			0b10		// 6-12ns output rise/fall time
#define GPIO_OUTSPD_VHI			0b11		// 4-16ns output rise/fall time

#define GPIO_PUPD_CLEARMASK	0b11
#define GPIO_PUPD_BITWIDTH	2
#define GPIO_PUPD_NO		0b00		// No pullup/pulldown
#define GPIO_PUPD_PU		0b01		// Yes pullup, no pulldown
#define GPIO_PUPD_PD		0b10		// No pullup, yes pulldown
										// 0b11 is reserved

#define GPIO_ALTL_NUMPINS	8
#define GPIO_ALT_CLEARMASK	0b1111
#define GPIO_ALT_BITWIDTH	4

typedef struct {
	uint8_t GPIO_PinNum;		// Valid values from GPIO_PIN_XX macros
	uint8_t GPIO_PinMode;		// Valid values from GPIO_MODE_XX macros
	uint8_t GPIO_PinOutType;	// Valid values from GPIO_OUTTYPE_XX macros
	uint8_t GPIO_PinOutSpd;		// Valid values from GPIO_OUTSPD_XX macros
	uint8_t GPIO_PinPupd;		// Valid values from GPIO_PUPD_XX macros
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
void GPIO_WritePort(GPIOx_RegDef_t* pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIOx_RegDef_t* pGPIOx, uint8_t pinNum);

void GPIO_IRQConfig(uint8_t IRQ_Num, uint8_t IRQ_Prio, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t pinNum);

//---------------------------------------------End of Written Code--------------------------------------


#endif /* INC_STM32L476XX_GPIO_DRIVER_H_ */
