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

#define GPIO_ALTL_NUMPINS		8
#define SYSCFG_EXTICR_NUMPINS	4

typedef enum {
	GPIO_MODE_BITWIDTH =		2,
	GPIO_OUTTYPE_BITWIDTH =		1,
	GPIO_OUTSPD_BITWIDTH =		2,
	GPIO_PUPD_BITWIDTH =		2,
	GPIO_ALT_BITWIDTH =			4,
	SYSCFG_EXTICR_BITWIDTH =	4
} GPIO_BITWIDTH;

typedef enum {
	GPIO_MODE_CLEAR =		0b11,
	GPIO_OUTTYPE_CLEAR =	0b1,
	GPIO_OUTSPD_CLEAR =		0b11,
	GPIO_PUPD_CLEAR =		0b11,
	GPIO_ALT_CLEAR =		0b1111,
	SYSCFG_EXTICR_CLEAR =	0b1111
} GPIO_CLEARMASK;

typedef enum {
	GPIO_MODE_IN =	0b00,
	GPIO_MODE_OUT =	0b01,
	GPIO_MODE_ALT =	0b10,		// Alt Function Mode
	GPIO_MODE_AN =	0b11		// Analog Out Mode
} GPIO_MODE;

typedef enum {
	GPIO_OUTTYPE_PP =	0,		// Push-pull output
	GPIO_OUTTYPE_OD =	1		// Open-drain output
} GPIO_OUTTYPE;

typedef enum {
	GPIO_OUTSPD_LO =	0b00,	// 25-110ns output rise/fall time
	GPIO_OUTSPD_MD =	0b01,	// 9-21ns output rise/fall time
	GPIO_OUTSPD_HI =	0b10,	// 6-12ns output rise/fall time
	GPIO_OUTSPD_VHI =	0b11	// 4-16ns output rise/fall time
} GPIO_OUTSPD;

typedef enum {
	GPIO_PUPD_NO =		0b00,	// No pullup/pulldown
	GPIO_PUPD_PU =		0b01,	// Yes pullup, no pulldown
	GPIO_PUPD_PD =		0b10	// No pullup, yes pulldown
} GPIO_PUPD;

typedef enum {
	AF0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15
} GPIO_ALT;

typedef enum {
	GPIO_IT_NO,		// No interrupts
	GPIO_IT_RT,		// Rising-edge trigger interrupts
	GPIO_IT_FT,		// Falling-edge trigger interrupts
	GPIO_IT_RFT		// Rising and falling edge trigger interrupts
} GPIO_IT;

typedef struct {
	GPIO_PIN		GPIO_PinNum;
	GPIO_MODE		GPIO_PinMode;
	GPIO_OUTTYPE	GPIO_PinOutType;
	GPIO_OUTSPD		GPIO_PinOutSpd;
	GPIO_PUPD		GPIO_PinPupd;
	GPIO_ALT		GPIO_PinAltFunc;
	GPIO_IT			GPIO_ITMode;
} GPIO_PinConfig_t;

typedef struct {
	GPIOx_RegDef_t*		pGPIOx_Base;
	GPIO_PinConfig_t 	GPIO_PinConfig;
	GPIO_PORT			GPIO_Port;
} GPIO_PinHandle_t;

void GPIO_Reg_Clear_Pin_Bits(uint32_t* GPIO_Reg_Addr, GPIO_PIN GPIO_PinNum, GPIO_BITWIDTH GPIO_Pin_BitWidth, GPIO_CLEARMASK GPIO_Clear_Mask);
void GPIO_Reg_Set_Pin_Bits(uint32_t* GPIO_RegAddr, GPIO_PIN GPIO_PinNum, GPIO_BITWIDTH GPIO_Pin_BitWidth, uint8_t GPIO_Set_Value);

void GPIO_Init(GPIO_PinHandle_t* pGPIO_PinHandle);
void GPIO_DeInit(GPIOx_RegDef_t* pGPIOx);
void GPIO_Pclk_Ctrl(GPIOx_RegDef_t* pGPIOx, BOOL ENorDI);

uint8_t GPIO_ReadPin(GPIOx_RegDef_t* pGPIOx, GPIO_PIN pinNum);
uint16_t GPIO_ReadPort(GPIOx_RegDef_t* pGPIOx);
void GPIO_WritePin(GPIOx_RegDef_t* pGPIOx, GPIO_PIN pinNum, BOOL value);
void GPIO_WritePort(GPIOx_RegDef_t* pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIOx_RegDef_t* pGPIOx, GPIO_PIN pinNum);

void GPIO_IRQConfig(uint8_t IRQ_Num, uint8_t IRQ_Prio, BOOL ENorDI);
void GPIO_IRQPriorityConfig(uint8_t IRQ_Num, uint8_t IRQ_Prio);
void GPIO_IRQHandling(GPIO_PIN pinNum);

//---------------------------------------------End of Written Code--------------------------------------


#endif /* INC_STM32L476XX_GPIO_DRIVER_H_ */
