/*
 * STM32L476xx_GPIO_Driver.c
 *
 *  Created on: Jul 25, 2025
 *      Author: Ray Dai
 */


#include "STM32L476xx_GPIO_Driver.h"

//---------------------------------------------Start of Written Code--------------------------------------

/*
 * GPIO_Init - Initializes a GPIO port to the specified configuration
 *
 * Params
 * 		pGPIO_PinHandle: Pointer to the struct containing the user-specified GPIO configuration details
 *
 * Return: None
 */
void GPIO_Init(GPIO_PinHandle_t* pGPIO_PinHandle) {
	uint8_t pinNum = pGPIO_PinHandle->GPIO_PinConfig.GPIO_PinNum;
	GPIO_PinConfig_t config = pGPIO_PinHandle->GPIO_PinConfig;
	GPIOx_RegDef_t* base = pGPIO_PinHandle->pGPIOx_Base;

	base->MODER &=		~(GPIO_MODE_CLEARMASK << (GPIO_MODE_BITWIDTH * pinNum));
	base->MODER |=		(config.GPIO_PinMode) << (GPIO_MODE_BITWIDTH * pinNum);
	base->OTYPER &=		~(GPIO_OUTTYPE_CLEARMASK << (GPIO_OUTTYPE_BITWIDTH * pinNum));
	base->OTYPER |=		(config.GPIO_PinOutType) << (GPIO_OUTTYPE_BITWIDTH * pinNum);
	base->OSPEEDR &=	~(GPIO_OUTSPD_CLEARMASK << (GPIO_OUTSPD_CLEARMASK * pinNum));
	base->OSPEEDR |=	(config.GPIO_PinOutSpd) << (GPIO_OUTSPD_BITWIDTH * pinNum);
	base->PUPDR &=		~(GPIO_PUPD_CLEARMASK << (GPIO_PUPD_BITWIDTH * pinNum));
	base->PUPDR |=		(config.GPIO_PinPupd) << (GPIO_PUPD_BITWIDTH * pinNum);

	if (config.GPIO_PinMode == GPIO_MODE_ALT) {
		if (pinNum < GPIO_ALTL_NUMPINS) {
			base->AFRL &=	~(GPIO_ALT_CLEARMASK << (GPIO_ALT_BITWIDTH * pinNum));
			base->AFRL |=	(config.GPIO_PinAltFunc) << (GPIO_ALT_BITWIDTH * pinNum);
		} else {
			base->AFRH &=	~(GPIO_ALT_CLEARMASK << (GPIO_ALT_BITWIDTH * (pinNum - GPIO_ALTL_NUMPINS)));
			base->AFRH |=	(config.GPIO_PinAltFunc) << (GPIO_ALT_BITWIDTH * (pinNum - GPIO_ALTL_NUMPINS));
		}
	}
}

/*
 * GPIO_DeInit - Resets a GPIO port's registers via the RCC AHB Reset Reg
 *
 * Params
 * 		pGPIOx: Pointer to the base address of the GPIO port
 *
 * Return: None
 */
void GPIO_DeInit(GPIOx_RegDef_t* pGPIOx) {

}

/*
 * GPIO_Pclk_Ctrl - Enable/Disables the peripheral clock for a specified GPIO port
 *
 * Params
 * 		pGPIOx: Pointer to base address of the GPIO port
 * 		ENorDI: ENable or DIsable clock. Recommended to use EN/DI macros
 *
 * Return: None
 */
void GPIO_Pclk_Ctrl(GPIOx_RegDef_t* pGPIOx, uint8_t ENorDI) {
	if (ENorDI == EN) {
		switch ((uint32_t) pGPIOx) {
			case GPIOA_BASE_ADDR:		GPIOA_PCLK_EN();	break;
			case GPIOB_BASE_ADDR:		GPIOB_PCLK_EN();	break;
			case GPIOC_BASE_ADDR:		GPIOC_PCLK_EN();	break;
			case GPIOD_BASE_ADDR:		GPIOD_PCLK_EN();	break;
			case GPIOE_BASE_ADDR:		GPIOE_PCLK_EN();	break;
			case GPIOF_BASE_ADDR:		GPIOF_PCLK_EN();	break;
			case GPIOG_BASE_ADDR:		GPIOG_PCLK_EN();	break;
			case GPIOH_BASE_ADDR:		GPIOH_PCLK_EN();	break;
			default:					return;
		}
	} else {
		switch ((uint32_t) pGPIOx) {
			case GPIOA_BASE_ADDR:		GPIOA_PCLK_DI();	break;
			case GPIOB_BASE_ADDR:		GPIOB_PCLK_DI();	break;
			case GPIOC_BASE_ADDR:		GPIOC_PCLK_DI();	break;
			case GPIOD_BASE_ADDR:		GPIOD_PCLK_DI();	break;
			case GPIOE_BASE_ADDR:		GPIOE_PCLK_DI();	break;
			case GPIOF_BASE_ADDR:		GPIOF_PCLK_DI();	break;
			case GPIOG_BASE_ADDR:		GPIOG_PCLK_DI();	break;
			case GPIOH_BASE_ADDR:		GPIOH_PCLK_DI();	break;
			default:					return;
		}

	}
}

/*
 * GPIO_ReadPin - Read the current state of the specified GPIO pin
 *
 * Params
 * 		pGPIOx: Pointer to base address of the GPIO port
 * 		pinNum: Integer corresponding to which pin should be read
 *
 * Return: 1 or 0 corresponding to the pin state
 */
uint8_t GPIO_ReadPin(GPIOx_RegDef_t* pGPIOx, uint8_t pinNum) {

}

/*
 * GPIO_ReadPort - Read the current state of the specified GPIO port
 *
 * Params
 * 		pGPIOx: Pointer to base address of the GPIO port
 *
 * Return: 16-bit integer corresponding to the GPIO port's 16 pin states
 */
uint16_t GPIO_ReadPort(GPIOx_RegDef_t* pGPIOx) {

}

/*
 * GPIO_WritePin - Change the state of the specified GPIO pin
 *
 * Params
 * 		pGPIOx: Pointer to base address of the GPIO port
 * 		pinNum: Integer corresponding to which pin to write to
 * 		value: State to change the pin to. Recommended to use HI/LO macros
 *
 * Return: None
 */
void GPIO_WritePin(GPIOx_RegDef_t* pGPIOx, uint8_t pinNum, uint8_t value) {

}

/*
 * GPIO_WritePort - Change the state of the specified GPIO port
 *
 * Params
 * 		pGPIOx: Pointer to base address of the GPIO port
 * 		value: 16-bit integer corresponding to the states to change the 16 GPIO port pins to
 *
 * Return: None
 */
void GPIO_WritePort(GPIOx_RegDef_t* pGPIOx, uint16_t value) {

}

/*
 * GPIO_TogglePin - Toggle the state of the specified GPIO pin
 *
 * Params
 * 		pGPIOx: Pointer to base address of the GPIO port
 * 		pinNum: Integer corresponding to which pin to toggle
 *
 * Return: None
 */
void GPIO_TogglePin(GPIOx_RegDef_t* pGPIOx, uint8_t pinNum) {

}

/*
 * GPIO_IRQConfig - Set up the IRQ number for GPIO interrupts
 *
 * Params
 * 		IRQ_Num: IRQ Number
 * 		IRQ_Prio: Desired IRQ exception priority
 * 		ENorDI: ENable or DIsable IRQ number. Recommended to use EN/DI macros
 *
 * Return: None
 */
void GPIO_IRQConfig(uint8_t IRQ_Num, uint8_t IRQ_Prio, uint8_t ENorDI) {

}

/*
 * GPIO_IRQHandling - Callback function for user to handle GPIO interrupts
 *
 * Params
 * 		pinNum: Pin number causing interrupt
 *
 * Return: None
 */
void GPIO_IRQHandling(uint8_t pinNum) {

}

//---------------------------------------------End of Written Code--------------------------------------
