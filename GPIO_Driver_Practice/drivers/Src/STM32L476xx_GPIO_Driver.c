/*
 * STM32L476xx_GPIO_Driver.c
 *
 *  Created on: Jul 25, 2025
 *      Author: Ray Dai
 */


#include "STM32L476xx_GPIO_Driver.h"

//---------------------------------------------Start of Written Code--------------------------------------

/*
 * GPIO_Reg_Clear_Pin_Bits - Clears the bits of a GPIO Register for a specified GPIO Pin
 *
 * Params
 * 		GPIO_Reg_Addr: Address of register to clear bits from
 *		GPIO_PinNum: Pin to clear bits for
 *		GPIO_Pin_BitWidth: Number of bits used per pin in the register
 *		GPIO_Clear_Mask: Bitmask used to clear reg bits
 *
 * Return: None
 */
void GPIO_Reg_Clear_Pin_Bits(uint32_t GPIO_Reg_Addr, GPIO_PIN GPIO_PinNum, GPIO_BITWIDTH GPIO_Pin_BitWidth, GPIO_CLEARMASK GPIO_Clear_Mask) {
	GPIO_Reg_Addr &= ~(GPIO_Clear_Mask << (GPIO_Pin_BitWidth * GPIO_PinNum));
}


/*
 * GPIO_Reg_Set_Pin_Bits - Sets bits of a GPIO Register for a specified GPIO Pin
 *
 * Params
 * 		GPIO_Reg_Addr: Address of register to clear bits from
 *		GPIO_PinNum: Pin to clear bits for
 *		GPIO_Pin_BitWidth: Number of bits used per pin in the register
 *		GPIO_Set_Value: Value to set the reg bits to
 *
 * Return: None
 *
 * Note: It is recommended to use this function with GPIO_Reg_Clear_Pin_Bits,
 * 		as this function does not reset reg bits to 0.
 * 		ex. GPIO_Set_Value = 0b10 will not reset the rightmost bit to 0.
 */
void GPIO_Reg_Set_Pin_Bits(uint32_t GPIO_RegAddr, GPIO_PIN GPIO_PinNum, GPIO_BITWIDTH GPIO_Pin_BitWidth, uint8_t GPIO_Set_Value) {
	GPIO_RegAddr |= GPIO_Set_Value << (GPIO_Pin_BitWidth * GPIO_PinNum);
}

/*
 * GPIO_Init - Initializes a GPIO port to the specified configuration
 *
 * Params
 * 		pGPIO_PinHandle: Pointer to the struct containing the user-specified GPIO configuration details
 *
 * Return: None
 */
void GPIO_Init(GPIO_PinHandle_t* pGPIO_PinHandle) {
	GPIO_PIN pinNum = pGPIO_PinHandle->GPIO_PinConfig.GPIO_PinNum;
	GPIO_PinConfig_t config = pGPIO_PinHandle->GPIO_PinConfig;
	GPIOx_RegDef_t* base = pGPIO_PinHandle->pGPIOx_Base;

	GPIO_Reg_Clear_Pin_Bits(base->MODER, pinNum, GPIO_MODE_BITWIDTH, GPIO_MODE_CLEAR);
	GPIO_Reg_Set_Pin_Bits(base->MODER, pinNum, GPIO_MODE_BITWIDTH, config.GPIO_PinMode);

	GPIO_Reg_Clear_Pin_Bits(base->OTYPER, pinNum, GPIO_OUTTYPE_BITWIDTH, GPIO_OUTTYPE_CLEAR);
	GPIO_Reg_Set_Pin_Bits(base->OTYPER, pinNum, GPIO_OUTTYPE_BITWIDTH, config.GPIO_PinOutType);

	GPIO_Reg_Clear_Pin_Bits(base->OSPEEDR, pinNum, GPIO_OUTSPD_BITWIDTH, GPIO_OUTSPD_CLEAR);
	GPIO_Reg_Set_Pin_Bits(base->OSPEEDR, pinNum, GPIO_OUTSPD_BITWIDTH, config.GPIO_PinOutSpd);

	GPIO_Reg_Clear_Pin_Bits(base->PUPDR, pinNum, GPIO_PUPD_BITWIDTH, GPIO_PUPD_CLEAR);
	GPIO_Reg_Set_Pin_Bits(base->PUPDR, pinNum, GPIO_PUPD_BITWIDTH, config.GPIO_PinPupd);

	if (config.GPIO_PinMode == GPIO_MODE_ALT) {
		if (pinNum < GPIO_ALTL_NUMPINS) {
			GPIO_Reg_Clear_Pin_Bits(base->AFRL, pinNum, GPIO_ALT_BITWIDTH, GPIO_ALT_CLEAR);
			GPIO_Reg_Set_Pin_Bits(base->AFRL, pinNum, GPIO_ALT_BITWIDTH, config.GPIO_PinAltFunc);
		} else {
			GPIO_Reg_Clear_Pin_Bits(base->AFRL, pinNum - GPIO_ALTL_NUMPINS, GPIO_ALT_BITWIDTH, GPIO_ALT_CLEAR);
			GPIO_Reg_Set_Pin_Bits(base->AFRL, pinNum - GPIO_ALTL_NUMPINS, GPIO_ALT_BITWIDTH, config.GPIO_PinAltFunc);
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
	switch ((uint32_t) pGPIOx) {
		case GPIOA_BASE_ADDR:		GPIOA_REG_RESET();	break;
		case GPIOB_BASE_ADDR:		GPIOB_REG_RESET();	break;
		case GPIOC_BASE_ADDR:		GPIOC_REG_RESET();	break;
		case GPIOD_BASE_ADDR:		GPIOD_REG_RESET();	break;
		case GPIOE_BASE_ADDR:		GPIOE_REG_RESET();	break;
		case GPIOF_BASE_ADDR:		GPIOF_REG_RESET();	break;
		case GPIOG_BASE_ADDR:		GPIOG_REG_RESET();	break;
		case GPIOH_BASE_ADDR:		GPIOH_REG_RESET();	break;
		default:					return;
	}
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
void GPIO_Pclk_Ctrl(GPIOx_RegDef_t* pGPIOx, BOOL ENorDI) {
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
uint8_t GPIO_ReadPin(GPIOx_RegDef_t* pGPIOx, GPIO_PIN pinNum) {
	return ((pGPIOx->IDR) >> pinNum) & 1;
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
	return pGPIOx->IDR;
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
void GPIO_WritePin(GPIOx_RegDef_t* pGPIOx, GPIO_PIN pinNum, BOOL value) {
	if (value == SET) {
		pGPIOx->ODR |= 1 << pinNum;
	} else {
		pGPIOx->ODR &= ~(1 << pinNum);
	}
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
	pGPIOx->ODR = value;
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
void GPIO_TogglePin(GPIOx_RegDef_t* pGPIOx, GPIO_PIN pinNum) {

	pGPIOx->ODR ^= (1 << pinNum);
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
void GPIO_IRQConfig(uint8_t IRQ_Num, uint8_t IRQ_Prio, BOOL ENorDI) {

}

/*
 * GPIO_IRQHandling - Callback function for user to handle GPIO interrupts
 *
 * Params
 * 		pinNum: Pin number causing interrupt
 *
 * Return: None
 */
void GPIO_IRQHandling(GPIO_PIN pinNum) {

}

//---------------------------------------------End of Written Code--------------------------------------
