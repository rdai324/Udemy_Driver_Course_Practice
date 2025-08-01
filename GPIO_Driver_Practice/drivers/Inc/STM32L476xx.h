/*
 * STM32L476RG.h
 *
 *  Created on: Jul 10, 2025
 *      Author: Ray Dai
 */

#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_

#include <stdint.h>

//---------------------------------------------Start of Written Code--------------------------------------

#define vol		volatile
#define EN		1
#define ENABLE	1
#define SET		1
#define HI		1
#define DI		0
#define DISABLE	0
#define RESET	0
#define LO		0

// Memory Base Address Macros
#define FLASH_BASE_ADDR		0x08000000UL
#define ROM_BASE_ADDR		0x1FFF0000UL
#define SRAM1_BASE_ADDR		0x20000000UL
#define SRAM2_BASE_ADDR		0x10000000UL

// Bus Domain Base Address Macros
#define PERIPH_BASE_ADDR	0x40000000UL
#define APB1_BASE_ADDR		(PERIPH_BASE_ADDR + 0x00000000UL)
#define APB2_BASE_ADDR		(PERIPH_BASE_ADDR + 0x00010000UL)
#define AHB1_BASE_ADDR		(PERIPH_BASE_ADDR + 0x00020000UL)
#define AHB2_BASE_ADDR		(PERIPH_BASE_ADDR + 0x08000000UL)

/*
 * Peripheral Base Address Macros
 * NOTE: not all peripherals done
 */

// AHB1 Peripherals
#define RCC_BASE_ADDR		(AHB1_BASE_ADDR + 0x00001000UL)

// AHB2 Peripherals
#define GPIO_BASE_ADDR		(AHB2_BASE_ADDR + 0x00000000UL)
#define GPIOA_BASE_ADDR		(GPIO_BASE_ADDR + 0x00000000UL)
#define GPIOB_BASE_ADDR		(GPIO_BASE_ADDR + 0x00000400UL)
#define GPIOC_BASE_ADDR		(GPIO_BASE_ADDR + 0x00000800UL)
#define GPIOD_BASE_ADDR		(GPIO_BASE_ADDR + 0x00000C00UL)
#define GPIOE_BASE_ADDR		(GPIO_BASE_ADDR + 0x00001000UL)
#define GPIOF_BASE_ADDR		(GPIO_BASE_ADDR + 0x00001400UL)
#define GPIOG_BASE_ADDR		(GPIO_BASE_ADDR + 0x00001800UL)
#define GPIOH_BASE_ADDR		(GPIO_BASE_ADDR + 0x00001C00UL)

// APB1 Peripherals
#define I2C_BASE_ADDR		(APB1_BASE_ADDR + 0x00005400UL)
#define I2C1_BASE_ADDR		(I2C_BASE_ADDR + 0x00000000UL)
#define I2C2_BASE_ADDR		(I2C_BASE_ADDR + 0x00000400UL)
#define I2C3_BASE_ADDR		(I2C_BASE_ADDR + 0x00000800UL)

#define SPI2_BASE_ADDR		(APB1_BASE_ADDR + 0x00003800UL)
#define SPI3_BASE_ADDR		(APB1_BASE_ADDR + 0x00003C00UL)

#define USART2_BASE_ADDR	(APB1_BASE_ADDR + 0x00004400UL)
#define USART3_BASE_ADDR	(APB1_BASE_ADDR + 0x00004800UL)
#define UART4_BASE_ADDR		(APB1_BASE_ADDR + 0x00004C00UL)
#define UART5_BASE_ADDR		(APB1_BASE_ADDR + 0x00005000UL)

// APB2 Peripherals
#define EXTI_BASE_ADDR		(APB2_BASE_ADDR + 0x00000400UL)
#define SPI1_BASE_ADDR		(APB2_BASE_ADDR + 0x00003000UL)
#define SYSCFG_BASE_ADDR	(APB2_BASE_ADDR + 0x00000000UL)
#define USART1_BASE_ADDR	(APB2_BASE_ADDR + 0x00003800UL)

/*
 * Peripheral Structures
 */
typedef struct {
	vol uint32_t CR;			// clock control reg										0x00
	vol uint32_t ICSCR;			// internal clock sources callibration reg					0x04
	vol uint32_t CFGR;			// clock config reg											0x08
	vol uint32_t PLLCFGR;		// PLL config reg											0x0C
	vol uint32_t PLLSAI1CFGR;	// PLLSAI1 config reg										0x10
	vol uint32_t PLLSAI2CFGR;	// PLLSAI2 config reg										0x14
	vol uint32_t CIER;			// clock interrupt enable reg								0x18
	vol uint32_t CIFR;			// clock interrupt flag reg									0x1C
	vol uint32_t CICR;			// clock interrupt clear reg								0x20
	uint32_t _RESERVED1_;		// xRESERVEDx
	vol uint32_t AHB1RSTR;		// AHB1 peripheral reset reg								0x28
	vol uint32_t AHB2RSTR;		// AHB2 peripheral reset reg								0x2C
	vol uint32_t AHB3RSTR;		// AHB3 peripheral reset reg								0x30
	uint32_t _RESERVED2_;		// xRESERVEDx
	vol uint32_t APB1RSTR1;		// APB1 peripheral reset reg 1								0x38
	vol uint32_t APB1RSTR2;		// APB1 peripheral reset reg 2								0x3C
	vol uint32_t APB2RSTR;		// APB2 peripheral reset reg								0x40
	uint32_t _RESERVED3_;		// xRESERVEDx
	vol uint32_t AHB1ENR;		// AHB1 peripheral clock enable reg							0x48
	vol uint32_t AHB2ENR;		// AHB2 peripheral clock enable reg							0x4C
	vol uint32_t AHB3ENR;		// AHB3 peripheral clock enable reg							0x50
	uint32_t _RESERVED4_;		// xRESERVEDx
	vol uint32_t APB1ENR1;		// APB1 peripheral clock enable reg 1						0x58
	vol uint32_t APB1ENR2;		// APB1 peripheral clock enable reg 2						0x5C
	vol uint32_t APB2ENR;		// APB2 peripheral clock enable								0x60
	uint32_t _RESERVED5_;		// xRESERVEDx
	vol uint32_t AHB1SMENR;		// AHB1 peripheral clock enable in sleep & stop modes reg	0x68
	vol uint32_t AHB2SMENR;		// AHB2 peripheral clock enable in sleep & stop modes reg	0x6C
	vol uint32_t AHB3SMENR;		// AHB3 peripheral clock enable in sleep & stop modes reg	0x70
	uint32_t _RESERVED6_;		// xRESERVEDx
	vol uint32_t APB1SMENR1;	// APB1 peripheral clock enable in sleep & stop modes reg 1	0x78
	vol uint32_t APB1SMENR2;	// APB1 peripheral clock enable in sleep & stop modes reg 2	0x7C
	vol uint32_t APB2SMENR;		// APB2 peripheral clock enable in sleep & stop modes reg	0x80
	uint32_t _RESERVED7_;		// xRESERVEDx
	vol uint32_t CCIPR;			// peripherals independent clock config reg					0x88
	uint32_t _RESERVED8_;		// xRESERVEDx
	vol uint32_t BDCR;			// backup domain control reg								0x90
	vol uint32_t CSR;			// control/status reg										0x94
	vol uint32_t CRRCR;			// clock recovery RC reg									0x98
	vol uint32_t CCIPR2;		// peripherals independent clock config reg					0x9C
} RCC_RegDef_t;

typedef struct {
	vol uint32_t MODER;		// GPIO port mode reg							0x00
	vol uint32_t OTYPER;	// GPIO port output type reg					0x04
	vol uint32_t OSPEEDR;	// GPIO port output speed reg					0x08
	vol uint32_t PUPDR;		// GPIO port pull-up/pull-down reg				0x0C
	vol uint32_t IDR;		// GPIO port input data reg						0x10
	vol uint32_t ODR;		// GPIO port output data reg					0x14
	vol uint32_t BSRR;		// GPIO port bit set/reset reg					0x18
	vol uint32_t LCKR;		// GPIO port configuration lock reg				0x1C
	vol uint32_t AFRL;		// GPIO port alt function low reg				0x20
	vol uint32_t AFRH;		// GPIO port alt function high reg				0x24
	vol uint32_t BRR;		// GPIO port bit reset reg						0x28
	vol uint32_t ASCR;		// GPIO port analog switch control reset reg	0x2C
} GPIOx_RegDef_t;

typedef struct {
	vol uint32_t CR1;		// I2C control reg 1		0x00
	vol uint32_t CR2;		// I2C control reg 2		0x04
	vol uint32_t OAR1;		// I2C own addr 1 reg		0x08
	vol uint32_t OAR2;		// I2C own addr 2 reg		0x0C
	vol uint32_t TIMINGR;	// I2C timing reg			0x10
	vol uint32_t TIMEOUTR;	// I2C timeout reg			0x14
	vol uint32_t ISR;		// I2C interrupt status reg	0x18
	vol uint32_t ICR;		// I2C interrupt clear reg	0x1C
	vol uint32_t PECR;		// I2C PEC reg				0x20
	vol uint32_t RXDR;		// I2C receive data reg		0x24
	vol uint32_t TXDR;		// I2C transmit data reg	0x28
} I2Cx_RegDef_t;

typedef struct {
	vol uint32_t CR1;		// SPI control reg 1		0x00
	vol uint32_t CR2;		// SPI control reg 2		0x04
	vol uint32_t SR;		// SPI status reg			0x08
	vol uint32_t DR;		// SPI data reg				0x0C
	vol uint32_t CRCPR;		// SPI CRC polynomial reg	0x10
	vol uint32_t RXCRCR;	// SPI RX CRC reg			0x14
	vol uint32_t TXCRCR;	// SPI TX CRC reg			0x18
} SPIx_RegDef_t;

typedef struct {
	vol uint32_t CR1;		// USART control reg 1				0x00
	vol uint32_t CR2;		// USART control reg 2				0x04
	vol uint32_t CR3;		// USART control reg 3				0x08
	vol uint32_t BRR;		// USART baud rate reg				0x0C
	vol uint32_t GTPR;		// USART guard time & prescaler reg	0x10
	vol uint32_t RTOR;		// USART receiver timeout reg		0x14
	vol uint32_t RQR;		// USART request reg				0x18
	vol uint32_t ISR;		// USART interrupt & status reg		0x1C
	vol uint32_t ICR;		// USART interrupt flag clear reg	0x20
	vol uint32_t RDR;		// USART receive data reg			0x24
	vol uint32_t TDR;		// USART transmit data reg			0x28
} USARTx_RegDef_t;

/*
 * Peripheral Macros
 */
#define RCC		((RCC_RegDef_t*) RCC_BASE_ADDR)

#define GPIOA	((GPIOx_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB	((GPIOx_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC	((GPIOx_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD	((GPIOx_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE	((GPIOx_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF	((GPIOx_RegDef_t*) GPIOF_BASE_ADDR)
#define GPIOG	((GPIOx_RegDef_t*) GPIOG_BASE_ADDR)
#define GPIOH	((GPIOx_RegDef_t*) GPIOH_BASE_ADDR)

/*
 * Clock Enable Macros
 */
#define GPIOA_PCLK_EN()		(RCC->AHB2ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB2ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB2ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB2ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB2ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB2ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB2ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB2ENR |= (1<<7))

#define I2C1_PCLK_EN()		(RCC->APB1ENR1 |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR1 |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR1 |= (1<<23))

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12)))
#define SPI2_PCLK_EN()		(RCC->APB1ENR1 |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR1 |= (1<<15))

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1<<14))
#define USART2_PCLK_EN()	(RCC->APB1ENR1 |= (1<<17))
#define USART3_PCLK_EN()	(RCC->APB1ENR1 |= (1<<18))
#define UART4_PCLK_EN()		(RCC->APB1ENR1 |= (1<<19))
#define UART5_PCLK_EN()		(RCC->APB1ENR1 |= (1<<20))

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<0))

/*
 * Clock Disable Macros
 */
#define GPIOA_PCLK_DI()	(RCC->AHB2ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()	(RCC->AHB2ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()	(RCC->AHB2ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()	(RCC->AHB2ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()	(RCC->AHB2ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()	(RCC->AHB2ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()	(RCC->AHB2ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()	(RCC->AHB2ENR &= ~(1<<7))

#define I2C1_PCLK_DI()		(RCC->APB1ENR1 &= ~(1<<21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR1 &= ~(1<<22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR1 &= ~(1<<23))

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12)))
#define SPI2_PCLK_DI()		(RCC->APB1ENR1 &= ~(1<<14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR1 &= ~(1<<15))

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI()	(RCC->APB1ENR1 &= ~(1<<17))
#define USART3_PCLK_DI()	(RCC->APB1ENR1 &= ~(1<<18))
#define UART4_PCLK_DI()	(RCC->APB1ENR1 &= ~(1<<19))
#define UART5_PCLK_DI()	(RCC->APB1ENR1 &= ~(1<<20))

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1<<0))

//---------------------------------------------End of Written Code--------------------------------------

#endif /* INC_STM32L476XX_H_ */
