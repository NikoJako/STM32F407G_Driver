/*
 * stm32f407g.h
 *
 *  Created on: Apr 16, 2021
 *      Author: njacobs
 */

#ifndef INC_STM32F407G_H_
#define INC_STM32F407G_H_

#include <stdint.h>


/************************Start: Processor Specific Details********************************************************************************************
 *
 * ARM Cortex M4 Processor NVIC Interrupt Set-Enable Register X (ISERx) Register Addresses
 * all should be volatile unsigned 32-bit integer pointers to corresponding memory addresses
 *
 * Section 4.2.2 (page 4-2) in the Cortex M4 Generic User Guide
 * */
#define NVIC_ISER0							((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1							((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2							((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3							((volatile uint32_t *)0xE000E11C)

/*
 * Interrupt Clear Enable Registers (ICERx)
 * Section 4.2.3 (page 4-3) Cortex M4 Generic User Guide
 *
 * */
#define NVIC_ICER0							((volatile uint32_t *)0xE000E180)
#define NVIC_ICER1							((volatile uint32_t *)0xE000E184)
#define NVIC_ICER2							((volatile uint32_t *)0xE000E188)
#define NVIC_ICER3							((volatile uint32_t *)0xE000E18C)

/*
 * Interrupt Priority Registers (NVIC_IPR0-IPR59)
 * Section 4.2.7 (page 4-7) Cortex M4 Generic User Guide
 *
 * */
#define NVIC_IPR_BASE_ADDR  			    ((volatile uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED				4



//Generic Macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET


/* Memory Base Addresses
 *
 * 112KB*1024 = 114688 bytes
 * convert 114688 to hex = 0x0001C000
 *
 * */
#define FLASH_BASE_ADDR						0x08000000U
#define SRAM1_BASE_ADDR						0x20000000U
#define SRAM2_BASE_ADDR						0x2001C000U
#define ROM_BASE_ADDR						0x1FFF0000U
#define OTP_BASE_ADDR						0x1FFF7800U

//Buses
#define	PERIPH_BASE							0x40000000U
#define	APB1_PERIPH_BASE					PERIPH_BASE
#define APB2_PERIPH_BASE					0x40010000U
#define AHB1_PERIPH_BASE					0x40020000U
#define AHB2_PERIPH_BASE					0x50000000U

//AHB1 Peripherals - GPIO & RCC Registers
#define GPIOA_BASE_ADDR			(AHB1_PERIPH_BASE + 0x0000)
#define GPIOB_BASE_ADDR		(AHB1_PERIPH_BASE + 0x0400)
#define GPIOC_BASE_ADDR		(AHB1_PERIPH_BASE + 0x0800)
#define GPIOD_BASE_ADDR		(AHB1_PERIPH_BASE + 0x0C00)
#define GPIOE_BASE_ADDR		(AHB1_PERIPH_BASE + 0x1000)
#define GPIOF_BASE_ADDR		(AHB1_PERIPH_BASE + 0x1400)
#define GPIOG_BASE_ADDR		(AHB1_PERIPH_BASE + 0x1800)
#define GPIOH_BASE_ADDR		(AHB1_PERIPH_BASE + 0x1C00)
#define GPIOI_BASE_ADDR		(AHB1_PERIPH_BASE + 0x2000)

#define RCC_BASE_ADDR 		(AHB1_PERIPH_BASE + 0x3800)

//APB1 Peripherals
#define USART_2_BASE_ADDR	(APB1_PERIPH_BASE + 0x4400)
#define USART_3_BASE_ADDR	(APB1_PERIPH_BASE + 0x4800)
#define UART_4_BASE_ADDR	(APB1_PERIPH_BASE + 0x4C00)
#define UART_5_BASE_ADDR	(APB1_PERIPH_BASE + 0x5000)

#define I2C_1_BASE_ADDR		(APB1_PERIPH_BASE + 0x5400)
#define I2C_2_BASE_ADDR		(APB1_PERIPH_BASE + 0x5800)
#define I2C_3_BASE_ADDR		(APB1_PERIPH_BASE + 0x5C00)

//APB2 Peripherals - SPI, USART, EXTI & SYSCFG
#define USART_1_BASE_ADDR	(APB2_PERIPH_BASE + 0x1000)
#define USART_6_BASE_ADDR	(APB2_PERIPH_BASE + 0x1400)

#define EXTI_BASE_ADDR		(APB2_PERIPH_BASE + 0x3C00)
#define SYSCFG_BASE_ADDR	(APB2_PERIPH_BASE + 0x3800)

/***********************Peripheral Register Definition Structures**********************/
/***************************RCC Register Map on Page 265of RM************************/
typedef struct
{
	volatile uint32_t RCC_CR;			//RCC clock control register									Address offset: 0x00
	volatile uint32_t RCC_PLLCFGR;		//RCC PLL configuration register								Address offset: 0x04
	volatile uint32_t RCC_CFGR;			//RCC clock configuration register								Address offset: 0x08
	volatile uint32_t RCC_CIR;			//RCC clock interrupt register									Address offset: 0x0C
	volatile uint32_t RCC_AHB1RSTR;		//RCC AHB1 peripheral reset register							Address offset: 0x10
	volatile uint32_t RCC_AHB2RSTR;		//RCC AHB2 peripheral reset register							Address offset: 0x14
	volatile uint32_t RCC_AHB3RSTR;		//RCC AHB3 peripheral reset register							Address offset: 0x18
	uint32_t RESERVED0;					//Reserved 0x1C
	volatile uint32_t RCC_APB1RSTR;		//RCC APB1 peripheral reset register							Address offset: 0x20
	volatile uint32_t RCC_APB2RSTR;		//RCC APB2 peripheral reset register							Address offset: 0x24
	uint32_t RESERVED1;					//Reserved 0x28
	uint32_t RESERVED2;					//Reserved 0x2C
	volatile uint32_t RCC_AHB1ENR; 		//RCC AHB1 peripheral clock enable register						Address offset: 0x30
	volatile uint32_t RCC_AHB2ENR;		//RCC AHB2 peripheral clock enable register						Address offset: 0x34
	volatile uint32_t RCC_AHB3ENR;		//RCC AHB3 peripheral clock enable register						Address offset: 0x38
	uint32_t RESERVED3;					//Reserved 0x3C
	volatile uint32_t RCC_APB1ENR;		//RCC APB1 peripheral clock enable register						Address offset: 0x40
	volatile uint32_t RCC_APB2ENR;		//RCC APB2 peripheral clock enable register						Address offset: 0x44
	uint32_t RESERVED4;					//Reserved 0x48
	uint32_t RESERVED5;					//Reserved 0x4C
	volatile uint32_t RCC_AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register	Address offset: 0x50
	volatile uint32_t RCC_AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register	Address offset: 0x54
	volatile uint32_t RCC_AHB3LPENR;	//RCC APB1 peripheral reset register							Address offset: 0x58
	uint32_t RESERVED6;					//Reserved 0x5C
	volatile uint32_t RCC_APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register	Address offset: 0x60
	volatile uint32_t RCC_APB2LPENR; 	//RCC APB2 peripheral clock enabled in low power mode			Address offset: 0x64
	uint32_t RESERVED7;					//Reserved 0x68
	uint32_t RESERVED8;					//Reserved 0x6C
	volatile uint32_t RCC_BDCR;			//RCC Backup domain control register							Address offset: 0x70
	volatile uint32_t RCC_CSR;			//RCC clock control & status register							Address offset: 0x74
	uint32_t RESERVED9;					//Reserved 0x78
	uint32_t RESERVED10;				//Reserved 0x7C
	volatile uint32_t RCC_SSCGR;		//RCC spread spectrum clock generation register					Address offset: 0x80
	volatile uint32_t RCC_PLLI2SCFGR;	//RCC PLLI2S configuration register								Address offset: 0x84

}RCC_RegDef_t;

//pointer to struct of type RCC_RegDef_t
#define RCC		((RCC_RegDef_t*)RCC_BASE_ADDR)


/**************************GPIO register map on page 287 of RM*************************/
typedef struct
{
	volatile uint32_t MODER;	//GPIO port mode register					Address offset: 0x00
	volatile uint32_t OTYPER;	//GPIO port output type register			Address offset: 0x04
	volatile uint32_t OSPEEDR;	//GPIO port output speed register			Address offset: 0x08
	volatile uint32_t PUPDR;	//GPIO port pull-up/pull-down register		Address offset: 0x0C
	volatile uint32_t IDR;		//GPIO port input data register				Address offset: 0x10
	volatile uint32_t ODR;		//GPIO port output data register			Address offset: 0x14
	volatile uint32_t BSRR;		//GPIO port bit set/reset register low		Address offset: 0x18
	volatile uint32_t LCKR;		//GPIO port configuration lock register		Address offset: 0x1C
	volatile uint32_t AFR[2]; 	//GPIO alternate function low register		Address offset: 0x20--> AFR[0]
								//GPIO alternate function high register		Address offset: 0x24--> AFR[1]
}GPIO_RegDef_t;

//GPIOx base addresses typecasted
#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASE_ADDR)

//Clock Enable Macros for GPIOx Peripherals
#define GPIOA_PCLK_EN()		(RCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->RCC_AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->RCC_AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->RCC_AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->RCC_AHB1ENR |= (1 << 8))

#define GPIOA_PCLK_DI()		(RCC->RCC_AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->RCC_AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->RCC_AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->RCC_AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->RCC_AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->RCC_AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->RCC_AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->RCC_AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->RCC_AHB1ENR &= ~(1 << 8))

/*
 * macros to reset GPIOx peripherals
 * RCC_AHB1RSTR - see page 223 of RM
 * */

#define GPIOA_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR &= ~(1 << 0));}while(0)		//don't add ";"
#define GPIOB_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 4)); (RCC->RCC_AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 5)); (RCC->RCC_AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 6)); (RCC->RCC_AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 7)); (RCC->RCC_AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1 << 8)); (RCC->RCC_AHB1RSTR &= ~(1 << 8));}while(0)

#define GPIO_BASE_ADDR_TO_CODE(x) ( (x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOF) ? 5 : \
									(x == GPIOG) ? 6 : \
									(x == GPIOH) ? 7 : \
									(x == GPIOI) ? 8 : 0 )

/**************************EXTI register map on page 287 of RM*************************/
typedef struct
{
	volatile uint32_t EXTI_IMR;		//Interrupt mask register 				Address offset: 0x00
	volatile uint32_t EXTI_EMR;		//Event mask register					Address offset: 0x04
	volatile uint32_t EXTI_RTSR;	//Rising trigger selection register		Address offset: 0x08
	volatile uint32_t EXTI_FTSR;	//Falling trigger selection register	Address offset: 0x0C
	volatile uint32_t EXTI_SWIER;	//Software interrupt event register		Address offset: 0x10
	volatile uint32_t EXTI_PR;		//Pending register						Address offset: 0x14

}EXTI_RegDef_t;

//pointer to struct of type EXTI_RegDef_t
#define EXTI 				((EXTI_RegDef_t*)EXTI_BASE_ADDR)

/**************************SYSCFG register map on page 294 of RM*************************/
typedef struct
{
	volatile uint32_t SYSCFG_MEMRMP;	//SYSCFG memory remap register									Address offset: 0x00
	volatile uint32_t SYSCFG_PMC;		//SYSCFG peripheral mode configuration register					Address offset: 0x04
	volatile uint32_t SYSCFG_EXTICR[4];	//Array of SYSCFG external interrupt configuration registers 	Address offset: 0x08 - 0x14
	uint32_t RESERVED1[2];				//need to add RESERVED registers
	volatile uint32_t SYSCFG_CMPCR;		//Compensation cell control register							Address offset: 0x20
	//uint32_t RESERVED2[2];			//Reserved, 0x24-x028
	//volitile uint32_t CFGR			//																Address offset: 0x2C
}SYSCFG_RegDef_t;

//pointer to struct of type SYSCFG_RegDef_t
#define SYSCFG 				((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
/**************************SPI register map on page 925 of RM*************************/
/***********************Reset Value = 0 unless otherwise stated***********************/
/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
typedef struct
{
	volatile uint32_t SPI_CR1;			//Control register 1			(0x00)
	volatile uint32_t SPI_CR2;			//Control registers 2		 	(0x04)
	volatile uint32_t SPI_SR; 			//Status register 				(0x08)				Reset value: 0x0002
	volatile uint32_t SPI_DR; 			//Data register 				(0x0C)
	volatile uint32_t SPI_CRCPR; 		//SPI CRC Polynomial register	(0x10)				Reset value: 0x0007
	volatile uint32_t SPI_RXCRCR; 		//SPI Rx CRC Register			(0x14)
	volatile uint32_t SPI_TXCRCR; 		//SPI Tx CRC Register			(0x18)
	volatile uint32_t SPI_I2SCFGR; 		//I2S Config Register			(0x1C)
	volatile uint32_t SPI_I2SPR;		//I2S Prescaler Register		(0x20)

}SPI_RegDef_t;

//#define SPIx	((SPI_RegDef_t*))

//**************************SPI Peripheral Definitions*************************************/
/**************************SPI Register map on page 925 of RM*************************/
#define SPI1				((SPI_RegDef_t*)SPI_1_BASE_ADDR)
#define SPI2				((SPI_RegDef_t*)SPI_2_BASE_ADDR)
#define SPI3				((SPI_RegDef_t*)SPI_3_BASE_ADDR)
#define SPI4				((SPI_RegDef_t*)SPI_4_BASE_ADDR)
#define SPI5				((SPI_RegDef_t*)SPI_5_BASE_ADDR)
#define SPI6				((SPI_RegDef_t*)SPI_6_BASE_ADDR)

//APB1 Peripherals
#define SPI_2_BASE_ADDR		(APB1_PERIPH_BASE + 0x3800)
#define SPI_3_BASE_ADDR		(APB1_PERIPH_BASE + 0x3C00)

//APB2 Peripherals - SPI
#define SPI_1_BASE_ADDR		(APB2_PERIPH_BASE + 0x3000)
#define SPI_4_BASE_ADDR		(APB2_PERIPH_BASE + 0x3400)
#define SPI_5_BASE_ADDR		(APB2_PERIPH_BASE + 0x5000)
#define SPI_6_BASE_ADDR		(APB2_PERIPH_BASE + 0x5400)

//Clock Enable/Disable Macros for SPIx Peripherals
#define SPI_1_PCLK_EN()		(RCC->RCC_APB2ENR |= (1 << 12))
#define SPI_2_PCLK_EN()		(RCC->RCC_APB1ENR |= (1 << 14))
#define SPI_3_PCLK_EN()		(RCC->RCC_APB1ENR |= (1 << 15))
#define SPI_4_PCLK_EN()		(RCC->RCC_APB2ENR |= (1 << 13))

#define SPI_1_PCLK_DI()		(RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI_2_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI_3_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1 << 15))
#define SPI_4_PCLK_DI()		(RCC->RCC_APB2ENR &= ~(1 << 13))

/*SPI Register Disable Macros*/
#define SPI_1_REG_RESET()	(RCC->RCC_APB2RSTR |= (1 << 12))
#define SPI_2_REG_RESET()	(RCC->RCC_APB1RSTR |= (1 << 14))
#define SPI_3_REG_RESET()	(RCC->RCC_APB1RSTR |= (1 << 15))
#define SPI_4_REG_RESET()	(RCC->RCC_APB2RSTR |= (1 << 13))

 /* Bit Position Definitions of the SPI Peripheral */

/*SPI_CR1*/
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BAUD_R3		3
#define SPI_CR1_BAUD_R4		4
#define SPI_CR1_BAUD_R5		5
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15

/*SPI_CR2*/
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXNEIE		7

/*SPI_SR*/
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRC_ERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
/**************************I2C Register map on page 872 of RM*************************/
/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

typedef struct
{
	volatile uint32_t I2C_R1;			//Control register 1			(0x00)
	volatile uint32_t I2C_CR2;			//Control registers 2		 	(0x04)
	volatile uint32_t I2C_OAR1;			//Own Address Register 1		(0x08)								(0x08)
	volatile uint32_t I2C_OAR2;			//Own Address Register			(0x0C)
	volatile uint32_t I2C_DR; 			//Data register 				(0x10)
    volatile uint32_t I2C_SR1; 			//Status register 				(0x14)				Reset value: 0x0000
    volatile uint32_t I2C_SR2; 			//Status register 				(0x18)				Reset value: 0x0000
	volatile uint32_t I2C_CCR;	 		//I2C CRC Polynomial register	(0x1C)				Reset value: 0x0000
	volatile uint32_t I2C_TRISE; 		//I2C Rx CRC Register			(0x20)				Reset value: 0x2
	volatile uint32_t I2C_FLTR; 		//I2C Tx CRC Register			(0x24)
}I2C_RegDef_t;


#define I2C1RST				21
#define I2C2RST				22
#define I2C3RST				23

#define I2C1EN				I2C1RST
#define I2C2EN				I2C2RST
#define I2C3EN				I2C3RST

/*I2Cx Base Addresses*/
#define I2C1_BASE_ADDR		(APB1_PERIPH_BASE + 5400)
#define I2C2_BASE_ADDR		(APB1_PERIPH_BASE + 5800)
#define I2C3_BASE_ADDR		(APB1_PERIPH_BASE + 5C00)

/*Pointers to I2Cx Base Addresses*/
#define I2C1 				((I2C_RegDef_t*)(I2C1_BASE_ADDR))
#define I2C2 				((I2C_RegDef_t*)(I2C2_BASE_ADDR))
#define I2C3 				((I2C_RegDef_t*)(I2C3_BASE_ADDR))

/*I2Cx Clock Enable/Disable Macros*/
#define I2C_1_PCLK_EN()		(RCC->RCC_APB1ENR |= (1 << I2C1EN))
#define I2C_2_PCLK_EN()		(RCC->RCC_APB1ENR |= (1 << I2C2EN))
#define I2C_3_PCLK_EN()		(RCC->RCC_APB1ENR |= (1 << I2C3EN))

#define I2C_1_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1 << I2C1EN))
#define I2C_2_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1 << I2C2EN))
#define I2C_3_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1 << I2C3EN))

/*I2Cx Register Reset Macros*/
#define I2C_1_REG_RESET		(RCC->RCC_APB1RSTR =| (1 << I2C1RST))
#define I2C_2_REG_RESET		(RCC->RCC_APB1RSTR =| (1 << I2C2RST))
#define I2C_3_REG_RESET		(RCC->RCC_APB1RSTR =| (1 << I2C3RST))

/* Bit Position Definitions of the SPI Peripheral */

/* I2C_CR1 */
#define PE				0
#define SMBU			1
#define RESERVED2		2
#define SMBTYPE			3
#define ENARP			4
#define ENPEC			5
#define ENGC			6
#define NO_STRETCH		7
#define START			8
#define STOP			9
#define ACK				10
#define POS				11
#define PEC				12
#define ALERT			13
#define RESERVED14      14
#define SWRST			15

/*I2C_CR2*/
#define FREQ0			0
#define FREQ1			1
#define FREQ2			2
#define FREQ3			3
#define FREQ4			4
#define FREQ5			5
#define ITERREN			8
#define ITEVTEN			9
#define ITBUFEN			10
#define DMAEN			11
#define LAST			12

/*I2C_OAR1*/
#define ADD_0 			0
#define ADD1			1
#define ADD2			2
#define ADD3			3
#define ADD4			4
#define ADD5			5
#define ADD6			6
#define ADD7			7
#define ADD8			8
#define ADD9			9
#define ADD_MODE		15

/*I2C_OAR2*/
#define ENDUAL			0
#define ADD2_1			1
#define ADD2_2			2
#define ADD2_3			3
#define ADD2_4			4
#define ADD2_5			5
#define ADD2_6			6
#define ADD2_7			7

/*I2C_SR1*/
#define SB				0
#define ADDR			1
#define BTF				2
#define ADD10			3
#define STOPF			4
#define RxNE			6
#define	TxE				7
#define	BERR			8
#define	ARLO			9
#define	AF				10
#define	OVR				11
#define	PEC_ERR			12
#define TIMEOUT			13
#define SMB_ALERT		15

/*I2C_SR2*/

/*I2C_CCR*/
#define CCR0			0
#define CCR1			1
#define CCR2			2
#define CCR3			3
#define CCR4			4
#define CCR5			5
#define CCR6			6
#define CCR7			7
#define CCR8			8
#define CCR9			9
#define CCR10			10
#define CCR11			11
#define DUTY			14
#define FS				15

/*Peripheral Interrupt Requests*/
/* MCU1 - 110 - 111
 *  IRQ (Interrupt Request) Number on STM32F407x MCU
 *  Listed in position column on pg 373 in RM
 *  NOTE: Update these macros with valid values according to you MCU
 *  TODO: Complete this list for other peripherals
 *  */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI5_9			23
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_EXTI10_15		40

/* MCU1 - 114
 *  Processor side - NVIC IRQ (Interrupt Request) Priority
 *  Use this when assigning an interrupt to a GPIO pin
 *  Use this especially when you're assigning an interrupt
 *  to multiple GPIO pins. The lower the number the more
 *  important it is
 *
 *  */
#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO2		2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4		4
#define NVIC_IRQ_PRIO5		5
#define NVIC_IRQ_PRIO6		6
#define NVIC_IRQ_PRIO7		7
#define NVIC_IRQ_PRIO8		8
#define NVIC_IRQ_PRIO9		9
#define NVIC_IRQ_PRIO10		10
#define NVIC_IRQ_PRIO11		11
#define NVIC_IRQ_PRIO12		12
#define NVIC_IRQ_PRIO13		13
#define NVIC_IRQ_PRIO14		14
#define NVIC_IRQ_PRIO15		15

//Clock Enable Macros for USARTx Peripherals
#define USART_2_PCLK_EN()	(RCC->RCC_APB1ENR |= (1 << 17))
#define USART_3_PCLK_EN()	(RCC->RCC_APB1ENR |= (1 << 18))
#define UART_4_PCLK_EN()	(RCC->RCC_APB1ENR |= (1 << 19))
#define UART_5_PCLK_EN()	(RCC->RCC_APB1ENR |= (1 << 20))
#define USART_1_PCLK_EN()	(RCC->RCC_APB2ENR |= (1 << 4))
#define USART_6_PCLK_EN()	(RCC->RCC_APB2ENR |= (1 << 5))

#define EXTI_PCLK_EN()		(EXTI->RCC_APB1ENR |= (1 << 14))
#define SYSCFG_PCLK_EN()	(RCC->RCC_APB2ENR |= (1 << 14))

#endif /* INC_STM32F407G_H_ */
