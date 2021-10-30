/*
 * 169_SPI_Interrupt_Arduino.c
 *
 *  Created on: Oct 26, 2021
 *      Author: njacobs
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f407g.h"
#include "stm32f407g_spi_driver.h"
#include "stm32f407g_gpio_driver.h"

/* LED States */
#define LED_OFF					0
#define LED_ON					1

/* Command Codes - each 1-byte of data*/
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

/*SPI Command Responses
 * 'ACK' conflicts with an I2C_CR1
 * bit position definition in
 * stm32f407g.h (line 426)*/
#define NACK 					0xA5		// 165 in decimal
#define ACKN					0xF5		// 245 in decimal

/*Arduino Analog Pins*/
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

/*Arduino LED */
#define LED_PIN					9

/*Dummy byte to receive ACK or NACK from slave*/
#define DUMMY_BYTE				0xFF


#define	MSG_1_SIZE				12

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


void SPI2_GPIO_Init()
{
	/* Create the handle to configure GPIO PORTB*/
	GPIO_Handle_t SPIPins;

	/*Now you need to fill in all this information
	 * typedef struct
	{
		uint8_t GPIO_PinNumber;			// possible values from @GPIO_PIN_NUMBERS
		uint8_t GPIO_PinMode;			// possible values from @GPIO PIN_MODES (below)
		uint8_t GPIO_PinSpeed;			// possible values from @GPIO_PIN_SPEED
		uint8_t GPIO_PinPuPdControl;	// possible values from @GPIO_PinPuPdControl
		uint8_t GPIO_PinOPType;			// possible values from @GPIO_PinOPType
		uint8_t GPIO_PinAltFuncMode;	// possible values from @GPIO_PinAltFunMode
	}GPIO_PinConfig_t;

	typedef struct
	{
		GPIO_RegDef_t *pGPIOx;				//this holds the base address of the GPIO port to which the pin belongs
		GPIO_PinConfig_t GPIO_PinConfig; 	//this holds GPIO pin configuration settings
	}GPIO_Handle_t;*/

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUNC;			//2
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = GPIO_ALT_FUNC_AF5;		//5
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;		//0
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;			//0
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;				//2

	/*MOSI - PB15*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;				//15
	GPIO_Init(&SPIPins);

	/*MISO - PB14*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	/*SCLK - PB13*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	/*NSS - PB12*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Init()
{
	/*Create a SPI handle, then fill in the data:
	 *
	 *  typedef struct
			 {
				 SPI_RegDef_t	*pSPIx; base address of SPIx(x:0,1,2)
				 SPI_Config_t	SPIConfig;
		}SPI_Handle_t;

		 typedef struct
		 {
			 uint8_t SPI_DeviceMode;
			 uint8_t SPI_BusConfig;
			 uint8_t SPI_SCLK_Speed;
			 uint8_t SPI_DFF;
			 uint8_t SPI_CPOL;
			 uint8_t SPI_CPHA;
			 uint8_t SPI_SSM;
		 }SPI_Config_t;*/

	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL;
	SPI2_Handle.SPIConfig.SPI_SCLK_Speed = SPI_SCLK_SPEED_DIV_8; 	/*Generates SCLK of 2 MHz*/
	SPI2_Handle.SPIConfig.SPI_DFF = SPI_DFF_8_Bits;
	SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; 					/* HW Slave Management*/

	/*this use to be SPI2_Init Apply settings*/
	SPI_Init(&SPI2_Handle);
}

void GPIO_ButtonInit()
{

	printf("In GPIO_Init()");
	/* create GPIO port handle to interface with button
	 GPIO_Handle_t
	 	 GPIO_RegDef_t *pGPIOx
	 	 GPIO_PinConfig_t GPIO_PinConfig */
	GPIO_Handle_t GPIO_btn;

	/* USER button is connected to PA0 on discovery board
	 set !APPLICABLE! GPIO_PinConfig members to configure
	 as INPUT PIN
	 GPIO_PinSpeed & GPIO_PinOPType are used when pin is
	 set to OUTPUT mode only*/
	GPIO_btn.pGPIOx = GPIOD;										//point the handle at GPIO port D
	GPIO_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;			//point the handle at pin 6
	GPIO_btn.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_INPUT;
	GPIO_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* Call API (stm32f407g_gpio_driver.c), send address of
	 GPIO_Handle GPIO_btn*/
	GPIO_Init(&GPIO_btn);
}

uint8_t SPI_Verify_Response(uint8_t *ack_byte)
{
	if (*ack_byte == ACKN)
	{
		/*Valid command*/
		return 1;
	}
	else
	{
		/*Invalid Command*/
		return 0;
	}
}

void Send_Slave_Commands(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	/*used to keep track of what command to send next
	can't be destroyed when function is done*/
	//uint8_t const cmd_count = 0;

	/* case 3 modifiable data and constant pointer
	use this to update the value of cmd_count*/
	//uint8_t *ptr_cmd_count = (uint8_t*)&cmd_count

	uint8_t cmd_code[5] = {COMMAND_LED_CTRL, COMMAND_SENSOR_READ, COMMAND_LED_READ, COMMAND_PRINT, COMMAND_ID_READ};

	/* Rx & Tx Buffers*/
	uint8_t ack_byte, dummy_read, analog_read, led_status, cmd_args[2];
	uint8_t dummy_write = DUMMY_BYTE;
	char const *msg_1 = "Hello World";			/* string literal - stored in ROM, never loaded in RAM*/
	char board_id[16];


	/*	1. if SPIx is disabled...*/
	if(SPI_GetFlag_Status(pSPIx, SPI_SPE_FLAG) == RESET)
	{
		/* 5. ENABLE the SPIx peripheral SPE bit in the CR*/
		SPI_Peripheral_Control(SPI2, ENABLE);

	}

	/* Hang here until next button press*/
	while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));


}

int main(void)
{
	/*Create GPIOD Handle for interrupt pin PD6*/
	GPIO_Handle_t GPIO_PD6_interrupt;

	/* PD6 will be used to receive an alert signal (high to low pulse)
	 * from the arduino when it has received new data from its
	 * serial terminal. In turn, PD6 triggers an interrupt
	 *
	 * Set !APPLICABLE! GPIO_PinConfig members to achieve this:
	 * uint8_t GPIO_PinNumber;
	 * uint8_t GPIO_PinMode;
	 * uint8_t GPIO_PinSpeed;
	 * uint8_t GPIO_PinPuPdControl;
	 * uint8_t GPIO_PinOPType;
	 * uint8_t GPIO_PinAltFuncMode;*/

	/*TODO: why does PD6 need to configured this way? */
	GPIO_PD6_interrupt.pGPIOx = GPIOD;										//point the handle at GPIO port D
	GPIO_PD6_interrupt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_PD6_interrupt.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_IN_FED;
	GPIO_PD6_interrupt.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_PD6_interrupt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	/* To apply the above settings call
	 * GPIO_Init(GPIO_Handle_t *pGPIOHandle)
	 * in stm32f407g_gpio_driver.c and send address of
	 * GPIO_Handle GPIO_PD6_interrupt*/
	GPIO_Init(&GPIO_PD6_interrupt);

	/* 5. Configure the NVIC registers */
	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI5_9, ENABLE);

	/* 6. Set interrupt priority */
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI5_9, NVIC_IRQ_PRIO15);

/* 1. Select what SPIx peripheral you want to use
 *
* 	ALT Function Mode (page 63 of datasheet):
* 	1) PB12 -> NSS
	2) PB13 -> SCLK
	3) PB14 -> MISO
	4) PB15 -> MOSI
 *
 * 2. Create a function that sets the selected GPIO pins to their desired SPI alt
 * 	  functions ( SPI2_SPIO_Init() )*/
	SPI2_GPIO_Init();

/* 3. Initialize the SPI2 Peripheral*/
	SPI2_Init();

/* 4. Enable SSOE Bit in SPI_CR2,
 * this enables the NSS output
 * When SSM = 0, the NSS pin is managed by the HW
 * i.e.
 * when we set SPE = 1, HW will pull the NSS pin low for us
 * when we set SPE = 0, HW pulls the NSS pin high for us
 * this is how HW manages the NSS pin
 * */
	SPI_SSOE_Config(SPI2, ENABLE);

	while(1)
	{
		/* Wait here for the high-to-low signal from the arduino*/
		while(1);

		/*9. Before disabling the SPI peripheral make sure its not
		 * transmitting data
		 *
		 * if SPI_GetFlag_Status returns 1 program will hang here
		 *
		 * otherwise it'll disable SPI2 */
		while(SPI_GetFlag_Status(SPI2, SPI_BUSY_FLAG));

		/*Might cause problems - not sure how*/
		SPI_Peripheral_Control(SPI2, DISABLE);

		return 0;
	}
}

/*Interrupt Handler that calls the IRQ_Handler APIb*/
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
}
