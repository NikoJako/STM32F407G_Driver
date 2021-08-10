/*
 * 150_SPI_TxOnly_Arduino_Slave.c
 *
 *  Created on: Aug 5, 2021
 *      Author: njacobs
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f407g.h"
#include "stm32f407g_spi_driver.h"
#include "stm32f407g_gpio_driver.h"

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

	/*MOSI*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;				//15
	GPIO_Init(&SPIPins);

	/*MISO*/
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

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
	/* create GPIO port handle to interface with button
	 GPIO_Handle_t
	 	 GPIO_RegDef_t *pGPIOx
	 	 GPIO_PinConfig_t GPIO_PinConfig */
	GPIO_Handle_t GPIO_btn;

	// Configure the GPIOB for the LED
	// set !APPLICABLE! GPIO_PinConfig members
	GPIO_btn.pGPIOx = GPIOD;										//point the handle at GPIO port D
	GPIO_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_btn.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_OUTPUT;
	GPIO_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
	GPIO_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	//Enable the clock for GPIOD
	GPIO_PeriClkControl(GPIOD, ENABLE);

	// Call API (stm32f407g_gpio_driver.c), send address of
	// GPIO_Handle GPIO_btn
	GPIO_Init(&GPIO_btn);
}

int main(void)
{

/*remember that user_data == uint8_t* */
	char user_data[] = "hello world";

	/* Configure and enable a GPIO port to handle the button press
	 * data is to be sent only when the button is pressed*/
	GPIO_ButtonInit();

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
		/* We want the program to hang here until is the button is pressed.
		 * When pressed the while expression computes to 0 and the program
		 * jumps over the while loop and sends the data
		 *
		 * GPIO_ReadFromInputPin will return:
		 * 	1 if the button has been pressed
		 * 	0 if not
		 * */
		while(!(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_0)));


		/*Add delay to address debouncing*/
		delay();

		/* 5. ENABLE the SPI2 peripheral SPE bit in the CR*/
		SPI_Peripheral_Control(SPI2, ENABLE);

		/*First send length information*/
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		/* 6. Send Data
		 * void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
		 * remember that user_data == uint8_t
		 * since we're using strlen we need to include string.h*/
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		/*Receive Data*/


		/*Before disabling the SPI peripheral make sure its not
		 * transmitting data
		 *
		 * if SPI_GetFlag_Status returns 1 program will hang here
		 *
		 * otherwise it'll disable SPI2 */
		while(SPI_GetFlag_Status(SPI2, SPI_SR_BSY));

		/*Might cause problems*/
		SPI_Peripheral_Control(SPI2, DISABLE);
	}



	return 0;
}
