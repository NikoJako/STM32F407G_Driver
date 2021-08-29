/*
 * 157_CMD_Handling.c
 *
 *  Created on: Aug 16, 2021
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
#define COMMAND_IDREAD			0x54

/*SPI Command Responses
 * 'ACK' conflicts with an I2C_CR1
 * bit position definition in
 * stm32f407g.h (line 426)*/
#define NACK 					0xA5
#define ACKN					0xF5

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
	GPIO_btn.pGPIOx = GPIOA;										//point the handle at GPIO port A
	GPIO_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
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

int main(void)
{

/* Configure and enable a GPIOD port to handle the button press
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
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));


		/*Add delay to address debouncing*/
		delay();

		/* 5. ENABLE the SPI2 peripheral SPE bit in the CR*/
		SPI_Peripheral_Control(SPI2, ENABLE);



		/*Sending Commands*/



		/* 6. Create TxBuffer & RxBuffer to store command code & response from
		 * slave
		 *
		 * REMEMBER if DFF == 16-bits
		 * cmd_code & slave_response needs to be uint16_t or
		 * typecasted to uint16_t*/
		uint8_t cmd_code = COMMAND_LED_CTRL;
		uint8_t slave_response;
		uint8_t cmd_args[2];
		uint8_t dummy_write;
		uint8_t dummy_read;

		/* 7. Send COMMAND_LED_CTRL  <pin number> <value>
		 *
		 * void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
		 * remember that pTxBuffer == uint8_t
		 *
		 * REMEMBER if DFF == 16-bits
		 * cmd_code needs to be uint16_t or typecasted to uint16_t
		 * and number of bytes sent a.k.a 'len' needs to be '2'*/
		SPI_SendData(SPI2, &cmd_code, 1);

		/*Dummy read to clear RXNE register
		 * whenever data is sent to a slave device,
		 * 1-byte is returned to the SPI master
		 * when it reaches the master, the RXNE bit will be set
		 * to clear a "dummy read" is performed to clear the RXNE bit*/
		SPI_ReceiveData(SPI2, &dummy_read,1);

		/* When the slave receives this command
		 * it checks if the command is valid
		 * if valid, slave returns ACK
		 * if invalid, slave returns NACK
		 * the slave can't initiate communication
		 * therefore a dummy byte needs to be sent to receive the slave'e reply
		 * update TxBuffer with dummy byte:*/
		dummy_write = DUMMY_BYTE;

		/*Send dummy byte to fetch response from slave*/
		SPI_SendData(SPI2, &dummy_write, 1);

		/*Receive response from slave*/
		SPI_ReceiveData(SPI2, &slave_response,1);

		/*Determine if the command sent to slave was valid or not
		 *
		 * If SPI_Verify_Response (&slave_response) returns '1'
		 * then the sent command was valid (ACK)
		 * and we want to enter the if statement
		 * to send the command arguments*/
		if (SPI_Verify_Response(&slave_response))
		{
			/*Send command arguments in array
			 * 'turn the LED pin on the arduino on' */
			cmd_args[0] = LED_PIN;
			cmd_args[1] = LED_ON;

			SPI_SendData(SPI2, &cmd_args[0], 2);
		}

		/* Wait here for button press */
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));



		/* @To-Do create function to:
		 * Send command
		 * Dummy Read to clear RXNE register
		 * Send dummy byte to fetch response from slave
		 * Receive Response from slave
		 * Verify Response from slave*/


		/* 8. Send COMMAND_SENSOR_READ:*/

		/* Assign Send COMMAND_SENSOR_READ command code to pTxBuffer */
		cmd_code = COMMAND_SENSOR_READ;

		/* Send COMMAND_SENSOR_READ over the wire*/
		SPI_SendData(SPI2, &cmd_code, 1);

		/*Dummy read to clear RXNE register*/
		SPI_ReceiveData(SPI2, &dummy_read,1);

		/*Send dummy byte to fetch response from slave*/
		SPI_SendData(SPI2, &dummy_write, 1);

		/*Receive response from slave*/
		SPI_ReceiveData(SPI2, &slave_response,1);

		if (SPI_Verify_Response(&slave_response))
		{
			/*Send COMMAND_SENSOR_READ argument
			 * analog pin number = ANALOG_PIN0*/
			cmd_args[0] = ANALOG_PIN0;

			/*Send Arguments*/
			SPI_SendData(SPI2, &cmd_args[0], 2);
		}

		/* Receive Sensor Data from Slave */

		/* Create RxBuffer */
		uint8_t analog_read;

		/*Dummy read to clear RXNE register*/
		SPI_ReceiveData(SPI2, &dummy_read,1);

		/*Send dummy byte to fetch response from slave*/
		SPI_SendData(SPI2, &dummy_write, 1);

		/*Receive response from slave*/
		SPI_ReceiveData(SPI2, &analog_read,1);






			/*	3. COMMAND_LED_READ		0x52
				4. COMMAND_PRINT			0x53
				5. COMMAND_IDREAD			0x54*/




		//SPI_SendData(SPI2, &dataLen, 1);
		//SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		/*8. Receive Data*/


		/*9. Before disabling the SPI peripheral make sure its not
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
