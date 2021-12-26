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


/*Dummy byte to receive ACK or NACK from slave*/
#define MAX_LEN 		500

/* Global Variables */

SPI_Handle_t SPI2_Handle;

volatile uint8_t rcvStop = 0;

char RcvBuff[MAX_LEN];

char ReadByte;

/* this will be set in the interrupt handler
 * of the Arduino Interrupt GPIO ( from the lecture,
 * not sure what that means) */
volatile uint8_t dataAvailable = 0;

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

void SPI2_Init(SPI_Handle_t *pSPI2_Handle)
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

	pSPI2_Handle->pSPIx = SPI2;
	pSPI2_Handle->SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	pSPI2_Handle->SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL;
	pSPI2_Handle->SPIConfig.SPI_SCLK_Speed = SPI_SCLK_SPEED_DIV_8; 	/*Generates SCLK of 2 MHz*/
	pSPI2_Handle->SPIConfig.SPI_Enable = DISABLE;
	pSPI2_Handle->SPIConfig.SPI_CRC_EN = DISABLE;
	pSPI2_Handle->SPIConfig.SPI_RX_ONLY = SPI_RXONLY_FULL_DUPLEX;
	pSPI2_Handle->SPIConfig.SPI_SSM = SPI_SSM_DI; 					/* HW Slave Management*/
	pSPI2_Handle->SPIConfig.SPI_SSI = SPI_SSI_HIGH;
	pSPI2_Handle->SPIConfig.SPI_LSB_FIRST = SPI_MSB_SENT_FIRST;
	pSPI2_Handle->SPIConfig.SPI_DFF = SPI_DFF_8_Bits;
	pSPI2_Handle->SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	pSPI2_Handle->SPIConfig.SPI_CPHA = SPI_CPHA_LOW;


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

void Slave_GPIO_InterruptPin_Init()
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

		/* Configure the NVIC registers */
		GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI5_9, ENABLE);

		/* Set interrupt priority */
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI5_9, NVIC_IRQ_PRIO15);
}


int main(void)
{


	uint8_t dummyByte = 0xFF;

	/* initializes PD6 */
	Slave_GPIO_InterruptPin_Init();

	/* 1. Select what SPIx peripheral you want to use
	 *
	* 	ALT Function Mode (page 63 of datasheet):
	* 	1) PB12 -> NSS
		2) PB13 -> SCLK
		3) PB14 -> MISO
		4) PB15 -> MOSI
	 *
	 * 2. Create a function that sets the selected GPIO pins to their desired SPI alt
	 * 	  functions  SPI2_SPIO_Init() )*/
	SPI2_GPIO_Init();

	/* 3. Initialize the SPI2 Peripheral*/
	SPI2_Init(&SPI2_Handle);

	/* 4. Enable SSOE Bit in SPI_CR2,
	 * this enables the NSS output
	 * When SSM = 0, the NSS pin is managed by the HW
	 * i.e.
	 * when we set SPE = 1, HW will pull the NSS pin low for us
	 * when we set SPE = 0, HW pulls the NSS pin high for us
	 * this is how HW manages the NSS pin
	 * */
	SPI_SSOE_Config(SPI2, ENABLE);

	/* Enable RXEIE interrupt to trigger whenever
	 * data is received*/
	SPI_IRQ_Interrupt_Config(IRQ_NO_SPI2, ENABLE);

	while(1)
	{
		rcvStop = 0;

		/* Wait here until data is available to be read
		 * from the Arduino */
		while(!dataAvailable);

		/* First code executed after EXTI9_5_IRQHandler(void) ISR
		 *
		 * Now that the master knows there is data available
		 * it disables the interrupt on PD6 until the data
		 * has been received from the slave device */
		GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI5_9, DISABLE);

		/* Enable SPI2 in preparation of reading data sent from
		 * the Arduino slave device*/
		SPI_Peripheral_Control(SPI2, ENABLE);

		while(!rcvStop)
		{

			/* Fetch the data from the SPI peripheral
			 * byte by byte in the interrupt mode
			 *
			 * SendData_IT set TxState to SPI_BUSY_IN_TX and sets TXEIE
			 * Once TXEIE is set it will trigger an interrupt when TXE == 1
			 * ReceiveData_IT sets RxState to SPI_BUSY_IN_RX and sets RXNEIE
			 * Once RXNEIE is set, it will trigger an interrupt once RXNE == 1
			 * RXNE == 1 means that the receive buffer isn't empty, i.e. data
			 * has arrived */
			while(SPI_SendData_IT(&SPI2_Handle, &dummyByte, 20) == SPI_BUSY_IN_TX);
			while (SPI_ReceiveData_IT(&SPI2_Handle, &ReadByte, 20) == SPI_BUSY_IN_RX);

		}

		/* Confirm SPI isn't busy */
		while(SPI_GetFlag_Status(SPI2, SPI_BUSY_FLAG));

		/* Disable the SPI2 peripheral */
		SPI_Peripheral_Control(SPI2, DISABLE);

		dataAvailable = 0;

		GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI5_9, ENABLE);

	}
}

/* When PD6 receives the high to low signal from the
 * Arduino, and data is available an interrupt is
 * triggered and this ISR is ran
 * Interrupt Handler that calls the IRQ_Handler APIb*/
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}

void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2_Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	static uint32_t i = 0;

	/* When AppEv == SPI_EVENT_RX_CMPLT,
	 * copy data into the rcv buffer,
	 * " \0 " indicates the end of the message (rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || (i == MAX_LEN))
		{
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i = 0;
		}
		printf("Rcvd data: %s\n" , RcvBuff);
	}
}
