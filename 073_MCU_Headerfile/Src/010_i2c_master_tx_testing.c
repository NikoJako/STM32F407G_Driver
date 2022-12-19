/*
 * 010_i2c_master_tx_testing.c
 *
 *  Created on: Dec 16, 2022
 *      Author: njacobs
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407g.h"


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

/*PB6--> SCL
 * PB9--> SDA
 *
 * */

void I2C1_GPIO_Init()
{
	/* Create the handle to configure GPIO PORTB*/
	GPIO_Handle_t I2C_Pins;

	I2CPins.pGPIOx = GPIOB;

	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUNC;				//2
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;			//1
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;								//1
	I2CPins.GPIO_PinConfig.GPIO_PinAltFuncMode = GPIO_ALT_FUNC_AF4;		//4
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;							//2
}

void I2C1_Init()
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

	I2C1_Handle_t I2C1_Handle;

	I2C12_Handle.pI2C1x = I2C12;
	I2C12_Handle.I2C1Config.I2C1_DeviceMode = I2C1_DEVICE_MODE_MASTER;
	I2C12_Handle.I2C1Config.I2C1_BusConfig = I2C1_BUS_CONFIG_FULL;
	I2C12_Handle.I2C1Config.I2C1_SCLK_Speed = I2C1_SCLK_SPEED_DIV_8; 	/*Generates SCLK of 2 MHz*/
	I2C12_Handle.I2C1Config.I2C1_DFF = I2C1_DFF_8_Bits;
	I2C12_Handle.I2C1Config.I2C1_CPOL = I2C1_CPOL_LOW;
	I2C12_Handle.I2C1Config.I2C1_CPHA = I2C1_CPHA_LOW;
	I2C12_Handle.I2C1Config.I2C1_SSM = I2C1_SSM_DI; 					/* HW Slave Management*/

	/*this use to be I2C2_Init Apply settings*/
	I2C_Init(&I2C1_Handle);
}

void GPIO_ButtonInit()
{

	printf("In GPIO_ButtonInit()");
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

int main(void)
{


}
}
