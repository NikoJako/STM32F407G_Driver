/*
 * 010_i2c_master_tx_testing.c
 *
 *  Created on: Dec 16, 2022
 *      Author: njacobs
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407g.h"
#include "stm32f407g_i2c_driver.h"
#include "stm32f407g_gpio_driver.h"

#define MY_ADDR					0x94
#define SLAVE_ADDR				0x68
#define LOW								0
#define HIGH 							1
#define BTN_PRESSED				HIGH
#define BTN_NOT_PRESSED	LOW

/*the accompanying Arduino sketch is written for the Wire Library.
 * THis library has a limit of 32-bytes that can be transferred i
 * in a single I2C transaction*/
uint8_t some_data[] = "We are testing I2C master Tx\n";

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

I2C_Handle_t I2C1_Handle;
//I2C_Handle_t *pI2C1_Handle;

/*PB6--> SCL
 * PB9--> SDA
 *
 * */

void I2C1_GPIO_Init()
{
	/* Create the handle to configure GPIO PORTB*/
	GPIO_Handle_t I2C_Pins;

	I2C_Pins.pGPIOx = GPIOB;

	I2C_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUNC;					//2
	I2C_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;				//1
	I2C_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;								//1
	I2C_Pins.GPIO_PinConfig.GPIO_PinAltFuncMode = GPIO_ALT_FUNC_AF4;		//4
	I2C_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;							//2

	/*SCL*/
	I2C_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2C_Pins);

	/*SDA*/
	I2C_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2C_Pins);

}

/*remember that I2CHandle_t *I2CHandle is an address
 * so the calling argument (from main.c) should be
 * I2C_Init(&something) */
void I2C1_Init(void)
{
	/*enable the clock*/
	//I2C_PeriClkControl(I2C1_Handle.pI2Cx, ENABLE);
	/*Create an I2C handle, then fill in the data:
	 *
	 *  typedef struct
			 {
				 I2C_RegDef_t     *pI2Cx;				//pointer of type I2CRegDef_t; I2CRegDef_t pointer
				I2C_Config_t		I2C_Config;
			}I2C_Handle_t;

				typedef struct
					{
						uint32_t I2C_SCLSpeed;
						uint8_t I2C_DeviceAddress;
						uint8_t I2C_ACKControl;
						uint8_t I2C_FMDutyCycle;
					}I2C_Config_t;*/

	I2C1_Handle.pI2Cx = I2C1;
	I2C1_Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1_Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1_Handle);


	/*this use to be I2C2_Init Apply settings*/
	//I2C_Init(&I2C1_Handle);
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

int main(void)
{
	GPIO_ButtonInit();

	/* I2C pin  inits*/
	I2C1_GPIO_Init();

	/*I2C peripheral configuration*/
	I2C1_Init();

	/*Enable the I2C Peripheral*/
//	I2C_Peripheral_Control(I2C1, ENABLE);

	/*Wait for button press*/
	//enter infinite loop
		while(1){

			// Read the status of the input register
			// use API

			if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
			{
				delay();

				I2C_MasterSendData(&I2C1_Handle, some_data, strlen((char *)some_data), SLAVE_ADDR);
			}

			/*Send Data*/


		}


	/*Chill here when done sending*/



	/**/

	/**/

	/**/

}

