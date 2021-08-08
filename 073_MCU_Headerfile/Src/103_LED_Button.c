/*
 * 103_LED_Button.c
 *
 *  Created on: May 17, 2021
 *      Author: njacobs
 */


#include "stm32f407g.h"
#include "stm32f407g_gpio_driver.h"

#define LOW				0
#define HIGH 			1
#define BTN_PRESSED		HIGH
#define BTN_NOT_PRESSED	LOW

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void){

	// create GPIO port handle
	// GPIO_Handle_t includes a GPIO_RegDef_t type pointer
	// and a GPIO_PinConfig_t type struct that includes pin settings
	// Create GPIO port handle to interface with button
	GPIO_Handle_t gpio_led, gpio_btn;

	// Configure the GPIOB for the LED
	// set !APPLICABLE! GPIO_PinConfig members
	gpio_led.pGPIOx = GPIOD;										//point the handle at GPIO port D
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_OUTPUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	//Enable the clock for GPIOD
	GPIO_PeriClkControl(GPIOD, ENABLE);

	// Call API (stm32f407g_gpio_driver.c), send address of
	// GPIO_Handle gpio_led
	GPIO_Init(&gpio_led);



	//******************GPIOB*****************************************//

	// Configure the GPIOB for the BUTTON
	// set !APPLICABLE! GPIO_PinConfig members
	gpio_btn.pGPIOx = GPIOA;										//point the handle at GPIO port A
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_INPUT;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//Enable the clock for GPIOA
	GPIO_PeriClkControl(GPIOA, ENABLE);

	//Call API, send address of GPIO_Handle gpio_btn
	GPIO_Init(&gpio_btn);

	//enter infinite loop
	while(1){

		// Read the status of the input register
		// use API

		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}



	}


	return 0;
}

