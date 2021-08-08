/*
 * 101_LED_Toggle.c
 *
 *  Created on: May 4, 2021
 *      Author: njacobs
 */

#include "stm32f407g.h"
#include "stm32f407g_gpio_driver.h"

void delay(void)
{
	for(uint32_t i = 0; i < 1000000; i++);
}

int main(void){

	// create GPIO port handle
	// GPIO_Handle_t includes a GPIO_RegDef_t type pointer
	// and a GPIO_PinConfig_t type struct that includes pin settings
	GPIO_Handle_t gpio_led;

	//point the handle at GPIO port D
	gpio_led.pGPIOx = GPIOD;

	//Configure the port, set !APPLICABLE! GPIO_PinConfig members
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_OUTPUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	//Enable the clock for GPIOD
	GPIO_PeriClkControl(GPIOD, ENABLE);

	//Call API, send address of GPIO_Handle gpio_led
	GPIO_Init(&gpio_led);

	//enter infinite loop
	while(1){

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();

	}


	return 0;
}
