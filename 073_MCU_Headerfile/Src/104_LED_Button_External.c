/*
 * 103_LED_Button.c
 *
 *  Created on: May 17, 2021
 *      Author: njacobs
 *
 *      uses on-board LED connected to PD12
 *
 *      uses external push button connected to PB12
 *      when button is pressed near side pins (A & B the pins that are closer to one another)
 *
 *      When not pressed the far side pins, the buttons with the push button in between
 *      them are connected
 *
 *      See OneNote notes
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
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//Enable the clock for GPIOD
	GPIO_PeriClkControl(GPIOD, ENABLE);

	// Call API (stm32f407g_gpio_driver.c), send address of
	// GPIO_Handle gpio_led
	GPIO_Init(&gpio_led);



	//******************GPIOB*****************************************//

	// Configure the GPIOB for the BUTTON
	// set !APPLICABLE! GPIO_PinConfig members appropriately
	gpio_btn.pGPIOx = GPIOB;										//point the handle at GPIO port A
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_INPUT;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;			//to use the on-board 40k PUR
																	//if we don't use PUR, PB12 is
																	//floating

	//Enable the clock for GPIOD
	GPIO_PeriClkControl(GPIOB, ENABLE);

	//Call API, send address of GPIO_Handle gpio_btn
	GPIO_Init(&gpio_btn);

	//enter infinite loop
	while(1){

		// Read the status of the GPIOB input register for the status of PB12
		// REMEMBER when the external button is PRESSED, PB12 should be LOW
		// REMEMBER when the external button is NOT PRESSED, PB12 should be high

		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_NOT_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}
	return 0;
}

void EXTI0_IRQHandler(void)
{
	/* call the GPIO_IRQHandler (provide pinNumber you're expecting interrupt from)
	 * located in stm32f407g_gpio_driver.c. Application needs to provide PinNumber
	 *
	 *	*/
	GPIO_IRQHandling(2);

}
