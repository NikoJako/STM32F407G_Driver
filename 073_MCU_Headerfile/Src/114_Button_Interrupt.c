/*
 * 114_Button_Interrupt
 *
 *  Created on: June 13, 2021
 *      Author: njacobs
 *
 *      TO-DO - Update this, this is from 104_LED_Button_Extrernal.c
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

#include<string.h>
#include "stm32f407g.h"
#include "stm32f407g_gpio_driver.h"

#define LOW				0
#define HIGH 			1
#define BTN_PRESSED		HIGH
#define BTN_NOT_PRESSED	LOW

void delay(void)
{
	/* this will create approx 200ms delay when the system clock is 16MHz
	 * (internal RC oscillator)*/
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void){

	// create GPIO port handle
	// GPIO_Handle_t includes a GPIO_RegDef_t type pointer
	// and a GPIO_PinConfig_t type struct that includes pin settings
	// Create GPIO port handle to interface with button
	GPIO_Handle_t gpio_led, gpio_btn;



	memset(&gpio_led, 0, sizeof(gpio_led));
	memset(&gpio_btn, 0, sizeof(gpio_btn));

	// Configure the GPIOD for the LED
	// set !APPLICABLE! GPIO_PinConfig members
	gpio_led.pGPIOx = GPIOD;										//point the handle at GPIO port D
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_OUTPUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//Enable the clock for GPIOD
	GPIO_PeriClkControl(GPIOD, ENABLE);

	// Call API (stm32f407g_gpio_driver.c), send address of
	// GPIO_Handle gpio_led
	GPIO_Init(&gpio_led);



	//******************GPIOD - Button Pin - *****************************************//

	/* Configure the GPIOD for the BUTTON to be connected to PD5
	 * set !APPLICABLE! GPIO_PinConfig members appropriately
	 *
	 * Since interrupt should be triggered on falling edge PinMode needs to be
	 * updated to GPIO_MODE_IN_FED
	 * */
	//
	gpio_btn.pGPIOx = GPIOD;										//point the handle at GPIO port D
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_IN_FED;		//interrupt falling-edge
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;			//to use the on-board 40k PUR
																	//if we don't use PUR, PB5 is
																	//floating

	//Enable the clock for GPIOD
	GPIO_PeriClkControl(GPIOD, ENABLE);

	//Call API, send address of GPIO_Handle gpio_btn
	GPIO_Init(&gpio_btn);

	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_RESET);

	/* IRQ Configuration and priority for PD5
	 * What's the IRQNumber & EXTI line PD5 is associated with?
	 * 23 & EXTI9_5
	 * page 373 of the RM
	 * */
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI5_9, NVIC_IRQ_PRIO15);
	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI5_9, ENABLE);

	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{

	/*Debounce delay*/
	delay();

	/* call the driver supplied GPIO interrupt handler "GPIO_IRQHandler"
	 * (provide pinNumber you're expecting interrupt from)
	 * located in stm32f407g_gpio_driver.c. Application needs
	 * to provide PinNumber
	 *	*/
	GPIO_IRQHandling(GPIO_PIN_NO_5);

	//Toggle the LED
	//GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);

}
