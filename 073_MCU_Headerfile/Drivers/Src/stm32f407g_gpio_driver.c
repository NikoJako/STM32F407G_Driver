/*
 * stm32f407g_gpio_driver.c
 *
 *  Created on: Apr 18, 2021
 *      Author: njacobs
 */

#include <stdio.h>
#include "stm32f407g.h"
#include "stm32f407g_gpio_driver.h"
#include "stm32f407g_spi_driver.h"


/*Peripheral Clock Setup - enable/disable peripheral clk for a given GPIO base address
 * @fn			- GPIO_PeriClkControl
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- ENABLE or DISABLE macros
 * @param[in]
 *
 * @return		- none
 *
 * @note		- none
*/
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if (pGPIOx == GPIOA){

			GPIOA_PCLK_EN();
		}
		else if	(pGPIOx == GPIOB){

			GPIOB_PCLK_EN();
		}
		else if	(pGPIOx == GPIOC){

			GPIOC_PCLK_EN();
		}
		else if	(pGPIOx == GPIOD){

			GPIOD_PCLK_EN();
		}
		else if	(pGPIOx == GPIOE){

			GPIOE_PCLK_EN();
		}
		else if	(pGPIOx == GPIOF){

			GPIOF_PCLK_EN();
		}
		else if	(pGPIOx == GPIOG){

			GPIOG_PCLK_EN();
		}
		else if	(pGPIOx == GPIOH){

			GPIOH_PCLK_EN();
		}
		else if	(pGPIOx == GPIOI){

			GPIOI_PCLK_EN();
		}
	}
	else{

		if (pGPIOx == GPIOA){

			GPIOA_PCLK_DI();
		}
		else if	(pGPIOx == GPIOB){

			GPIOB_PCLK_DI();
		}
		else if	(pGPIOx == GPIOC){

			GPIOC_PCLK_DI();
		}
		else if	(pGPIOx == GPIOD){

			GPIOD_PCLK_DI();
		}
		else if	(pGPIOx == GPIOE){

			GPIOE_PCLK_DI();
		}
		else if	(pGPIOx == GPIOF){

			GPIOF_PCLK_DI();
		}
		else if	(pGPIOx == GPIOG){

			GPIOG_PCLK_DI();
		}
		else if	(pGPIOx == GPIOH){

			GPIOH_PCLK_DI();
		}
		else if	(pGPIOx == GPIOI){

			GPIOI_PCLK_DI();
		}
	}
}


/*Init & De-Init
* init the GPIO port and pin; takes pointer to a handle struct
* user application should create a variable of this type
* (GPIO_Handle_t *pGPIOHandle), initialize it and send the
* pointer of that variable to the GPIO Init
* mode GPIO Port Mode Register
* speed
* output type
* PU/PD
* Alternate function
* Don't use assignment operator, use bitwise OR
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	/* Enable the clock for the GPIOD peripheral*/
	GPIO_PeriClkControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;							//temp register

		/*#define GPIO_MODE_INPUT		0		// When in input mode, pin can deliver interrupt
		#define GPIO_MODE_OUTPUT		1
		#define GPIO_MODE_ALT_FUNC		2
		#define GPIO_MODE_ANALOG		3
		#define GPIO_MODE_IN_FED		4		//input mode with falling edge detection
		#define GPIO_MODE_IN_RED		5		//input mode with rising edge detection
		#define GPIO_MODE_IN_RFED		6		//input mode with rising edge falling edge detection
	 * 1. Mode, if value of the RHS is <= 3, the
	 mode is non-interruptible
	*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		/* multiplied by 2 because each pin is represented by two bit fields
		 * in the GPIO Mode Register. If PinNumber == 1, then 2 * 1 = 2 (MODER1),
		 * the value will be stored at indexes 2 & 3*/
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		//clear required bit-fields
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		/*Set the MODER register
		 * 00: Input (reset state)
		   01: General purpose output mode
		   10: Alternate function mode
		   11: Analog mode */
		pGPIOHandle->pGPIOx->MODER |= temp;

		temp = 0;
	}
	else
	{
		/* These modes are interruptible therefore
		 * we must configure the edge detection
		 *
		 * FED - falling edge detection
		 * RED - rising edge detection
		 * FTSR - falling trigger selection register
		 * RTSR - rising trigger selection register
		 * */

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FED)
		{
			//input mode with falling edge detection
			//configure the FTSR pg 385 in RM
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear the corresponding bit in the RTSR (rising trigger selection register)
			//(do as a safe measure)
			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RED)
		{
			// configure the bit in the RTSR
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear the corresponding bit in the FTSR (do as a safe measure)
			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RFED)
		{
			// configure the bit in the RTSR
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// configure the bit in the FTSR
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
	}

		/*2. Assign the GPIO pin to the appropriate EXTI line
		 * using SYSCFG_EXTICR[4]:
		 *
		 * temp1 is the index of SYSCFG_EXTICR[4] that is needs to be
		 * configured. Based on the pin number this tells you what
		 * SYSCFG_EXTICRx register needs to be configured
		 *
		 * temp2 is the 4 bits within the above selected
		 * SYSCFG_EXTICR[temp1] register that needs to be set */
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		/*TODO - Write an explanation for this*/
		uint8_t portcode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx);

		/* Enable the SYSCFG clock */
		SYSCFG_PCLK_EN();

		/*Configure the appropriate SYSCFG_EXTICR register for
		 * given pin number*/
		SYSCFG->SYSCFG_EXTICR[temp1] = portcode << (temp2 * 4);

		//3. Enable the EXTI interrupt delivery using the interrupt mask register (IMR)
		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		//2. OSPEEDR - Speed
	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	 //clear required bit-fields
	 pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	 /*set the OSPEEDR register
	  * 00: Low speed
		01: Medium speed
		10: High speed
		11: Very high speed*/
	 pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	 temp = 0;

	 //4. OTYPER - configure output type
	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	 //clear required bit-fields
	 pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	 /*OTYPER - Set the output type:
	  * 0: Output push-pull (reset state)
		1: Output open-drain
	  * */
	 pGPIOHandle->pGPIOx->OTYPER |= temp;

	 temp = 0;


	//3. PUPDR - Configure the pull-up/pull-down settings into temp
	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	 //clear required bit-fields
	 pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	 /*PUPDR - Set the PUPDR register
	  * 00: No pull-up, pull-down
		01: Pull-up
		10: Pull-down
		11: Reserved*/
	 pGPIOHandle->pGPIOx->PUPDR |= temp;
	 temp = 0;

	 //5. configure the alt functionality, for this to mean anything, PinMode must be set to AltFunc mode
	 /* temp1 calculation determines what alt function register (High|Low) needs to be configured
	  * temp2 determines the starting bit index in the alt function register, that the GPIO_PinConfig.GPIO_PinAltFuncMode
	  * value is applied to. Should correspond to GPIO_PinNumber
	  * */
	 if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FUNC)
	 {
		 uint8_t temp1, temp2;

		 temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		 temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);

		 //clearing the AFR register before setting
		 pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));

		 //setting appropriate ARF bits
		 pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2));
	 }
}

/* takes the GPIO base address that is to be reset
 * Set RCC_AHB1RSTR register, page 223 of RM
 * See stm32f407g for macro definition
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

		if (pGPIOx == GPIOA){

			GPIOA_REG_RESET();
		}
		else if	(pGPIOx == GPIOB){

			GPIOB_REG_RESET();
		}
		else if	(pGPIOx == GPIOC){

			GPIOC_REG_RESET();
		}
		else if	(pGPIOx == GPIOD){

			GPIOD_REG_RESET();
		}
		else if	(pGPIOx == GPIOE){

			GPIOE_REG_RESET();
		}
		else if	(pGPIOx == GPIOE){

			GPIOE_REG_RESET();
		}
		else if	(pGPIOx == GPIOF){

			GPIOF_REG_RESET();
		}
		else if	(pGPIOx == GPIOG){

			GPIOG_REG_RESET();
		}
		else if	(pGPIOx == GPIOH){

			GPIOH_REG_RESET();
		}
		else if	(pGPIOx == GPIOI){

			GPIOI_REG_RESET();
		}

}

/*Data Read & Write
 * Read from GPIO input pin
 * takes base address of GPIO port and pin number
 * returns 0 | 1
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*Takes pointer to base address, reads entire GPIO
 * input port, returns port value
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/* Write value to output pin
 * Input parameters:
 * Pointer to GPIO base address
 * Pin number
 * value to be written to pin (1/0)
* Return value:
* 	None */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t val)
{
	if(val == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//clearing provided pinNumber
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/*Input parameters:
 	* Pointer to GPIO base address
	* 16-bit value to be written to port
* Return value:
	* None */
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/*Input parameters:
 * Pointer to GPIO base address
 * Pin number
 * value to be written to pin (1/0)
* Return value:
* 	None
* XOR Truth Table
*   0, 0 = 0
*   0, 1 = 1
*   1, 0 = 1
*   1, 1 = 0 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/* Configure the NVIC registers on the Cortex M4
 * Input parameters:
 	 *IRQ number
 	 *IRQ Priority
 	 *IRQ EN=(1) or DIS=(0)
 * Return Value:
 * 	 None  */
void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if (IRQNumber <= 31 )
		{
			/* Configure the ISER0 register:
			 * this register enables interrupts and shows which interrupts are enabled
			 * Dereference and set appropriate value...the IRQNumber
			 */
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 64 )
		{
			/* Configure ISER1 register
			 * If the IRQNumber is greater 31 we need to determine
			 * the bit position to set in ISER1 without referring to
			 * the RM:
			 *
			 * 			IRQNumber % 32
			 *
			 * E.g. IRQNumber = 50:
			 *
			 * 50 % 32 => 18 <= (1 18/32)
			 * */
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber > 64 && IRQNumber <= 96 )
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	}
	else //Disable therefore Interrupt Clear Enable Register
	{
		if (IRQNumber <= 31)
		{
			/* Configure the ICER0 register:
			 * this register enables interrupts and shows which interrupts are enabled
			 * Dereference and set appropriate value...the IRQNumber
			 */
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 64 )
		{
			/* Configure ICER1 register
			 * If the IRQNumber is greater 31 we need to determine
			 * the bit position to set in ISER1 without referring to
			 * the RM:
			 *
			 * 			IRQNumber % 32
			 *
			 * E.g. IRQNumber = 50:
			 *
			 * 50 % 32 => 18 <= (1 18/32)
			 * */
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber > 64 && IRQNumber <= 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}

	}

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. determine the IPR register
	uint8_t iprx = IRQNumber / 4;

	//2. determine the priority section (field)
	uint8_t iprx_section = IRQNumber % 4;

	//3. jumping to IPR by adding iprx to the priority register base address
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx)  |= (IRQPriority << shift_amount);
}


/*when interrupt occurs main can call this to process the interrupt handler code
 * Input Parameters:
 	 *uint8_t pin number the interrupt occurred on
 *Return value:
 	 *None */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	/* clear the EXTI pending register corresponding to the pinNumber
	 * if EXTI of PR bit position corresponding to this bit position is set,
	 * then the interrupt is reading pending, therefore we clear
	 * */

	if(EXTI->EXTI_PR & (1 << PinNumber))
	{
		//clear - in the Cortex Generic UG, writing a 1 to the PR clears it
		EXTI->EXTI_PR |= (1 << PinNumber);
	}

	uint8_t inc_msg;
	uint32_t msg_len = 500;

	SPI_ReceiveData(SPI2, &inc_msg, msg_len);

}

