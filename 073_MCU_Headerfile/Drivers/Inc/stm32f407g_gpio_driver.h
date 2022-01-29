/*
 * stm32f407g_gpio_driver.h
 *
 *  Created on: April 18, 2021
 *      Author: njacobs
 *
*    This is a configuration structure for a GPIO pin
 */


#include "stm32f407g.h"
//********************************************************************************************************************************
//*******************************Handle & Configuration Structs*******************************************************************
//

// Settings to configure a GPIO port
typedef struct
{
	uint8_t GPIO_PinNumber;						// possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;							// possible values from @GPIO PIN_MODES (below)
	uint8_t GPIO_PinSpeed;							// possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;				// possible values from @GPIO_PinPuPdControl
	uint8_t GPIO_PinOPType;						// possible values from @GPIO_PinOPType
	uint8_t GPIO_PinAltFuncMode;				// possible values from @GPIO_PinAltFunMode
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;						//this holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; 		//this holds GPIO pin configuration settings
}GPIO_Handle_t;


//*******************************************************************************************************************************
//**********************************GPIO Register Macros**************************************************************************
/********************************************************************************************************************************
 * @GPIO_PIN_NUMBERS
 * GPIO Pin Numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 * @GPIO PIN_MODES
 * GPIO Possible Pin Modes
 * */
#define GPIO_MODE_INPUT			0		// When in input mode, pin can deliver interrupt
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALT_FUNC		2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IN_FED		4		//input mode with falling edge detection
#define GPIO_MODE_IN_RED		5		//input mode with rising edge detection
#define GPIO_MODE_IN_RFED		6		//input mode with rising edge falling edge detection

/*
 * @GPIO_PinOPType
 * GPIO Output Types
 */
#define GPIO_OUTPUT_TYPE_PP		0		//reset state
#define GPIO_OUTPUT_TYPE_OD		1

/*
 *@GPIO_PIN_SPEED
 * GPIO Pin Output Speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MED			1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_VERY_FAST	3

/*
 * @GPIO_PinPuPdControl
 * GPIO Pullup/Pulldown
 */
#define GPIO_NO_PUPD			0
#define GPIO_PU					1
#define GPIO_PD					2

/*
 * GPIO Alternate Function Low/High
 */
#define GPIO_ALT_FUNC_AF0		0x0
#define GPIO_ALT_FUNC_AF1		0x1
#define GPIO_ALT_FUNC_AF2		0x2
#define GPIO_ALT_FUNC_AF3		0x3
#define GPIO_ALT_FUNC_AF4		0x4
#define GPIO_ALT_FUNC_AF5		0x5
#define GPIO_ALT_FUNC_AF6		0x6
#define GPIO_ALT_FUNC_AF7		0x7
#define GPIO_ALT_FUNC_AF8		0x8
#define GPIO_ALT_FUNC_AF9		0x9
#define GPIO_ALT_FUNC_AF10		0xA
#define GPIO_ALT_FUNC_AF11		0xB
#define GPIO_ALT_FUNC_AF12		0xC
#define GPIO_ALT_FUNC_AF13		0xD
#define GPIO_ALT_FUNC_AF14		0xE
#define GPIO_ALT_FUNC_AF15		0xF

//***********************************************************************
//							APIs Supported by this driver
//***********************************************************************


//Peripheral Clock Setup - enable/disable peripheral clk for a given GPIO base address
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);

/*Init & De-Init
* init the GPIO port and pin; takes pointer to a handle struct
* user application should create a variable of this type
* (GPIO_Handle_t *pGPIOHandle), initialize it and send the
* pointer of that variable to the GPIO Init
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/* takes the GPIO base address that is to be reset
 * Set RCC_AHB1RSTR register
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Data Read & Write
 * Read from GPIO input pin
 * takes base address of GPIO port and pin number
 * returns 0 | 1
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
																																						//
/*Takes pointer to base address, reads entire GPIO
 * input port, returns port value
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/* Write value to output pin
 * Input parameters:
 * Pointer to GPIO base address
 * Pin number
 * value to be written to pin (1/0)
* Return value:
* 	None */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t val);

/*Input parameters:
 	* Pointer to GPIO base address
	* 16-bit value to be written to port
* Return value:
	* None */
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

/*Input parameters:
 * Pointer to GPIO base address
 * Pin number
 * value to be written to pin (1/0)
* Return value:
* 	None */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* Input parameters:
 	 *IRQ number
 	 *IRQ Priority
 	 *IRQ en=(1) or di=(0)
 * Return Value:
 * 	 None  */
void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t ENorDI);

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);


/*when interrupt occurs main can call this to process the interrupt handler code
 * Input Parameters:
 	 *uint8_t pin number the interrupt occurred on
 *Return value:
 	 *None */
void GPIO_IRQHandling(uint8_t PinNumber);







