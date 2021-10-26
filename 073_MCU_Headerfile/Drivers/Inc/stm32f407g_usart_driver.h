/*
 * stm32f407g_usart_driver.h
 *
 *  Created on: Oct 23, 2021
 *      Author: njacobs
 */

#ifndef INC_STM32F407G_USART_DRIVER_H_
#define INC_STM32F407G_USART_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f407g.h"

/***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 USART & UART Configuration & Handle Structs  ****************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 Configuration Struct*******************************************************************
 ***************************************************************************************
 */
 typedef struct
 {
	 uint8_t USART_Mode;
	 uint32_t USART_BaudRate;
	 uint8_t USART_NumOfStopBits;
	 uint8_t USART_WordLength;
	 uint8_t USART_ParityControl;
	 uint8_t USART_HW_FlowControl;
 }USART_Config_t;

 /****************************************************************************************
  USART & UART Handle Struct*******************************************************************
  ****************************************************************************************/
 typedef struct
 {
	 USART_RegDef_t	*pUSARTx; 			/*This holds the base address of USARTx*/
	 USART_Config_t	USART_Config;

 }USART_Handle_t;

 /*******************************************************************************************************************************************************************************
   *										APIs Supported By This Driver
   * 					For More Information About the APIs see the Function definitions
   * ******************************************************************************************************************************************************************************
   */

  //Peripheral Clock Setup - enable/disable peripheral clk for a given USART base address
  void USART_PeriClkControl(USART_RegDef_t *pUSARTx, uint8_t ENorDI);

  /*Init & De-Init
  *
  */
  void USART_Init(USART_Handle_t *pUSARTHandle);

  /*
   *
   */
  void USART_DeInit(USART_Handle_t *pUSARTHandle);

  /*when interrupt occurs main can call this to process the interrupt handler code
   * Input Parameters:
   	 *
   *Return value:
   	 *None  */
  uint8_t USART_GetFlag_Status(USART_RegDef_t *pUSARTx, uint32_t FlagName);

 void USART_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t ENorDI);

 void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);

  /*when interrupt occurs main can call this to process the interrupt handler code
   * Input Parameters:
   	 *
   *Return value:
   	 *None */
  void USART_IRQHandling(USART_Handle_t *pHandle);


  /* Other Peripheral Control APIs*/
 void USART_Peripheral_Control(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
 void USART_Clear_Flag(USART_RegDef_t *pUSARTx);

#endif /* INC_STM32F407G_USART_DRIVER_H_ */
