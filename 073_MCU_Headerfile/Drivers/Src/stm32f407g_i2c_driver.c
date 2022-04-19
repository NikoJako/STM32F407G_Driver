/*
 * stm32f407g_i2c_driver.c
 *
 *  Created on: Jan 28, 2022
 *      Author: njacobs
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f407g.h"
#include "stm32f407g_i2c_driver.h"


/*Peripheral Clock Setup - enable/disable peripheral clk for a given I2C base address*/
 void I2C_PeriClkControl(I2C_RegDef_t *pI2Cx, uint8_t En_Reset_Or_DI)
 {
	 /*Same thing for GPIO & SPI, see page 265 & 174 respectively of RM */
	 if (En_Reset_Or_DI == ENABLE)
		 	{
		 		if (pI2Cx == I2C1)
		 		{
		 			I2C_1_PCLK_EN();
		 		}
		 		else if	(pI2Cx == I2C2)
		 		{
		 			I2C_2_PCLK_EN();
		 		}
		 		else
		 		{
		 			I2C_3_PCLK_EN();
		 		}
		 	}
		else if (En_Reset_Or_DI == DISABLE)
		{
			if (pI2Cx == I2C1)
			{
				I2C_1_PCLK_DI();
			}
			else if	(pI2Cx == I2C2)
			{
				I2C_2_PCLK_DI();
			}
			else
			{
				I2C_3_PCLK_DI();
			}
		}
		else
		{
	 	 	 if(pI2Cx == I2C1)
	 	 	 {
	 	 		 I2C_1_REG_RESET();
	 	 	 }
	 	 	 else if (pI2Cx == I2C2)
	 	 	 {
	 	 		 I2C_2_REG_RESET();
	 	 	 }
	 	 	 else
	 	 	 {
	 	 		 I2C_3_REG_RESET();
	 	 	 }
		}
 }

 /* *remember (value << location)*
  *  See the top of page 862 in RM
  *  */
 void I2C_Init(I2C_Handle_t *pI2CHandle)
 {
	 uint32_t tempreg = 0;

 }

 /* */
 void I2C_DeInit(I2C_Handle_t *pI2CHandle)
 {
	 pI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);

	 if(pI2CHandle->pI2Cx == I2C1)
	 {
		 I2C_1_PCLK_DI();
	 }
	 else if (pI2CHandle->pI2Cx == I2C2)
	 {
		 I2C_2_PCLK_DI();
	 }
	 else
	 {
		 I2C_3_PCLK_DI();
	 }

  }

 /* */
 void I2C_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t ENorDI)
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

  }

 void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
 {
	 //1. determine the IPR register
	 	uint8_t iprx = IRQNumber / 4;

	 	//2. determine the priority section (field)
	 	uint8_t iprx_section = IRQNumber % 4;

	 	//3. jumping to IPR by adding iprx to the priority register base address
	 	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	 	*(NVIC_IPR_BASE_ADDR + iprx)  |= (IRQPriority << shift_amount);

  }

 /* Other Peripheral Control APIs*/
void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{

 }

/* */
 uint8_t I2C_GetFlag_Status(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
 {

  }

/* Application Callback - used for interrupt-based API*/
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t App_Event)
{

 }
