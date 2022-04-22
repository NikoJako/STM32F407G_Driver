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

uint16_t AHB_Prescalar[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_Prescalar[4] = {2, 4, 8, 16};

/*Function to calculate the PLL clock value
 * * @fn
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]*/
uint32_t RCC_GetPLLOutputClock(void)
{

	uint32_t empty;
	/* Dummy function - not implemented */
	return empty;
}
/* Determines the current system clock source by reading the SWSx[3:2] bits
 * in  the RCC_CFGR register. See page 167 in RM
 * @fn
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]
 * */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, apb_prescalar, ahb_prescalar;

	/* Right shifts the contents of RCC_CFGR over twice
	 * and then clears everything but bits 0 & 1
	 * which are now SWS1 and SWS0 */
	clksrc = (RCC->RCC_CFGR >> 2) & 0x3;

	if (clksrc == 0)
	{
		SystemClk = 16000000;
	}
	else if (clksrc == 1)
	{
		SystemClk = 8000000;
	}
	else if (clksrc == 2)
	{
		/* Dummy function - not implemented */
		SystemClk = RCC_GetPLLOutputClock();
	}

	/* Read the current value of the RCC_CFGR bits 7:4 HPRE
	 * See page 167 of the RM  */
	temp  = (RCC->RCC_CFGR >> 4) & 0xF;

	/*   */
	if (temp < 8 )
	{
		ahb_prescalar = 1;
	}
	else
	{
		/* This determines the current system clock prescaler value
		 * e.g. When temp == 8, then the system clock is divided by 2
		 * in this case,  the HPRE  bits == 8
		 * 1000: system clock divided by 2
		 *1001: system clock divided by 4
		 *1010: system clock divided by 8
		 *1011: system clock divided by 16
		 *1100: system clock divided by 64
		 *1101: system clock divided by 128
		 *1110: system clock divided by 256
		 *1111: system clock divided by 512*/
		ahb_prescalar = AHB_Prescalar[temp - 8];
	}

	/* Read the current value of the RCC_CFGR bits 12:10 PPRE1
	 * Shift to the far right, and clear everything after the first
	 * 3 bits
	 * See page 166 of the RM
	 *
	   */
		temp  = (RCC->RCC_CFGR >> 10) & 0x7;

		/*  0xx: AHB clock not divided  */
		if (temp < 4 )
		{
			apb_prescalar = 1;
		}
		else
		{
			/* This determines the current AHB prescaler value
			 * e.g. When temp == 5, then the AHB  is divided by 4
			 * in this case,  the PPRE1 bits == 5
			 * 100: AHB clock divided by 2
			 * 101: AHB clock divided by 4
			 *	110: AHB clock divided by 8
			 *	111: AHB clock divided by 16*/
			apb_prescalar = APB_Prescalar[temp - 4];
		}

		pclk1 = ((SystemClk / ahb_prescalar) / apb_prescalar);

	return pclk1;
}
/*Peripheral Clock Setup - enable/disable peripheral clk for a given I2C base address
 * * @fn
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * */
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

	 /*ACK Control Bit
	  * REVIEW THIS
	  * Value << Location
	  *  Not sure why we need to read the value into temp*/
	 tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);

	 /* FREQ bits configuration in I2C_CR2
	  * 16MHz */
	 tempreg = 0;

	 /* returns uint32_t , divide by 1M to remove trailing 0s e.g. "16" */
	 tempreg |= RCC_GetPCLK1Value() / 1000000U;

	 /* 0x3F = 0011 1111, this is to clear out everything but the first 6 bits
	  * of tempreg to ensure that only the FREQ bits are
	  * being updated in I2C_CR2 */
	 pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);

	 /* OWN ADDRESS configuration
	  * we shift to the left once because we're using a 7-bit address
	  * the ADD0  bit is used for a 10-bit address ONLY
	  * 0xFE = 1111 1110 - In the lecture Kiran didn't do the 0xFE mask */
	 tempreg = 0;
	 tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);

	 /* RM says this must always be set to 1, see page 864 */
	 tempreg |= (1 << 14);

	 /* Apply changes to I2C_OAR1*/
	 pI2CHandle->pI2Cx->I2C_OAR1 |= (tempreg & 0xFE);

	 /* CCR Calculations */
	 uint16_t ccr_value = 0;
	 tempreg = 0;

	 /* Checking the Serial Clock Speed*/
	 if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	 {
		 /* Get/Calculate Standard Mode CCR value*/
		 ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);

		 /*Program  CCR value
		  * ccr_value is uint16_t but we only need the first 12-bits
		  * see page 870 in RM - 0000 1111 1111 1111 = 0xFFF */
		 tempreg |= (ccr_value & 0xFFF);
	 }
	 else
	 {
		 /*Fast Mode*/

		 /*Set F/S Master Mode bit */
		 tempreg |= (SET << FS);

		 /* Fast Mode Duty Cycle
		  * Supplied by the application/user/ I2C_Config_t*/
		 tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << DUTY);

		 /*Get/Calculate Fast Mode CCR Value*/
		 if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		 {
			 ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		 }
		 else
		 {
			 ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		 }

		 /* Program the CCR Value*/
		 tempreg |= (ccr_value & 0xFFF);
	 }

		 pI2CHandle->pI2Cx->I2C_CCR = tempreg;






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
