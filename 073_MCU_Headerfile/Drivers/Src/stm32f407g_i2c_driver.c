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

/* Private fuction - don't need to add to i2c_driver.h*/
uint8_t I2C_GetFlag_Status(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDR_Flag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
/*
 * * @fn
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]  */
 uint8_t I2C_GetFlag_Status(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
 {
	 if(!(pI2Cx->I2C_SR1 & FlagName))
		 {
			 return FLAG_RESET;
		 }
		 else
		 {
			 return FLAG_SET;
		 }
  }

/* Generates the START condition by setting the START bit in I2C_CR1
 * * @fn
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]  */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	 /* value << location */
	 pI2Cx->I2C_CR1 |= (SET <<  I2C_CR1_START);

}

/* @fn
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]  */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	/*1. Since the slave address is only 7-bits
	 * Shift the SlaveAddr over one space to make room for the r/w bit  0th bit index */
	SlaveAddr = (SlaveAddr << 1);

	/*2. Clear the 0th bit*/
	SlaveAddr &= ~(1);

	/*3. Write the address to the I2C_DR*/
	pI2Cx->I2C_DR = SlaveAddr;
}

/* @fn - Clears the ADDR bit in I2C_SR1 by reading reading SR1 register followed reading SR2
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]  */
static void I2C_ClearADDR_Flag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead =pI2Cx->I2C_SR1;
	dummyRead =pI2Cx->I2C_SR2;

	/* add "(void)" otherwise the complier will give you an error
	 * in the video Kiran initially makes dummyRead uint16_t and
	 * then changes it to uint32. If it was uint16 is the typecast (void)
	 * necessary?*/
	(void)dummyRead;
}

/*
 * * @fn
 *
 * @brief - Stop generation after the current byte transfer or
 * after the current Start condition is sent.
 *
 * Possibly waits until all the bit are shifted out of the shift register
 *
 * @param[in]
 * @param[in]
 * @param[in]*/
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	/* value << location */
		 pI2Cx->I2C_CR1 |= (SET <<  I2C_CR1_STOP);
}

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
	 * which are now SWS1 and SWS0
	 * SWS: System clock switch status
		Set and cleared by hardware to indicate which clock source is used as the system clock.
		00: HSI oscillator used as the system clock
		01: HSE oscillator used as the system clock
		10: PLL used as the system clock
		11: not applicable*/
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

	/* NOW we need to determine the current value of the AHB Prescaler
	 * Read the current value of the RCC_CFGR bits 7:4 HPRE
	 * See page 167 of the RM*/
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

	/* NOW we need to determine the current value of the APB1 Prescaler
	 * Read the current value of the RCC_CFGR bits 12:10 PPRE1
	 * Shift to the far right, and clear everything after the first
	 * 3 bits
	 * See page 166 of the RM
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

	 I2C_PeriClkControl(pI2CHandle->pI2Cx, ENABLE);

	 uint32_t tempreg = 0;

	 /*ACK Control Bit
	  * REVIEW THIS
	  * Value << Location
	  *  Not sure why we need to read the value into temp*/
	 tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	 pI2CHandle->pI2Cx->I2C_CR1 = tempreg;

	 /* Clear tempreg out since you're done configuring I2C_CR1*/
	 tempreg = 0;

	 /* FREQ bits configuration in I2C_CR2
	 	  * 16MHz
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
	 tempreg = (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);

	 /* RM says this must always be set to 1, see page 864 */
	 tempreg |= (1 << 14);

	 /* Apply changes to I2C_OAR1*/
	 pI2CHandle->pI2Cx->I2C_OAR1 = (tempreg & 0xFE);

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

	 /* TRISE Calculation
	  * See page 871  in the RM
	  * FPCLK1 x Trise(MAX) + 1
	  * FPCLK1 see  line 138 above
	  * Trise(MAX) see page 44 of the I2C spec, this is the max Trise time allowed
	  * for a given I2C speed i.e. stnadard mode, fast mode, etc)*/
		 if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		 {
			 /* Standard Mode */
			 tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;

		 }
		 else
		 {
			 /*Fast Mode*/
			 tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000U) + 1 ;
		 }
		 pI2CHandle->pI2Cx->I2C_TRISE = (tempreg & 0x3F);

		 tempreg =0;

		 /*Setting PE bit  in I2C_CR1*/
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

 /*  */
 void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr)
 {
	 /*Setting PE bit  in I2C_CR1*/
	 pI2CHandle->pI2Cx->I2C_CR1 |= 0x1;

	 /*1. Generate the START condition*/
	 I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/* 2. Confirm that the START condition has been generated by checking the
	 * SB bit in I2C_SR1
	 * NOTE: Until SB is cleared, the SCL will be stretched (pulled LOW) */
	 while(!I2C_GetFlag_Status(pI2CHandle->pI2Cx, I2C_FLAG_SR1_SB));

	 /*3. Send the address of the slave with the r/w bit set to w (0) (total 8 bits)  */
	 I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	 /*4. Confirm that the address phase is complete by checking the ADDR flag in the I2C_SPI register*/
	 while(!I2C_GetFlag_Status(pI2CHandle->pI2Cx, I2C_FLAG_SR1_ADDR));

	 /*5. Clear the ADDR bit according to its SW (code) sequence
	  * NOTE: Until ADDR is cleared, the SCL will be stretched (pulled LOW) */
	 I2C_ClearADDR_Flag(pI2CHandle->pI2Cx);

	 /*6. Send data until length == 0
	  * Remember to confirm whether the data register is empty
	  * or not by checking the TxE bit/flag
	  * TxE == 0 means DR is not empty
	  * TxE == 1 means DR is empty */

	 while(Len > 0)
	 {
		 /* If I2C_GetFlag_Status returns 0 that means that the DR is not empty
		  *  (currently transferring to the shift register) and we want to wait until
		  *  it's empty to  send data, so we add the "!" so the code inside the while
		  *  loop executes
		  *
		  * If I2C_GetFlag_Status returns 1, that means that  the DR is empty and
		  * we want to send data
		  *
		  * Lecture 196, 00:00:12 check this again I think he has it backwards
		  * the follow code is what it should be */
		 while( I2C_GetFlag_Status(pI2CHandle->pI2Cx, I2C_FLAG_SR1_TxE));
		 pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		 pTxBuffer++;
		 Len--;

		 /* 7. When Len == 0, wait for TxE == 1 & BTF == 1 before generating the
		  * STOP condition.
		  * NOTE: When TxE & BTF == 1, that means both the shift register (SR) & DR
		  * are empty and the next transmission should begin. When BTF ==1 the SCL
		  * will be stretched
		  *
		  * Do you need to add the " ! "? */
		 while( I2C_GetFlag_Status(pI2CHandle->pI2Cx, I2C_FLAG_SR1_TxE));
		 while( I2C_GetFlag_Status(pI2CHandle->pI2Cx, I2C_FLAG_SR1_BTF));


		 /* 8. Generate STOP condition and master need not to wait for the completion of
		  * the STOP condition.
		  * NOTE: Generating STOP, automatically clears the BTF*/
		 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
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

/* Application Callback - used for interrupt-based API*/
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t App_Event)
{

 }
