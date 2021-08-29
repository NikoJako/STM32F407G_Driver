/*
 * stm32f407g_spi_driver.c
 *
 *  Created on: Jul 13, 2021
 *      Author: njacobs
 */
#include <stdint.h>
#include <stdio.h>
#include "stm32f407g.h"
#include "stm32f407g_spi_driver.h"
 /*******************************************************************************************************************************************************************************
  *										APIs Supported By This Driver
  * 					For More Information About the APIs see the Function definitions
  * ******************************************************************************************************************************************************************************
  */

 //Peripheral Clock Setup - enable/disable peripheral clk for a given SPI base address
 void SPI_PeriClkControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
 {
	 if (ENorDI == ENABLE)
	 	{
	 		if (pSPIx== SPI1)
	 		{
	 			SPI_1_PCLK_EN();
	 		}
	 		else if	(pSPIx== SPI2)
	 		{
	 			SPI_2_PCLK_EN();
	 		}
	 		else if	(pSPIx== SPI3)
	 		{
	 			SPI_3_PCLK_EN();
	 		}
	 		else if	(pSPIx== SPI4)
	 		{
	 			SPI_4_PCLK_EN();
	 		}
	 	}
	else
	{

		if (pSPIx== SPI1)
		{
			SPI_1_PCLK_DI();
		}
		else if	(pSPIx== SPI2)
		{
			SPI_2_PCLK_DI();
		}
		else if	(pSPIx== SPI3)
		{
			SPI_3_PCLK_DI();
		}
		else if	(pSPIx== SPI4)
		{
			SPI_4_PCLK_DI();
	 	}
	}
 }
 /*Init & De-Init
 *
 * user configurable item is SPI_Config_t struct
 *
 *
 */
 void SPI_Init(SPI_Handle_t *pSPIHandle)
 {
	 /* Enable the SPI Peripheral Clock */
	 SPI_PeriClkControl(pSPIHandle->pSPIx, ENABLE);

	 /*configure the SPI_CR1 register:
	  * uint8_t SPI_DeviceMode;
	  	uint8_t SPI_BusConfig;
	 	uint8_t SPI_SCLK_Speed;
	 	uint8_t SPI_DFF;
	 	uint8_t SPI_CPOL;
	 	uint8_t SPI_CPHA;
	 	uint8_t SPI_SSM;

	 	remember when the function receives *pSPIHandle
	 	it's already going to have information in it and
	 	the job of all these functions is to apply the
	 	values to the HW
	  * */
	  uint32_t tempreg;

	  /* SHould we first clear CR1 to rid it of garbage values?
	  	 ANS - no since all the changes are being made to tempreg
	  	 which is initialized to 0. WHen all the changes are made
	  	 we set CR1 = tempreg, overwriting any garbage value CR1
	  	 may already have

	  	 *remember (value << location)*
	  */

	  /*1. DeviceMode*/
	  tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	  /*2. BusConfig
	  	  if full duplex, BIDIMODE is cleared
	   * 0: 2-line unidirectional data mode selected
	   * 1: 1-line bidirectional data mode selected*/
	  if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL)
	  {
		  tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
	  }
	  else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF )
	  {
		  /*if half duplex, BIDIMODE is set*/
		  tempreg |= (1 << SPI_CR1_BIDI_MODE);
	  }
	  else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_RX)
	  {
		  /*If Simplex_Rx_Only, BIDIMODE is cleared and RXONLY is set*/
		  tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
		  tempreg |= (1 << SPI_CR1_RX_ONLY);
	  }

	  /*3. SCLK_Speed  -- (value << location) */
	   tempreg |= pSPIHandle->SPIConfig.SPI_SCLK_Speed << SPI_CR1_BAUD_R3;

	  /*4. DFF */
	   tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	  /*5. CPOL */
	   tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	  /*6. CPHA */
	   tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	  /*7. SSM */
	   tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	   /*7. Apply tempreg to the CR1 register*/
	   pSPIHandle->pSPIx->SPI_CR1 = tempreg;

 }
 /*
  *
  */
 void SPI_DeInit(SPI_Handle_t *pSPIHandle)
 {
	 SPI_PeriClkControl(pSPIHandle->pSPIx, DISABLE);

	 /*Same thing for GPIO, see page 265 of RM */
	 if(pSPIHandle->pSPIx == SPI1)
	 {
		 SPI_1_REG_RESET();
	 }
	 else if (pSPIHandle->pSPIx == SPI2)
	 {
		 SPI_2_REG_RESET();
	 }
	 else if (pSPIHandle->pSPIx == SPI3)
	 {
		 SPI_3_REG_RESET();
	 }
	 else
	 {
		 SPI_4_REG_RESET();
	 }
 }

 uint8_t SPI_GetFlag_Status(SPI_RegDef_t *pSPIx, uint32_t FlagName) // @suppress("No return") uint8_t maskINfo
 {


	 /*
	  * SPI_SR_RXNE			Receive buffer not empty
	  * SPI_SR_TXE			Transmit buffer empty
	  * SPI_SR_CHSIDE		Channel side
	  * SPI_SR_UDR			Under run flag
	  * SPI_SR_CRC_ERR		Under run flag
	  * SPI_SR_MODF			Mode Fault
	  * SPI_SR_OVR			Over run flag
	  * SPI_SR_BSY			Busy flag
	  * SPI_SR_FRE			Frame format error
	  * */

	 /*Transmit buffer empty
	  *
	  * Check TXE flag in SPI_SR
	  * 0 means the transmit buffer is not clear/empty
	  * 1 means the transmit buffer is empty
	  *
	  * If it's not empty (inner bracket-> (!(pSPIx->SPI_SR & (1 << 1)) == 0) == 1,
	  * we want to wait until it is
	  *
	  * If it's empty, (Inner Bracket->(!(pSPIx->SPI_SR & (1 << 1)) == 1) == 0)
	  * no need to wait, need to check the DFF bit
	  *
	  * if tx buffer isn't empty...*/
	 if(!(pSPIx->SPI_SR & FlagName))
	 {
		 return FLAG_RESET;
	 }
	 else
	 {
		 /*Tx buffer is Empty */
		 return FLAG_SET;
	 }


 }

 /* Data Send and Receive
  * pSPIx user-provided pointer to the data to be sent
  * pTxBuffer is used to store the user provided data
  * len is used to tell how many bytes the API should send
  *
  * */
 void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
 {
	 /*While there is data to be sent,
	  * Otherwise there is no data to send*/
	 while(len > 0)
	 {
		 /*Get TXE flag status in CR1
		  * if the Tx buffer isn't empty then wait here for remaining data
		  * to be sent,
		  * otherwise move on to checking the DFF flag
		  *
		  * if something breaks on the SPI peripheral, there is a chance
		  * that the code could hang here permanently - need watchdog
		  * timer */

		 while(SPI_GetFlag_Status(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		 /*Check the DFF value (11th bit in SPI_CR)
		  * 0 means set to 8-bit
		  * 1 means set to 16-bit
		  *
		  * pSPIx->SPI_CR1 & (1 << 11))
		  * 0000100000000000
		  *     1
		  * */
		 if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		 {
			 /*16-bit - CLAIRIFY THIS - Load DR with 1 byte of data
			  * and increment the buffer address
			  *
			  * 1. load the data into the register by type casting
			  * uint8_t pTxBuffer into uint16_t* pointer type - I don't understand that
			  * if you omitted the uint16_t* you would only load 8 bits
			  * into the DR
			  *
			  * Breakdown of *((uint16_t*)pTxBuffer);
			  * "*" is the dereference operator
			  * "(uint16_t*)" is the typecast to uint16_t pointer data type
			  * if I were to change it to uint32_t* the date would then be
			  * uint32_t* data
			  *
			  * */
			 pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);

			 len--;
			 len --;

			 /*type casting and incrementing once is the same incrementing
			  * pTxBuffer twice*/
			 (uint16_t*)pTxBuffer++;

		 }
		 else
		 {
			 /*8-bit - Load DR with 1 byte of data
			  * and increment the buffer address*/
			 pSPIx->SPI_DR = *pTxBuffer;

			 len--;

			 pTxBuffer++;

		 }


	 }
 }
 void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
 {
	 /*While there is data to be received*/
	 while(!(len > 0))
	 {
		 /*If the receive buffer hasn't received anything yet wait here
		  * otherwise check the DFF bit*/
		 while(SPI_GetFlag_Status(pSPIx, SPI_RXNE_FLAG) == RESET);

		 if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		 {
			 /*Move the received data in the SPI_DR into the pRxBuffer */
			 *((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;

			 len--;
			 len --;

			 /*type casting and incrementing once is the same incrementing
			  * pTxBuffer twice*/
			 (uint16_t*)pRxBuffer++;
		 }
		 else
		 {
			 /*8-bit - Move 1 byte of received data in the SPI_DR
			  * into the pRxBuffer and increment the buffer address*/
			  *pRxBuffer = pSPIx->SPI_DR;

			 len--;

			 pRxBuffer++;
		 }
	 }
 }
 void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t ENorDI)
 {

 }
 void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
 {

 }

 /*when interrupt occurs main can call this to process the interrupt handler code
  * Input Parameters:
  	 *
  *Return value:
  	 *None */
 void SPI_IRQHandling(SPI_Handle_t *pHandle)
 {

 }

 /* Other Peripheral Control APIs*/

 /* Enables or Disables the SPIx peripheral in the SPI_CR1
  * register by clearing or setting the SPE bit
  * */
 void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
 {
	 if(EnOrDi == ENABLE)
	 {
		 /* Set the SPE bit (#6) in SPI_CR1*/
		 pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	 }
 	else
 	 {
 		/* Clear the SPE bit (#6) in SPI_CR1*/
		 pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
 	 }
 }

 /* @fn				- void SPI_SSOE_Config
  *
  * @brief
  *
  * @param[in]
  * @param[in]
  *
  * @return
  *
  * @Note
  * */
 void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
 {
	 if(EnOrDi == ENABLE)
	 {
		 pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	 }
	 else
	 {
		 pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	 }

 }


