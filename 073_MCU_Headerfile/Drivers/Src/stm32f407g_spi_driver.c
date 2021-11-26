/*
 * stm32f407g_spi_driver.c
 *
 *  Created on: Jul 13, 2021
 *      Author: njacobs
 */
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include "stm32f407g.h"
#include "stm32f407g_spi_driver.h"

/* These are private helper functions for
 * void SPI_IRQHandling(SPI_Handle_t *pHandle)
 * we use static so application code can't call them  */

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);

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

 uint8_t SPI_GetFlag_Status(SPI_RegDef_t *pSPIx, uint32_t FlagName)
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
	 /*While there is data to be received */
	 while(len > 0)
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

 /*Configures the Interrupt Set-enable Registers
  * NVIC_ISER0-NVIC_ISER7 registers
  * (page 4-4 in Cortex M4 Generic User Guide)
   * Input Parameters:
   	 *IRQNumber
   	 *ENABLE of DISABLE
   *Return value:
   	 *None */
 void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t ENorDI)
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
 void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
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

 /* This API doesn't send any data, only saves pTxBuffer
  *  and Len to global variables. Also enables the
  *  TXEIE bit so that an interrupt is triggered whenever
  *  the TXE flag is set in the SPIx_SR
  *
  * Input Parameters:
   	 * SPI_Handle_t *pSPIHandle
   	 * uint8_t *pTxBuffer
   	 * uint32_t len
   	 *
   *Return value:
   * the current state value */
 uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
 {
	 uint8_t curr_state = pSPIHandle->TxState;

	 /* If SPIx isn't busy transmitting
	  * do the following */
	 if(curr_state != SPI_BUSY_IN_TX)
	 {
		 /* 1. Save the Tx Buffer address and len information in
		 	  * some global variables */
		 pSPIHandle->pTxBuffer = pTxBuffer;
		 pSPIHandle->TxLen = len;

		 /* 2. Mark the SPI state as busy in transmission so that no
		  * other code can take over the same peripheral until
		  * transmission is over*/
		 pSPIHandle->TxState = SPI_BUSY_IN_TX;

		 /* 3. Enable the TXEIE control bit to get interrupt whenever
		  * TXE flag is SET in SR */
		 pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);

		 /* 4. Data transmission will be handled by the ISR code
		  * (will implement later)*/
	 }

	 return curr_state;

 }


 uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
 {
	 uint8_t curr_state = pSPIHandle->RxState;

	 /* If SPIx isn't busy Receiving
	 	  * do the following */
	 	 if(curr_state != SPI_BUSY_IN_RX)
	 	 {
		 /* 1. Save the Tx Buffer address and len information in
			  * some global variables */
		 pSPIHandle->pRxBuffer = pRxBuffer;
		 pSPIHandle->RxLen = len;

		 /* 2. Mark the SPI state as busy in transmission so that no
		  * other code can take over the same peripheral until
		  * transmission is over*/
		 pSPIHandle->RxState = SPI_BUSY_IN_RX;

		 /* 3. Enable the RXNEIE control bit to get interrupt whenever
		  * RXNE flag is SET in SR */
		 pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);

		 /* 4. Data transmission will be handled by the ISR code
		  * (will implement later)*/
	 	 }

	 return curr_state;
 }

  /* when the interrupt is triggered, main calls this to figure out what event
   * caused an interrupt to trigger and what caused it to trigger
   *
   * Input Parameters:
   	 * SPI handle
   *Return value:
   	 *None */
 void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
 {
	 /* create variable to check SPIx_SR */
	 uint8_t temp1, temp2;

	 /* Check the TXE bit
	  * if TXE is SET, temp1 will be SET
	  * otherwise temp2 will be CLEARED*/
	 temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE);

	 /* Check if the state of TXEIE in
	  * SPIx_CR2
	  * if TXE is SET, temp1 will be SET
	  * otherwise temp2 will be CLEARED*/
	 temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);

	 if(temp1 && temp2)
	 {
		 /* handle TXE  - helper function */
		 spi_txe_interrupt_handle(pSPIHandle);
	 }

	 /* Check for RXNE*/
	 temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE);
	 temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);

	 if(temp1 && temp2)
	 {
		 /* handle RXNE */
		 spi_rxne_interrupt_handle(pSPIHandle);
	 }

	 /* check the OVR flag */
	 temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR);
	 temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);

	 if(temp1 && temp2)
	 {
		 /* handle OVR Error */
		 spi_ovr_err_interrupt_handle(pSPIHandle);
	 }

 }

 static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
 {
	 /* Reuse code from SPI_SendData */
	 if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	 {
		 /*16-bit - Load DR with 1 byte of data
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
		 pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);

		 pSPIHandle->TxLen--;
		 pSPIHandle->TxLen--;

		 /*type casting and incrementing once is the same incrementing
		  * pTxBuffer twice*/
		 (uint16_t*)pSPIHandle->pTxBuffer++;

	 }
	 else
	 {
		 /*8-bit - Load DR with 1 byte of data
		  * and increment the buffer address*/
		 pSPIHandle->pSPIx->SPI_DR = *((uint8_t*)pSPIHandle->pTxBuffer);
		 pSPIHandle->TxLen--;
		 pSPIHandle->pTxBuffer++;
	 }

	 if(!pSPIHandle->TxLen)
	 {

		 SPI_Close_SPI_Transmission(pSPIHandle);

		 /* Application Callback to Notify Application
		  * that SPI communication is over*/
		 SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

		 /* SPI_EVENT_TX_COMPLT is on possible event
		  * other are defined in stm32f407g_spi_driver.h*/

	 }
 }

 static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
 {
	 /* 16-bit */
	 if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	 {
		 /*Move the received data in the SPI_DR into the pRxBuffer */
		 *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;

		 /*Decrement the length */
		 pSPIHandle->RxLen -= 2;


		 /*Why am I decrementing? Does this have something to do with
		  * big and little endian*/
		 pSPIHandle->pRxBuffer--;
		 pSPIHandle->pRxBuffer--;

		 /*type casting and incrementing once is the same incrementing
		  * pTxBuffer twice*/
		 (uint16_t*)pSPIHandle->pRxBuffer++;
	 }
	 else /* 8-bit */
	 {
		 /*8-bit - Move 1 byte of received data from the SPI_DR
		  * into the pRxBuffer and */
		 pSPIHandle->pRxBuffer = ((uint8_t*)pSPIHandle->pSPIx->SPI_DR);

		 /*Decrement length count*/
		 pSPIHandle->RxLen--;

		 /*decrement pointer address to next byte to be read in*/
		 pSPIHandle->pRxBuffer--;
	 }

	 if(!pSPIHandle->RxLen)
	 {
		 SPI_Close_SPI_Reception(pSPIHandle);

		 /* Application Callback to Notify Application
		  * that SPI communication is over*/
		 SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

		 /* SPI_EVENT_TX_COMPLT is on possible event
		  * other are defined in stm32f407g_spi_driver.h*/

	 }
 }

 static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
 {
	 /* 1. Clear the OVR flag
	  * 2. Inform the application*/

	 uint8_t temp;

	 /* If the SPIx isn't currently busy Tx'n,
	  * clear the OVR flag by:
	 	  * 1. read the SPI_DR register
	 	  * 2. read/access the SPI_SR register
	 	  *
	  * */
	 if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	 {
		 SPI_Clear_OVR_Flag(pSPIHandle->pSPIx);
	 }

	 /*Application Callback to Notify application*/
	 SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

	 /*What if the SPIx is busy in Tx and the OVR
	  * occurs? SPI_ApplicationEventCallback will be
	  * sent to the application and now the application
	  * needs a way to clear the OVR flag and close SPI
	  * communication */
 }

 void SPI_Clear_OVR_Flag(SPI_RegDef_t *pSPIx)
 {
	 /* 1. Clear the OVR flag
	  * 2. Inform the application*/

	 uint8_t temp;

	 /* If the SPIx isn't currently busy Tx'n,
	  * clear the OVR flag by:
		  * 1. read the SPI_DR register
		  * 2. read/access the SPI_SR register*/
	 temp = pSPIx->SPI_DR;
	 temp = pSPIx->SPI_SR;


 }


 void SPI_Close_SPI_Transmission(SPI_Handle_t *pSPIHandle)
 {
	 /* Close the SPI communication and
	  * inform the application that SPI
	  * is over:
	  *
	  * 1. Disable TXEIE interrupts generated from
	  * setting the TXE flag */
	 pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);

	 /* Reset TxBuffers */
	 pSPIHandle->pTxBuffer = NULL;
	 pSPIHandle->TxLen = 0;
	 pSPIHandle->TxState = SPI_READY;
 }

 void SPI_Close_SPI_Reception(SPI_Handle_t *pSPIHandle)
 {
	 /* Close the SPI communication and
	  * inform the application that SPI
	  * is over:
	  *
	  * 1. Disable RXNEIE interrupts generated from
	  * setting the TXE flag */
	 pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);

	 /* Reset RxBuffers */
	 pSPIHandle->pRxBuffer = NULL;
	 pSPIHandle->RxLen = 0;
	 pSPIHandle->RxState = SPI_READY;

	 /* Application Callback to Notify Application
	  * that SPI communication is over*/
	 SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

 }

 /*  we implemented this function as a place holder expecting the application writers to implement they're own function to over ride this one
	iii. if they don't, this will raise a compiler error
	iv. to prevent the compiler error, we use the GCC '_attribute_'((weak)) keyword before the function name to make this function a 'weak' function
	v. with the __attribute__((weak)) GCC attribute, this weak function will be called if the application writers forget to override it with a function of
	their own
  *  */
 __attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t App_Event)
 {
	 /* Weak implementation, with hopes that the application may
	  * over-write this function*/
 }

