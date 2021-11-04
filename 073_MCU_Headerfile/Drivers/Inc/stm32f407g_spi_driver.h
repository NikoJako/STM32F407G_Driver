/*
 * stm32f407g_spi_driver.h
 *
 *  Created on: Jul 13, 2021
 *      Author: njacobs
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f407g.h"


#ifndef INC_STM32F407G_SPI_DRIVER_H_
#define INC_STM32F407G_SPI_DRIVER_H_

/***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 SPI Configuration & Handle Structs  ****************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 ***************************************************************************************
 Configuration Struct*******************************************************************
 ***************************************************************************************
 */
 typedef struct
 {
	 uint8_t SPI_DeviceMode;
	 uint8_t SPI_BusConfig;
	 uint8_t SPI_SCLK_Speed;
	 uint8_t SPI_DFF;
	 uint8_t SPI_CPOL;
	 uint8_t SPI_CPHA;
	 uint8_t SPI_SSM;
 }SPI_Config_t;

 /****************************************************************************************
  Handle Struct*******************************************************************
  ****************************************************************************************/

 typedef struct
 {
	 SPI_RegDef_t	*pSPIx; 			/*This holds the base address of SPIx(x:0,1,2)*/
	 SPI_Config_t	SPIConfig;
	 uint8_t		*pTxBuffer;
	 uint8_t		*pRxBuffer;
	 uint32_t		TxLen;
	 uint32_t		RxLen;
	 uint8_t		TxState;
	 uint8_t		RxState;

 }SPI_Handle_t;


 /****************************************************************************************
   SPI MACROS*******************************************************************
   ****************************************************************************************/

/* SPI Application States */
#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

 /* SPI Application Events */
#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4

 /*
  *
  *
  * @SPI_DeviceMode - master or slave
  *
  * */
 #define SPI_DEVICE_MODE_MASTER	1
 #define SPI_DEVICE_MODE_SLAVE	0

 /*
   * @SPI_BusConfig - half or full duplex
   *
   * */
#define SPI_BUS_CONFIG_FULL		1
#define SPI_BUS_CONFIG_HALF		2
#define SPI_BUS_CONFIG_SIMP_RX	3

 /*
    * @SPI_SCLK_Speed - these are the prescalars to divide the peripheral clock
    * by fpclk by (peripheral clk)
    *
    * */
#define SPI_SCLK_SPEED_DIV_2	0
#define SPI_SCLK_SPEED_DIV_4	1
#define SPI_SCLK_SPEED_DIV_8	2
#define SPI_SCLK_SPEED_DIV_16	3
#define SPI_SCLK_SPEED_DIV_32	4
#define SPI_SCLK_SPEED_DIV_64	5
#define SPI_SCLK_SPEED_DIV_128	6
#define SPI_SCLK_SPEED_DIV_256	7

 /*
    * @SPI_DFF - data framing format size
    * determines if 8 or 16 bits are buffered in the shift register before pushing
    * to Rx buffer
    *
    * */
#define SPI_DFF_8_Bits			0
#define SPI_DFF_16_Bits			1


 /*
    * @SPI_CPOL - clock polarity - the value that the clock is at time = 0
    *
    * */
#define SPI_CPOL_HIGH			1
#define SPI_CPOL_LOW			0


 /*
    * @SPI_CPHA - clock phase - determines what edge of the clock data will be
    * sampled on, rising or falling
    *
    * */
#define SPI_CPHA_HIGH			1
#define SPI_CPHA_LOW			0

 /*
    * @SPI_SSM - slave select management
    *
    * */
#define SPI_SSM_EN				1
#define SPI_SSM_DI				0

 /*
    * SPI related status flag definitions
    * aka "masking info"
    * */
#define SPI_RXNE_FLAG			(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG			(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG			(1 << SPI_SR_UDR)
#define SPI_CRC_ERR_FLAG		(1 << SPI_SR_CRC_ERROR)
#define SPI_MODF_FLAG			(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG			(1 << SPI_SR_OVR)
#define SPI_BUSY_FLAG			(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG 			(1 << SPI_SR_FRE)
#define SPI_SPE_FLAG			(1 << SPI_CR1_SPE)

 /*******************************************************************************************************************************************************************************
  *										APIs Supported By This Driver
  * 					For More Information About the APIs see the Function definitions
  * ******************************************************************************************************************************************************************************
  */

 //Peripheral Clock Setup - enable/disable peripheral clk for a given SPI base address
 void SPI_PeriClkControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

 /*Init & De-Init
 *
 *
 *
 *
 */
 void SPI_Init(SPI_Handle_t *pSPIHandle);



 /*
  *
  */
 void SPI_DeInit(SPI_Handle_t *pSPIHandle);

 /* maskINfo values should be macros on lines 118-126 above*/
 uint8_t SPI_GetFlag_Status(SPI_RegDef_t *pSPIx, uint32_t FlagName);

 /* Data Send and Receive
  * *pSPIx user-provided pointer to the data to be sent
  * *pTxBuffer is used to store the user provided data
  *  len is used to tell how many bytes the API should send
  *
  * */
 void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
 void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);


 uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
 uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);


 void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t ENorDI);
 void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
 void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


 /* Other Peripheral Control APIs*/
void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_Clear_OVR_Flag(SPI_RegDef_t *pSPIx);
void SPI_Close_SPI_Transmission(SPI_Handle_t *pSPIHandle);
void SPI_Close_SPI_Reception(SPI_Handle_t *pSPIHandle);

/* Application Callback*/
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t App_Event);



#endif /* INC_STM32F407G_SPI_DRIVER_H_ */
