/*
 * stm32f407g_i2c_driver.h
 *
 *  Created on: Jan 28, 2022
 *      Author: njacobs
 */

#ifndef INC_STM32F407G_I2C_DRIVER_H_
#define INC_STM32F407G_I2C_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f407g.h"

/* I2Cx Configuration Struct */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

/* I2Cx Handle Struct
 *
 * where *pI2Cx points to any of the following I2Cx peripherals:
 *  I2C1 				((I2C_RegDef_t*)(I2C1_BASE_ADDR))
	I2C2 				((I2C_RegDef_t*)(I2C2_BASE_ADDR))
    I2C3 				((I2C_RegDef_t*)(I2C3_BASE_ADDR))
 * */
typedef struct
{
	I2C_RegDef_t *pI2Cx;				//pointer of type I2CRegDef_t; I2CRegDef_t pointer
	I2C_Config_t		I2C_Config;
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 *
 * */
#define I2C_SCL_SPEED_SM					100000
#define I2C_SCL_SPEED_FM2K				200000
#define I2C_SCL_SPEED_FM4K				200000

/*
 * @ I2C_ACKControl
 *  page 860 in RM
 * */
#define I2C_ACK_ENABLE						1
#define I2C_ACK_DISABLE					0

/*
 * @I2C_FMDutyCycle
 *  of serial clock in fast mode
 *  page 870 in RM
 * */
#define I2C_FM_DUTY_2						0
#define I2C_FM_DUTY_16_9					1

/*******************************************************************************************************************************************************************************
  *										APIs Supported By This Driver
  * 					For More Information About the APIs see the Function definitions
  * ******************************************************************************************************************************************************************************
  */
/*Function to calculate the FREQ value in I2C_CR2, called in I2C_Init*/
uint32_t RCC_GetPCLK1Value(void);

/*Peripheral Clock Setup - enable/disable peripheral clk for a given SPI base address*/
 void I2C_PeriClkControl(I2C_RegDef_t *pI2Cx, uint8_t En_Reset_Or_DI);

 /* */
 void I2C_Init(I2C_Handle_t *pI2CHandle);

 /* */
 void I2C_DeInit(I2C_Handle_t *pI2CHandle);

 /* */
 void I2C_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t ENorDI);
 void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);

 /* Other Peripheral Control APIs*/
void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/* */
 uint8_t I2C_GetFlag_Status(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/* Application Callback - used for interrupt-based API*/
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t App_Event);

#endif /* INC_STM32F407G_I2C_DRIVER_H_ */
