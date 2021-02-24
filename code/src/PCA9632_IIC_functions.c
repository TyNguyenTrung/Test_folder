/**
  ******************************************************************************
  * @file    PCA9632_IIC_functions.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2017-02-20
  * @brief
  ******************************************************************************
  *
  *
  *
  *
  *
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "GPIO_functions.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t PCA9632_Init(void)
{
//uint16_t Temp;
int16_t Error = 0;

//----- Send a Start bit by Master -----
	TCN75_StartBit();

//----------------- Send Slave Address ----------------------	
	SendByte(_8pinversion + write);			// Address Byte

//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();

//-- Write a Mode and destination register address to the control register --
	SendByte(AutoIncOn + AutoIncAll + MODE1_Reg);	// Send Byte

//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();

//--------------- Write the Mode 1 register ------------------
	SendByte(SleepNormalMode);	// Send Byte
        
//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();
        
//--------------- Write the Mode 2 register ------------------
	SendByte(OutputLogicInvert + OutputsTotemPole);	// Send Byte
        
//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();
        
//---------------- Write the PWM0 register -------------------
	SendByte(0);	// Send Byte
        
//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();
        
//---------------- Write the PWM1 register -------------------
	SendByte(0);	// Send Byte
        
//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();
        
//---------------- Write the PWM2 register -------------------
	SendByte(0);	// Send Byte
        
//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();
        
//---------------- Write the PWM3 register -------------------
	SendByte(0);	// Send Byte
        
//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();
        
//--- Write the Group duty cycle control, GRPPWM register ---
	SendByte(0);	// Send Byte
        
//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();
        
//------- Write the Group frequency, GRPFREQ register -------
	SendByte(_41ms);	// Send Byte
        
//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();
        
//--- Write the LED driver output state, LEDOUT register ----
	SendByte(LDR3IndPWM + LDR2IndPWM + LDR1IndPWM + LDR0IndPWM);	// Send Byte
        
//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();
        
//------------------- Communication Stop --------------------
	TCN75_StopBit(); 
          
	return Error; // Return "0" on success
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t PCA9632_PWM_DutySet(int16_t PWM, uint16_t Duty)
{
#define DutyScale 
//uint32_t Temp;
int16_t Error = 0;

  if(Duty > 100) Duty = 100;

  Duty = ((uint32_t)20890*Duty) >> 13; //3Q13*16Q0=19Q13->32Q0 bit format
  
//----- Send a Start bit by Master -----
	TCN75_StartBit();

//----------------- Send Slave Address ----------------------	
	SendByte(_8pinversion + write);			// Address Byte

//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();

//-- Write a Mode and destination register address to the control register --
	SendByte(AutoIncOff + PWM);	// Send Byte

//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();

//---------------- Write the PWMx register ------------------
	SendByte(Duty);	// Send Byte
        
//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();

//------------------- Communication Stop --------------------
	TCN75_StopBit(); 
          
	return Error; // Return "0" on success
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t PCA9632_SoftReset(void)
{
int16_t Error = 0;

//----- Send a Start bit by Master -----
	TCN75_StartBit();

//--------------- Send Reset Command SWRST ------------------	
	SendByte(0x06);			// Address Byte

//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();

//------------------ Send Reset Byte 1 ----------------------
	SendByte(0xA5);	// Send Byte

//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();

//------------------ Send Reset Byte 2 ----------------------
	SendByte(0x5A);	// Send Byte

//--------- Receive Acknowledge Bit from the Slave ----------
	Error |= TCN75_AcknowledgeBitReceive();

//------------------- Communication Stop --------------------
	TCN75_StopBit(); 
          
	return Error; // Return "0" on success
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
