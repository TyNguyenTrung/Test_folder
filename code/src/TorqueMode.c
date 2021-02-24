/**
  ******************************************************************************
  * @file    TorqueMode.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2014-02-03
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
#include "TorqueMode.h"
#include "SysMonitor.h"
#include "TimersFunctions.h"
//#include "LeadAngle.h"
#include "Flags.h"
#include "GPIO_functions.h"
#include "CurrentControl.h"
#include "PositionVelocityControl.h"

struct tm TM;

extern int16_t ExtVar[];
extern uint16_t ptr;
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
void DriveStatus(uint16_t *Value1, uint32_t *Value2)
{
	int32_t Temp;

	if(AL.Source) Temp = 1;
	else          Temp = 0;

	*Value1 = (TM.LoadDetect << 7) | (TM.Mode << 4) | (CC.ServoOn << 1) | Temp;

	if(AL.Source){
		*Value2 = AL.Source;
	}
	else{
		//Temp = REF.StepSize >> 16;		// Homing Mode Devisor; 16Q16 -> 33Q-1bit format
		*Value2 = TM.Deviation/Temp;          // Move back step calculation
	}
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void DriveStatus2(uint16_t *Value1, uint32_t *Value2)
{
	int32_t Temp;

	if(AL.Source) Temp = 1;
	else          Temp = 0;

	*Value1 = (TM.LoadDetect << 7) | (TM.Mode << 4) | (CC.ServoOn << 1) | Temp;

        *Value2 = 0;
          
	if(AL.Source){
          *Value2 |= AL.Source;          //
	}
        else{
          Temp = PositionFBController.Ref - PositionFBController.Fdbk;
          if(Temp < -128) Temp = -128; 
          else if(Temp > 127) Temp = 127;
          *Value2 |= Temp;          //
        }
}
/******************************************************************************/
/******************************** END OF FILE *********************************/
