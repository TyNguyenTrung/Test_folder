/**
  ******************************************************************************
  * @file    TuningFunction.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2015-06-18
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
#include "PositionVelocityControl.h"
#include "PIReg.h"		        // include header for PIREG module
#include "CurrentControl.h"
#include "IQmathlib.h"
#include "Velocity_Measure.h"
#include "Motor.h"
#include "TimersFunctions.h"
#include "TypesDef.h"
#include "ADC_functions.h"
#include "HardwareConfiguration.h"
#include "PWM_Functions.h"
#include "GPIO_functions.h"
#include "Flags.h"
#include "Parameters.h"
#include "SysMonitor.h"
#include "LPFReg.h"
#include "IQmathLib.h"         	// include header for IQmath library
#include "PIDReg.h"		// include header for PIREG module
#include "TypesDef.h"

#include "InertiaIdentification.h"
#include "TuningFunctions.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

void TuningVelocity(void)
{
  // Velocity Control
      VelocityFBController.Kp1 = 255876;   // 16Q16  // 3.9
      VelocityFBController.Ki = 200;       // 16Q16  // 0.00305
      VelocityFBController.Kd = 32768;     // = 0.5*2^16  //  //16Q16
}

void TuningPosition(void)
{
      PositionFBController.Kp = 198304;   //16Q16  // 3.025
      PositionFBController.Ki = 0;
      PositionFBController.Kd = 251072;   //16Q16  // 3.83
}

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/


/*****************************************************************************/
/******************************** END OF FILE *********************************/

