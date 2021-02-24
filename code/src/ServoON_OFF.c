/**
  ******************************************************************************
  * @file    ServoON_OFF.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2012-02-01
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
#include "ServoON_OFF.h"
#include "Parameters.h"
#include "HardwareConfiguration.h"
#include "CurrentControl.h"
#include "GPIO_functions.h"
#include "ADC_functions.h"
#include "TimersFunctions.h"
#include "Flags.h"
#include "SysMonitor.h"
#include "PWM_Functions.h"
#include "Velocity_Measure.h"
#include "PositionVelocityControl.h"
#include "REFGenFunctions.h"
#include "CommInterface.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#define BrakeDelay 3000	//250 mS Delay
uint16_t BrakeCntr;
extern s16 UqRef;
extern uint16_t SkipFirst;
extern uint32_t status_word;
extern int32_t RefVel;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void Servo_ON(void)
{
    if(CC.ServoOn){
          return;		    // Controller already works
    }

    TIM2_Config();                  // Timer2 Initialisation
    TIM3_Config();                  // Timer3 Initialisation
    TIM5_Config();                  // Timer5 Initialisation
    TIM8_Config();                  // Timer8 Initialisation  
    
    HardwareConfiguration();	    // Calculate Current and Voltage Scaling Coefficients
    CurrentControlInit();	    // ReInit of the Current Controller

    SinAngleInit(def_HALL);
    VelocityMeasInit();
    PositionVelocityControlInit();
    
    TIM_Cmd(TIM8, ENABLE);          // Enable Timer8, PWMs & Timer8 interrupts

    Driver_Enable(); 		    // Driver Enable
    delay_us(5000);                 // Delay ~5ms
    ADC_Zero();                     // Zero Level Definition of Phase A, B and c

    VoltagesCalculation();	    // Measures and calculates Drivers' Voltages

    LED_SON_ON();                   // Turn On "ServoOn" Led

    CC.ServoOn = 1;		    // Start Current Controller
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void Servo_OFF(void)
{
  BrakeON();			    // Motor hold
  if(!(ADCInt & Flag_Brake))	    // Test BIT4
    return;
  Flag_Brake_Clear		    // Clear BIT4
  if(BrakeCntr--)	return;
  BrakeCntr = BrakeDelay;           // Counter Reload
  
  Driver_Disable();		    // Driver Disable
  
  CC.ServoOn = 0;		    // Stop Current Controller

  TIM8_Config();                    // Timer8 Initialisation
  TIM_Cmd(TIM8, ENABLE);            // TIM8 counter enable

  INACTIVE_ALL;
  Delay(50000);                     // 50uS delay
  
  LED_SON_OFF();		    // Turn Off "ServoOn" Led
  
  InPos(Bit_SET/*!Flags.bit.InPos*/);	    // "Out of Position" Signal
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
