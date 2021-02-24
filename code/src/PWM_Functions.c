/**
  ******************************************************************************
  * @file    PWM_Functions.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2015-02-02
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
#include "PWM_Functions.h"
#include "TimersFunctions.h"
#include "TypesDef.h"
#include "SIN_Tab.h"
#include "CurrentControl.h"
#include "svgen_dq.h"
#include "SysMonitor.h"

SVGENDQ svgen_dq = SVGENDQ_DEFAULTS;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define CR1_UDIS_Set                ((u16)0x0002)
#define CR1_UDIS_Reset              ((u16)0x03FD)
/* Private variables ---------------------------------------------------------*/

//struct Alarm AL;
//union UINAL Uinal;
//union UMOTAL Umotal;
//struct SysMonitor SM;

extern s16 ExtVar[];
extern int16_t _PhA, Ph, LPF2Pos;
u16 Hall;
u16 HallEmul = 0;
int16_t UqRef;
int16_t PhA, PhB, PhC;
uint32_t PWMTimerCounterOld;
extern int16_t PhAOld;
uint16_t SempPoitSelect = 1;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void PMSM_PWMs_Update(int32_t Ua, int32_t Ub, int32_t Uc)
{
int32_t Temp[3];
int16_t DeltaDuty, TimePhD;
   
  // Phase voltages limitation
  if(Ua < -FOC_PWMLimit) Ua = -FOC_PWMLimit; else
  if(Ua >  FOC_PWMLimit) Ua =  FOC_PWMLimit;

  if(Ub < -FOC_PWMLimit) Ub = -FOC_PWMLimit; else
  if(Ub >  FOC_PWMLimit) Ub =  FOC_PWMLimit;
  
  if(Uc < -FOC_PWMLimit) Uc = -FOC_PWMLimit; else
  if(Uc >  FOC_PWMLimit) Uc =  FOC_PWMLimit;
    
//ExtVar[0] = Ua;//<<<<<<<<<<<<<<<<<<<<<<<<<<
//ExtVar[1] = Ub;//<<<<<<<<<<<<<<<<<<<<<<<<<<
//ExtVar[2] = Uc;//<<<<<<<<<<<<<<<<<<<<<<<<<<
  
  // Find the two max voltages and put them in Temp[0] and Temp[1]
  Temp[0] = Ua + PWMTimerPeriodMiddle; 
  Temp[1] = Ub + PWMTimerPeriodMiddle;
  Temp[2] = Uc + PWMTimerPeriodMiddle;

  if(Temp[0] > Temp[1])
  {
    if(Temp[1] > Temp[2])
    {}
    else
    {   
      if(Temp[0] > Temp[2]){ Temp[1] = Temp[2];}
      else{
        Temp[1] = Temp[0];
        Temp[0] = Temp[2];
      }
    }
  }
  else
  {
    if(Temp[0] > Temp[2])
    {
      Temp[2] = Temp[0];
      Temp[0] = Temp[1];
      Temp[1] = Temp[2];
    }
    else if(Temp[1] > Temp[2]){ 
      Temp[0] = Temp[1];
      Temp[1] = Temp[2];
    }
    else
    {                       
      Temp[0] = Temp[2];
    }
  }  
  
  // Sampling point before counter overflow
  //PWMTimer->CCER &= 0xDFFF;     // Set Polarity of CC4 High
  // Set CC4 as PWM mode 1(default)
  PWMTimer->CCMR2 &= CCMR2_CH4_DISABLE;
  PWMTimer->CCMR2 |= CCMR2_CH4_PWM_MODE1;
  
  // ADC Syncronization setting value             
  if((u16)(PWMTimerPeriod-Temp[0]) > TW_AFTER)
  {
    TimePhD = PWMTimerPeriod - 1;
  }
  else
  {
    DeltaDuty = (u16)(Temp[0] - Temp[1]);
    
    // Definition of crossing point
    if(DeltaDuty > (u16)(PWMTimerPeriod-Temp[0])*2) 
    {
      TimePhD = Temp[0] - TW_BEFORE; // Ts before Phase A 
    }
    else
    {
      TimePhD = Temp[0] + TW_AFTER; // DT + Tn after Phase A
       
      if(TimePhD >= PWMTimerPeriod)
      {        
        // Sampling point after counter overflow
        //PWMTimer->CCER |= 0x2000; // Set Polarity of CC4 Low
        // Set CC4 as PWM mode 2
        PWMTimer->CCMR2 &= CCMR2_CH4_DISABLE;
        PWMTimer->CCMR2 |= CCMR2_CH4_PWM_MODE2;
  
        TimePhD = (2*PWMTimerPeriod) - TimePhD-1;
      }
    }
  }

  // Set the phase voltages
  TIM_SetCompare1(PWMTimer,(Ua + PWMTimerPeriodMiddle));
  TIM_SetCompare2(PWMTimer,(Ub + PWMTimerPeriodMiddle));
  TIM_SetCompare3(PWMTimer,(Uc + PWMTimerPeriodMiddle));
 
  // Set the sampling point
  if(SempPoitSelect) TIM_SetCompare4(PWMTimer,TimePhD);
  
  Temp[1] = TW_BEFORE;
  Temp[2] = TW_AFTER;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void PMSM_UdUq_2_UAlfaBeta_2_Uabc(int16_t Ud, int16_t Uq)
{
#define Sqrt3	28378 // 2Q14 bit format
register int16_t Ualfa, Ubeta;//, Ua, Ub, Uc;
sDW Temp;

  // The (d,q)->(a,b) projection (inverse Park transformation)
  // Ualfa = Ud*cos(Theta) - Uq*sin(Theta)
  // Ubeta = Ud*sin(Theta) + Uq*cos(Theta)

  Temp.dw = (int32_t)Ud*CC.cos - (int32_t)Uq*CC.sin;	// 16Q0*1Q15=17Q15 bit format
  Temp.dw <<= 1;					// 17Q15 -> 16Q16 bit format
  Ualfa = Temp.w[1];					// 16Q0 bit format

  Temp.dw = (int32_t)Ud*CC.sin + (int32_t)Uq*CC.cos;	// 16Q0*1Q15=17Q15 bit format
  Temp.dw <<= 1;					// 17Q15 -> 16Q16 bit format
  Ubeta = Temp.w[1];					// 16Q0 bit format

  // The (a,b)->(a,b,c) projection (inverse Clarke transformation)
  // Ua = Ualpha
  // Ub = (-1/2)*Ualpha + (sqrt(3)/2)*Ubetha
  // Uc = (-1/2)*Ualpha - (sqrt(3)/2)*Ubetha

  /*Ua = Ualfa;				// 16Q0 bit format
  Temp.dw = (int32_t)Ubeta*Sqrt3;	// 16Q0*2Q14 = 18Q14 bit format
  Temp.dw <<= 2;			// 18Q14 -> 16Q16 bit format
  Ub = ( Temp.w[1] - Ualfa) >> 1;	// 16Q0 bit format
  Uc = (-Temp.w[1] - Ualfa) >> 1; 	// 16Q0 bit format

  PMSM_PWMs_Update(Ua,Ub,Uc);*/

  //------------ 3 Phase SVM -----------
  svgen_dq.Ualpha = Ualfa;  // Pass inputs to svgen_dq
  svgen_dq.Ubeta = Ubeta;   // Pass inputs to svgen_dq
  svgen_dq.calc(&svgen_dq); // Call compute function for svgen_dq

  PMSM_PWMs_Update(-svgen_dq.Ta,-svgen_dq.Tb,-svgen_dq.Tc);
}
/*****************************************************************************/



int16_t Pcorr=0;
/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void SinAngleInit(uint16_t Hall)
{
#define Degree60  1000.0/6.0

  if(Hall == Hall4) PhA = (int16_t)(4*Degree60); else
  if(Hall == Hall3) PhA = (int16_t)(3*Degree60); else
  if(Hall == Hall2) PhA = (int16_t)(2*Degree60); else
  if(Hall == Hall1) PhA = (int16_t)(1*Degree60); else
  if(Hall == Hall0) PhA = (int16_t)(0*Degree60); else
  if(Hall == Hall5) PhA = (int16_t)(5*Degree60); else
  {if(!AL.Source)  AL.Source = 7;}	// Alarm Source Set
  
  PhAOld = PhA;
  Pcorr = PhA;
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
