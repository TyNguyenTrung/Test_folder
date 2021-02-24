/**
  ******************************************************************************
  * @file    HardwareConfiguration.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2012-01-25
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
#include "HardwareConfiguration.h"
#include "TypesDef.h"
#include "ADC_functions.h"
#include "TimersFunctions.h"
#include "CurrentControl.h"
#include "SysMonitor.h"
#include "BackEMF_Control.h"
#include "Flags.h"
#include "PIReg.h"		        // include header for PIREG module
#include "Parameters.h"

extern volatile uint16_t  ADCInt;
extern PIREG IdFBController;
extern PIREG IqFBController;

//extern int16_t Umotsens;
extern int16_t FlagUmotsens;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct HardwareConfiguration HC;

//extern PIREG IqFBController;
//extern PIREG IdFBController;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void HardwareConfiguration(void)
{
sDW Temp, Temp1;
float Temp2, Temp3;

  Temp2 = (float)HC.Isensor/(float)32768;		// from int 1Q15 to float
  Temp3 = (float)HC.Kiamp/(float)2048;			// from int 5Q11 to float
  Temp2 *= Temp3;
  Temp2 *= ADC_Res;
  Temp3 = (int32_t)512*ADC_Umax;
  Temp3 /= Temp2;
  Temp3 *= 2048;
  HC.Iscaling = RoundInt(Temp3);	                  // 1 in 5Q11 bit format
  
  Temp.dw = (int32_t)HC.Iscaling*(ADC_Res >> 1);          // 5Q11*7Q9 = 12Q20 bit format
  Temp.dw <<= 5;					  // 12Q20 -> 7Q25 bit format
  Temp1.dw = (int32_t)29491*Temp.w[1];                    // 1Q15*7Q9 = 8Q24 bit format. Max 90% from theoretical Max Current.
  Temp1.dw <<= 1;                                         // 8Q24 -> 7Q25 bit format
  HC.Imax = Temp1.w[1];					  // 7Q9 bit format
  
  Temp1.dw = (int32_t)31130*Temp.w[1];                    // 1Q15*7Q9 = 8Q24 bit format. Max 95% from theoretical Max Current.
  Temp1.dw <<= 1;                                         // 8Q24 -> 7Q25 bit format
  HC.Ioverlim = Temp1.w[1];				  // 7Q9 bit format
  
  /*Temp.dw = ((int32_t)HC.UsensorUp+(int32_t)HC.UsensorDown)*10; // (16Q0+16Q0)*16Q0 = 32Q0 bit format
  Temp1.dw = (int32_t)HC.UsensorDown*10;		  // 16Q0*16Q0 = 32Q0 bit format
  Temp2 = (float)Temp.dw/(float)Temp1.dw;
  Temp2 *= 1.21;
  Temp2 *= 256;
  HC.Uscaling = RoundLong(Temp2); needs modification*/			  // 24Q8 bit format
  HC.Uscaling = 1150;                                     // 1Q15 bit format
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void VoltagesCalculation(void)
{
sDW Temp;
static uint16_t Umotsensfi = 2000;

  if(!Flag_U) return;
  
  //--------------------------- Calculate Umot -----------------------------
  Umotsensfi += Umotsens; // Voltage Filter
  Umotsensfi >>= 1;
  
  Temp.dw = (int32_t)Umotsensfi*HC.Uscaling;   // 16Q0*17Q15 = 17Q15 bit format

  Temp.dw = (Temp.dw+16384) << 1;         // Rounding up; 17Q15 -> 16Q16 bit firmat
  if(!Temp.w[1])  Temp.w[1] = 1;          // 1V min

  if(Temp.w[1] > 511)    Temp.dw = 33554431;
  else if(Temp.w[1] < 0) Temp.dw = 0;
  
  HC.Umot = Temp.w[1];
  
//-- Current Controller Output Limit Calculation - from -512 to 511.99 V --
  Temp.dw <<= 6;			  // 16Q16 -> 10Q22 bit format
  IqFBController.SatOut = Temp.dw;	  // 10Q22 bit format
  IdFBController.SatOut = Temp.dw;	  // 10Q22 bit format
        
  //-------------------------- Calculate 1/Umot ----------------------------
  Temp.dw = (uint32_t)65536/HC.Umot;      // 16Q16/16Q0 = 16Q16 bit format
  Temp.dw >>= 1;                          // 16Q16 -> 17Q15 bit firmat
  HC.Umotinv = Temp.w[0];                 // 1Q15 bit format

  //--------------------------------- Kpwm ---------------------------------
  Temp.dw = (int32_t)FOC_PWMLimit*HC.Umotinv; // 16Q0*1Q15 = 17Q15 bit format
  HC.Kpwm = Temp.dw >> 5;                     // 17Q15 -> 22Q10 bit format
  
  //------------------------------- Kinvpwm --------------------------------
  Temp.dw = (int32_t)HC.Umot << 14;        // 32Q0 -> 18Q14 bit format
  Temp.dw /= FOC_PWMLimit;                 // 18Q14/16Q0 = 18Q14 bit format
  
  VoltagesMonitor();
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
