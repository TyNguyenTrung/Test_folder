/**
  ******************************************************************************
  * @file    Velocity_Measure.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2015-06-16
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
#include "Velocity_Measure.h"
#include "Motor.h"
#include "Parameters.h"
#include "HardwareConfiguration.h"
#include "IQmathLib.h"         	// include header for IQmath library
#include "CurrentControl.h"
#include "GPIO_functions.h"
#include "ADC_functions.h"
#include "TimersFunctions.h"
#include "Flags.h"
#include "SysMonitor.h"
#include "PWM_Functions.h"

extern int16_t ExtVar[], Pcorr;
extern uint32_t hallccr1;
extern int32_t REF_Pos_Abs;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

struct FEEDBACK FB;
struct REFERENCE REF;

int32_t MedianFilter[3];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void VelocityMeasInit(void)
{
uint16_t i;
int32_t Temp;

    FB.Pos = 0;
    FB.PosOld = 0;
    FB.PosOv = 0;
    //FB.PosOvOld = 0;
    FB.TCntrOld = 0;

    //FB.SpeedM1 = 0;
    FB.VelocityM2 = 0;
    FB.VelocityM2Old = 0;

    //--- Speed Method1 Filter1 ---
    //for(i=0;i<mas1size;i++){FB.mass1[i]=0;}

    //--- Speed Method1 Filter2 ---
    //FB.Sum2 = 0;
    //FB.ptr2 = FB.mass2;
    //for(i=0;i<mas2size;i++){FB.mass2[i]=0;}

    //--- Speed Method2 Filter ---
    FB.Sum3 = 0;
    FB.ptr3 = FB.mass3;
    for(i=0;i<Mas3SizeMax;i++){FB.mass3[i]=0;}

    FB.Mas3Size = 2;
    FB.Mas3Shift = 0;
    for(i = FB.Mas3Size; i > 1; i >>= 1){
            FB.Mas3Shift++; 
    }

    //------------- Median Filter -------------
    MedianFilter[0] = 0;
    MedianFilter[1] = 0;
    MedianFilter[2] = 0;
    
    //--- Speed Monitoring Filter ---
    FB.VelocityMonitor = 0;
    FB.SumMon = 0;
    FB.ptrMon = FB.massMon;
    for(i=0;i<MasMonSizeMax;i++){FB.massMon[i]=0;}

    FB.MasMonSize = 6*MOT.PolePairs;
    FB.MasMonShift = _IQ15div(1,FB.MasMonSize);
    
    REF.Pos = 0;
    REF.PosOld = 0;
    REF.PosOv = 0;
    REF.PosOvOld = 0;
    REF.TCntrOld = 0;
    REF.Velocity = 0;
    //REF.SpeedOld = 0;
    //REF.Acc = 0;

    //--- Reference Speed Filter ---
    REF.Sum = 0;
    REF.ptr = REF.mass;
    for(i=0;i<Mas3SizeMax;i++){REF.mass[i]=0;}

    //--- Acceleration Filter ---
    //REF.Sum2 = 0;
    //REF.ptr2 = REF.mass2;
    //for(i=0;i<mas4size;i++){REF.mass2[i]=0;}
//MOT.EncRes = 1000;//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    Temp = (long)f0_PV*60;
    Temp >>= 2;
    FB.PPStoRPM = _IQ8div(Temp,MOT.EncRes);	// 24Q8 bit format
    
    
    Temp = _IQ16div(4*MOT.EncRes,MOT.PolePairs);  // input 16Q0, output 16Q16 bit format
    
    MOT.EncoderStepSize = _IQ16div(((int32_t)1000*65536),Temp); // input 16Q16, output 16Q16 bit format
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void FeedbackPosition(void)
{
uint16_t /*Pos1,*/Pos2;
int32_t Temp;
sQW Temp1;
  //--------- Feedback position calculation ---------
    Pos2 = EncoderCntr;

  //-------- Counter wrapping -------
    if((Pos2 < 16384) && (FB.TCntrOld > 49152)) 	FB.PosOv+=(EncoderPer+1);
    else if((Pos2 > 49152) && (FB.TCntrOld < 16384))	FB.PosOv-=(EncoderPer+1);
    FB.TCntrOld = Pos2;

 //---------------- Compute the FeedBack Position ------------------
    FB.Pos = FB.PosOv + Pos2; 	                        // Compute Current position
    Temp1.qw = (int64_t)FB.Pos*MOT.EncoderStepSize;     // 32Q0*16Q16=48Q16 format
    
    Temp1.w[0] = Temp1.w[1];				// 16Q0 format
    Temp1.w[1] = Temp1.w[2];				// 16Q0 format
    Temp = Temp1.dw[0];                                 // 32Q0 format
    
    Temp1.qw = (int64_t)Temp*4294967;		// 32Q0*0Q32=32Q32 format; Devision by 1000;
    Temp1.dw[1] *= 1000;			// 32Q0*16Q0=32Q0 format;
    Temp = (Temp - Temp1.dw[1]);	        // 16Q0 format; Remainder;
    
    Temp += Pcorr;
    
    if(Temp >= 1000)  Temp -= 1000;
    else if(Temp < 0) Temp += 1000;
    
    MOT.RotorPosition = Temp;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief Used method V = X2-X1/T = (f0*counts*60)/(4*encoder resolution)[rpm]
  * @param  None
  * @retval None
*****************************************************************************/
void VelocityMeas(void)
{
uint32_t Pos1/*,Pos2*/;
int32_t Temp;

 //+++++++ Read Counters +++++++
    Pos1 = RefTimeCntr;
    //Pos2 = Timer3Cntr;

 //---------------- Compute the Reference Position ------------------
    REF.Pos = REF.PosOv + Pos1; 	// Compute Current position
    Temp = REF.Pos - REF.PosOld;
    REF.PosOld = REF.Pos;
    Temp = (Temp*FB.PPStoRPM) >> 8;	// 32Q0*24Q8=24Q8 -> 32Q0 bit format

 //---------------- Reference Speed Filter -------------------
    REF.Sum -= (*REF.ptr);
    *REF.ptr = Temp;			//Filter input
    REF.Sum += Temp;
    if((&REF.mass[FB.Mas3Size-1])<(++REF.ptr)) REF.ptr = REF.mass;
    Temp = REF.Sum >> FB.Mas3Shift;	    // devision
    REF.Velocity = Temp;

 //============================ FeedBack Speed ============================
    Temp = FB.Pos - FB.PosOld;
    FB.PosOld = FB.Pos;
    Temp = (Temp*FB.PPStoRPM) >> 8;	// 32Q0*24Q8=24Q8 -> 32Q0 bit format

 //---------------- 3 Points Median Filter -----------------
  if(MedianFilter[0] != Temp){
    MedianFilter[2] = MedianFilter[1];
    MedianFilter[1] = MedianFilter[0];
    MedianFilter[0] = Temp;
  }
    if(MedianFilter[2] > MedianFilter[1]){
           if(MedianFilter[1] > MedianFilter[0])        Temp = MedianFilter[1];
           else{   
              if(MedianFilter[2] > MedianFilter[0])     Temp = MedianFilter[0];
              else                                      Temp = MedianFilter[2];
            }
    }
    else{
          if(MedianFilter[2] > MedianFilter[0])         Temp = MedianFilter[2];
          else if(MedianFilter[1] > MedianFilter[0])    Temp = MedianFilter[0];
          else                                          Temp = MedianFilter[1];
    }

 //---------------- Speed Method2 Filter -------------------
    FB.Sum3 -= (*FB.ptr3);
    *FB.ptr3 = Temp;			//Filter input
    FB.Sum3 += Temp;
    if((&FB.mass3[FB.Mas3Size-1])<(++FB.ptr3)) FB.ptr3 = FB.mass3;
    //FB.VelocityM2 = FB.Sum3 >> FB.Mas3Shift;    // devision
FB.VelocityM2 = Temp;//
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
