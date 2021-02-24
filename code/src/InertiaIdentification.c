/**
  ******************************************************************************
  * @file    PositionVelocityControl.c 
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

#include "arm_itm.h" // Ton adding

int8_t num1 = -128;                    // 8
int16_t num2 = 32767;                  // 16
int32_t num3 = 2147483647;             // 32
int64_t num4 = 9223372036854775807;    // 64

uint8_t num5 = 255;                      // 8
uint16_t num6 = 65535;                   // 16
uint32_t num7 = 4294967295;              // 32
         

extern int32_t VelocityFBControllerOut;

extern float SUMInertia = 0;

uint16_t Count = 0;
uint16_t Number = 0;
uint16_t i= 0;

uint16_t Current_Iq_ref = 0;
uint32_t Velocity_FB = 0;

// iq
float Array_iq[20]; 
float iqmax[5], iqmin[5];

float averageiqmax = 0;
float averageiqmin = 0;

float iq1 = 0;
float iq2 = 0;

float SUMiq = 0;
float SUMiq1 = 0;
// acc
int32_t Velocity = 0;
int32_t Revelocity = 0;
float Array_acc[20];

float accmax[5], accmin[5];

float averageaccmax = 0;
float averageaccmin = 0;

float SUMacc = 0;

float Acc_float1 = 0;
float Acc_float2 = 0;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//int32_t Hoang = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void Inertia_total(void)
{
  sDW Temp;
  sDW Acc;  
   
  //Hoang++;
  //ITM_EVENT16_WITH_PC(1, Hoang);
  
    //----------------- Reference Current -----------------
      
    //--- Low Pass Filter - Input is 16Q16 bit format; Output 16Q16 bit format
    Temp.dw = RefCurrentLPF.Filter(VelocityFBControllerOut,&RefCurrentLPF);	// 16Q16 bit format
     
    //- Reference Current Limitation - 16Q0 but we assume it is in 7Q9 bit format -
    if(Temp.w[1] < -CC.RefLimit)	Temp.w[1] = -CC.RefLimit;
    else if(Temp.w[1] > CC.RefLimit)	Temp.w[1] =  CC.RefLimit;

   
   //------------- Current controller Phase q ------------
    IqFBController.Ref = Temp.w[1];		         // 7Q9 bit format  int16_t (-4194304 ~ 4194303.998023437)
    IqFBController.Fdbk = CC.Iq;                         // 7Q9 bit format  int16_t (-4194304 ~ 4194303.998023437)
    //----------------- Feedback velocity -----------------
    VelocityFBController.Fdbk = FB.VelocityM2;		// 32Q0 bit format 
       
    /////////////////////////////////////////////////////////
    //////// Hoang read Velocity-FB and Current-Iq-REF //////
    /////////////////////////////////////////////////////////
    
    Current_Iq_ref = IqFBController.Ref;
    Velocity_FB = VelocityFBController.Fdbk;
    int16_t t = 10;
    ITM_EVENT16_WITH_PC(1, t);  // Ton adding
    ITM_EVENT32_WITH_PC(2, Velocity_FB);     // Ton adding

    
    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////
    if ( Number <= 20)
    {
    iq1 = (float)(IqFBController.Ref);
    iq2 = iq1/512; 
    
    //ITM_EVENT16_WITH_PC(1, iq2);  // Ton adding
    
    //----------- Current ----------------
    Array_iq[Count] = iq2;                 
    
    //----------- Acceleration -----------
    Velocity = VelocityFBController.Fdbk;                 // 32Q0 ~ int32_t

    Acc.dw = (int32_t)(Velocity - Revelocity);                // 32Q0*16Q0 = 32Q0 bit format
      
    Acc_float1 = (float)(Acc.dw ); // rpm
    Acc_float2 = (Acc_float1/60)*2*3.14*200; //rad/s^2
    
    Array_acc[Count] = Acc_float2;   // 2kHz                           // 32Q0
    Revelocity = Velocity;
    
      if(Count > 1)
      {
        if (Number <=4){i=0;} 
        else if(Number >5 && Number<=8)  {i=1;}
        else if(Number >9 && Number<=12)  {i=2;}
        else if(Number >13 && Number<=16)  {i=3;}
        else if(Number >17 && Number<=20) {i=4;}
        
        //----------------- Find iqmax in 20 period---------
        if(Array_iq[Count] >= Array_iq[Count-1])
        {iqmax[i] = Array_iq[Count];}                       // 7Q9 ~ int16_t
        else 
        {iqmax[i] = Array_iq[Count-1];}                     // 7Q9 ~ int16_t
        //----------------- Find iqmin in 20 period---------     
        if(Array_iq[Count] < Array_iq[Count-1])
        {iqmin[i] = Array_iq[Count];}                       // 7Q9 ~ int16_t
        else 
        {iqmin[i] =  Array_iq[Count-1];}                    // 7Q9 ~ int16_t
        
        //----------------- Find accmax in 20 period---------
        if(Array_acc[Count] >= Array_acc[Count-1])          
        {accmax[i] = Array_acc[Count];}                     // 32Q0
        else 
        {accmax[i] = Array_acc[Count-1];}                   // 32Q0
        //----------------- Find accmin in 20 period---------     
        if(Array_acc[Count] < Array_acc[Count-1])
        {accmin[i] = Array_acc[Count];}                     // 32Q0
        else
        {accmin[i] =  Array_acc[Count-1];}                  // 32Q0
      }
      if(Count > 4)
      { Count = 1; }   
      else 
      { }
      
    SUMInertia = SUMInertia;
    }
    //----------------- Reset 5 period  -----------------
    else if (Number > 20)
    { 
      averageiqmax = (iqmax[0]+iqmax[1]+iqmax[2]+iqmax[3]+iqmax[4])/5; // 7Q9 ~ int16_t
      averageiqmin = (iqmin[0]+iqmin[1]+iqmin[2]+iqmin[3]+iqmin[4])/5; // 7Q9 ~ int16_t
      
      if (averageiqmax >0) {averageiqmax = averageiqmax;}
      else {averageiqmax = -averageiqmax;}
      if (averageiqmin >0) {averageiqmin = averageiqmin;}
      else {averageiqmin = -averageiqmin;}
      
      averageaccmax = (accmax[0]+accmax[1]+accmax[2]+accmax[3]+accmax[4])/5; // 32Q0 ~ int32_t
      averageaccmin = (accmin[0]+accmin[1]+accmin[2]+accmin[3]+accmin[4])/5; // 32Q0 ~ int32_t
      
      if (averageaccmax  >0) {averageaccmax = averageaccmax;}
      else {averageaccmax  = -averageaccmax ;}
      if (averageaccmin >0) {averageaccmin = averageaccmin;}
      else {averageaccmin = -averageaccmin;}
        
       
      SUMiq  =  averageiqmax+averageiqmin; 

      SUMacc = averageaccmax+averageaccmin; 
      
      
      SUMInertia = (0.0613*(SUMiq/SUMacc))*10000000; // (g*cm^2)
      
      Number = 0;
      Count = 0;
    }
    
    Count++;  // if count >4 =. count = 0 => throgh this line count = 0+1 = 1
    Number++; // if Number >20 =. Number = 0 => throgh this line Number = 0+1 = 1
    
    
   
    
    /*  
    if ( Number <= 100)
    {
    //----------- Current ----------------
    Array_iq[Count] = IqFBController.Ref;                 // 7Q9 ~ int16_t
    //----------- Acceleration -----------
    Velocity = VelocityFBController.Fdbk;                 // 32Q0 ~ int32_t
    Acc.dw = (int32_t)(Velocity - Revelocity)*2000;                // 32Q0*16Q0 = 32Q0 bit format
      
    Array_acc[Count] = Acc.dw;   // 2kHz                           // 32Q0
    Revelocity = Velocity;
    
      if(Count > 1)
      {
        if (Number <=20){i=0;} 
        else if(Number >21 && Number<=40)  {i=1;}
        else if(Number >41 && Number<=60)  {i=2;}
        else if(Number >61 && Number<=80)  {i=3;}
        else if(Number >81 && Number<=100) {i=4;}
        
        //----------------- Find iqmax in 20 period---------
        if(Array_iq[Count] >= Array_iq[Count-1])
        {iqmax[i] = Array_iq[Count];}                       // 7Q9 ~ int16_t
        else 
        {iqmax[i] = Array_iq[Count-1];}                     // 7Q9 ~ int16_t
        //----------------- Find iqmin in 20 period---------     
        if(Array_iq[Count] < Array_iq[Count-1])
        {iqmin[i] = Array_iq[Count];}                       // 7Q9 ~ int16_t
        else 
        {iqmin[i] =  Array_iq[Count-1];}                    // 7Q9 ~ int16_t
        
        //----------------- Find accmax in 20 period---------
        if(Array_acc[Count] >= Array_acc[Count-1])          
        {accmax[i] = Array_acc[Count];}                     // 32Q0
        else 
        {accmax[i] = Array_acc[Count-1];}                   // 32Q0
        //----------------- Find accmin in 20 period---------     
        if(Array_acc[Count] < Array_acc[Count-1])
        {accmin[i] = Array_acc[Count];}                     // 32Q0
        else
        {accmin[i] =  Array_acc[Count-1];}                  // 32Q0
      }
      if(Count > 20)
      { Count = 1; }   
      else 
      { }
      }
    //----------------- Reset 5 period  -----------------
    else if (Number > 100)
    { 
      averageiqmax = (iqmax[0]+iqmax[1]+iqmax[2]+iqmax[3]+iqmax[4])/5; // 7Q9 ~ int16_t
      averageiqmin = (iqmin[0]+iqmin[1]+iqmin[2]+iqmin[3]+iqmin[4])/5; // 7Q9 ~ int16_t
      
      if (averageiqmax >0) {averageiqmax = averageiqmax;}
      else {averageiqmax = -averageiqmax;}
      if (averageiqmin >0) {averageiqmin = averageiqmin;}
      else {averageiqmin = -averageiqmin;}
      
      averageaccmax = (accmax[0]+accmax[1]+accmax[2]+accmax[3]+accmax[4])/5; // 32Q0 ~ int32_t
      averageaccmin = (accmin[0]+accmin[1]+accmin[2]+accmin[3]+accmin[4])/5; // 32Q0 ~ int32_t
      
      if (averageaccmax  >0) {averageaccmax = averageaccmax;}
      else {averageaccmax  = -averageaccmax ;}
      if (averageaccmin >0) {averageaccmin = averageaccmin;}
      else {averageaccmin = -averageaccmin;}
        
      SUMInertia = (float)((averageiqmax+averageiqmin)/((averageaccmax+averageaccmin)*512));   // 2^9 = 512
                           
      Number = 0;
      Count = 0;
    }
    
    Count++;  // if count >20 =. count = 0 => throgh this line count = 0+1 = 1
    Number++; // if Number >100 =. Number = 0 => throgh this line Number = 0+1 = 1
    
    */
}

/*****************************************************************************/
/******************************** END OF FILE *********************************/

