/**
  ******************************************************************************
  * @file    CurrentControl.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2015-06-08
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
#include "CurrentControl.h"
#include "PIReg.h"		        // include header for PIREG module
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
//#include "InertiaIdentification.h"
//#include "Observer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

struct CONTROLLER CC;
PIREG IqFBController = PIREG_DEFAULTS;	 // initialize the IqController module with default settings
PIREG IdFBController = PIREG_DEFAULTS;	 // initialize the IdController module with default settings
LPFREG RefCurrentLPF;

struct MOTOR MOT;

extern PIREG SpeedFBController;
extern int16_t ExtVar[];
extern int16_t UqRef;
extern int32_t SpeedFBControllerOut;

extern int32_t IcurrentTmp;
extern int16_t IcurrentAv;
extern int32_t VelocityFBControllerOut;
extern int16_t Iaverage;
int16_t TorqueLimitBuff;

extern int32_t Speed_Buff; 	// command maximum speed (from RS232)
extern int16_t CPPulse;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void CurrentControlInit(void)
{
  CurrentControlSingleInit();
  CurrentControlMultipleInit();
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void CurrentControlSingleInit(void)
{
//++++++++++ Current Controller Status Init +++++++++
  CC.ServoOn = 0;
  CC.Ia = 0;
  CC.Ib = 0;
  CC.Ic = 0;
  CC.Iq = 0;
  CC.Id = 0;
  
  //----- Limit to Imax -----
  CC.RefLimit = HC.Imax;
  TorqueLimitBuff = 300; // 300%
  
  //++++++++++ Current Controller Coeffitients Init +++++++++
  IqFBController.PPart = 0;
  IqFBController.IPart = 0;
  IqFBController.Ref =	0;
  IqFBController.Fdbk=	0;
  IqFBController.Err =	0;
  IqFBController.Out = 	0;
  IqFBController.SatOut = 0;
  
  IdFBController.PPart = 0;
  IdFBController.IPart = 0;
  IdFBController.Ref =	0;
  IdFBController.Fdbk=	0;
  IdFBController.Err =	0;
  IdFBController.Out = 	0;
  IdFBController.SatOut = 0;

//++++++++++++ Reference Current Filter Init +++++++++++
  //RefCurrentLPF.Bandwidth = 5000;		// [Hz] 16Q0 bit format
  RefCurrentLPF.Filter = LPF;
  RefCurrentLPF.U1.dw = 0;
  RefCurrentLPF.Y1.dw = 0;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void CurrentControlMultipleInit(void)
{
sDW Temp;
int32_t Temp1;

  //------------- Kp=2*pi*fb*L ------------
    Temp.dw = (int32_t)_2pi*CC.Bandwidth;	// 16Q16*16Q0=16Q16 bit format
    Temp.dw += 32768;				// Rounding up
    Temp.dw = (int32_t)Temp.w[1]*MOT.La;	// 16Q0*1Q15=17Q15 bit format
    IqFBController.Kp = Temp.dw << 1;		// 17Q15 -> 16Q16 bit format;
    IdFBController.Kp = IqFBController.Kp;

  //----------- Ki=R/(L*f0) -----------
    Temp.dw = (int32_t)f0_C*MOT.La;		// 32Q0*1Q15=17Q15 bit format
    Temp1 = (int32_t)MOT.Ra << 7;		// 8Q8 -> 17Q15 bit format
    Temp.dw = _IQ15div(Temp1,Temp.dw);		// 17Q15/17Q15 = 17Q15 bit format;
    IqFBController.Ki =	Temp.dw << 1;		// 17Q15 -> 16Q16 bit format;
    IdFBController.Ki =	IqFBController.Ki;
    
  //++++++++++++ Reference Current Filter Init +++++++++++
    LPF_Init(RefCurrentLPF.Bandwidth,T0_C,&RefCurrentLPF);
    
  //--------------- Current Limit Calculation ------------------
    Temp1 = (long)MOT.Inom*TorqueLimitBuff;   // 7Q9*16Q0=25Q9 bit format
    Temp1 *= 82;                              // 25Q9*3Q13=10Q22 bit format; division by 100
    Temp1 >>= 13;                             // 10Q22 -> 25Q9 bit format
    if(Temp1 > HC.Imax) Temp1 = HC.Imax;      // Limit the max motor's curren to the max drive one
    CC.RefLimit = Temp1;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void Delay_T0uS(uint16_t Cycles)
{
  while(Cycles--){ 	          // X ADC samples
    while(!(ADCInt & Flag_CC)){}  // Test BIT0; Wait for a new ADC convertion
      Flag_CC_Clear		  // Clear BIT0
  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void PMSM_CurrentController(void)
{
sDW Temp;
register int16_t Ud, Uq;

  //----------------- Reference Current -----------------
      
    //--- Low Pass Filter - Input is 16Q16 bit format; Output 16Q16 bit format
    Temp.dw = RefCurrentLPF.Filter(VelocityFBControllerOut,&RefCurrentLPF);	// 16Q16 bit format
     
    //- Reference Current Limitation - 16Q0 but we assume it is in 7Q9 bit format -
    if(Temp.w[1] < -CC.RefLimit)	Temp.w[1] = -CC.RefLimit;
    else if(Temp.w[1] > CC.RefLimit)	Temp.w[1] =  CC.RefLimit;


  //------------- Current controller Phase q ------------
    IqFBController.Ref = Temp.w[1];						// 7Q9 bit format
    IqFBController.Fdbk = CC.Iq;

    IqFBController.Err = (IqFBController.Ref - IqFBController.Fdbk) >> 3;	// 7Q9 - 7Q9 = 7Q9 -> 26Q6 bit format
    IqFBController.Controller(&IqFBController);				        // call q axis FeedBack current controller

  //--------- Transformation Voltage from volts to PWM units -----------
  // Output format is 10Q6*16Q16 = 10Q22 !!!!!!!!!
    Temp.dw = IqFBController.Out;			// 10Q22 bit format Voltage in Volts;
    Temp.dw = Temp.w[1]*HC.Kpwm;			// 10Q6*22Q10 = 16Q16 bit format; Voltage in PWM units
    Uq = Temp.w[1];					// 16Q0 bit format; Voltage in PWM units
    

  //------------- Current controller Phase d ------------
    IdFBController.Ref = 0;													// 7Q9 bit format
    IdFBController.Fdbk= CC.Id;												// 7Q9 bit format
    IdFBController.Err = (IdFBController.Ref - IdFBController.Fdbk) >> 3;	// 7Q9 - 7Q9 = 7Q9 -> 26Q6 bit format
    IdFBController.Controller(&IdFBController);					// call d axis FeedBack current controller

  //--------- Transformation Voltage from volts to PWM units -----------
  // Output format is 10Q6*16Q16 = 10Q22 !!!!!!!!!
    Temp.dw = IdFBController.Out;			// 10Q22 bit format Voltage in Volts;
    Temp.dw = Temp.w[1]*HC.Kpwm;			// 10Q6*22Q10 = 16Q16 bit format; Voltage in PWM units
    Ud = Temp.w[1];					// 16Q0 bit format; Voltage in PWM units

    PMSM_UdUq_2_UAlfaBeta_2_Uabc(Ud, Uq);

    OverLoadMonitor(Temp.w[1]);
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
