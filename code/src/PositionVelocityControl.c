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

extern int16_t ExtVar[];

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

PIDREG VelocityFBController = PIDREG_DEFAULTS;    // initialize the Speed Controller module with default settings
PIDREG PositionFBController = PIDREG_DEFAULTS;	// initialize the Position Controller module with default settings

LPFREG PositionFFLPF;
LPFREG PositionLPF;


extern int32_t TotalInertia;           // 8Q24 bit format;

int32_t RefSpeed = 0;
int32_t VelocityFBControllerOut;
int16_t Kpff;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void PositionVelocityControlInit(void)
{
    PositionVelocityControlSingleInit();
    PositionVelocityControlMultipleInit();
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void PositionVelocityControlSingleInit(void)
{
uQW Temp;

//++++++++++ Position Controller Coeffitients Init +++++++++
    PositionFBController.SatOut =	_IQ15(13100);
    PositionFBController.Ref =		0;
    PositionFBController.Fdbk =		0;
    PositionFBController.PPart =	0;
    PositionFBController.IPart =	0;
    PositionFBController.DPart =	0;
    PositionFBController.Err =		0;
    PositionFBController.Out =		0;
    
//+++++++++++ Speed Controller Coeffitients Init +++++++++++
    VelocityFBController.SatOut =	_IQ(CC.RefLimit);
    VelocityFBController.Ref =		0;
    VelocityFBController.Fdbk =		0;
    VelocityFBController.PPart =	0;
    VelocityFBController.IPart =	0;
    VelocityFBController.Err =		0;
    VelocityFBController.Out =		0;
    
    Temp.qw = MOT.InertiaRatio*MOT.RotorInertia; // 16Q16/8Q24 = 24Q40 bit format;
    Temp.qw <<= 16;                              // 24Q40 -> 8Q56 bit format;
    TotalInertia = Temp.dw[1] + MOT.RotorInertia;// Save in 8Q24 bit format

//++++++++++ Position Controller Coeffitients Init +++++++++    
    PositionFBController.Kp = 65536; // 16Q16 bit format
    PositionFBController.Kd = 130000;

//+++++++++++ Speed Controller Coeffitients Init +++++++++++
    VelocityFBController.Kp1 = 400000;
    VelocityFBController.Ki = 500;
    
//---------- Position Feedforward Low pass Filter -----------
    PositionFFLPF.U1.dw = 0;
    PositionFFLPF.Y1.dw = 0;
    PositionFFLPF.Filter = LPF;
    
  //---------- Position Low pass Filter -----------
    PositionLPF.U1.dw = 0;
    PositionLPF.Y1.dw = 0;
    PositionLPF.Filter = LPF;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void PositionVelocityControlMultipleInit(void)
{
int32_t Temp;
int16_t Vbw, Pbw;

//---------- Position Feedforward Low pass Filter -----------
    PositionFFLPF.Bandwidth = 900;			// [Hz] 16Q0 bit format
    if(PositionFFLPF.Bandwidth >= (f0_PV/2))    PositionFFLPF.Bandwidth = (f0_PV/2) - 1;
    LPF_Init(PositionFFLPF.Bandwidth,T0_PV,&PositionFFLPF);
    
    
  //---------- Position Low pass Filter -----------
    PositionLPF.Bandwidth = 500;			// [Hz] 16Q0 bit format
    if(PositionLPF.Bandwidth >= (f0_PV/2))    PositionLPF.Bandwidth = (f0_PV/2) - 1;
    LPF_Init(PositionLPF.Bandwidth,T0_PV,&PositionLPF);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void PositionControl(void)
{

  PositionFBController.Ref  = REF.Pos;
  PositionFBController.Fdbk = FB.Pos;

  PositionFBController.Controller(&PositionFBController);
	
  PositionFBController.Out = PositionLPF.Filter(PositionFBController.Out,&PositionLPF);    // Input in 16Q16 bit format; Output in 16Q16 bit format
        
  PositionFBController.Out >>= 16;		//16Q16 -> 32Q0 bit format
  
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void VelocityControl(void)
{
int32_t Temp;
sQW Temp1;

    Temp = REF.Velocity*Kpff;					        // 32Q0*2Q14 = 18Q14 bit format
    Temp <<= 2;								// 18Q14 -> 16Q16 bit format
    Temp1.dw[0] = PositionFFLPF.Filter(Temp,&PositionFFLPF);		// Input in 16Q16 bit format; Output in 16Q16 bit format

    VelocityFBController.Ref = PositionFBController.Out+Temp1.w[1];	// 32Q0 bit format

    VelocityFBController.Fdbk = FB.VelocityM2;		// 32Q0 bit format

    VelocityFBController.Kp = VelocityFBController.Kp1;//Temp;
      
    VelocityFBController.Controller(&VelocityFBController);     // input - 32Q0 bit format; output 16Q16 bit format

    VelocityFBControllerOut = VelocityFBController.Out; // 16Q16 bit format

}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
