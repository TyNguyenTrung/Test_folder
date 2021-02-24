/**
  ******************************************************************************
  * @file    Background.c 
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
#include "Background.h"
#include "HardwareConfiguration.h"
#include "GPIO_functions.h"
#include "Flags.h"
#include "ServoON_OFF.h"
#include "CurrentControl.h"
#include "Command_functions.h"
#include "SysMonitor.h"
#include "IQmathlib.h"
//#include "Dumping.h"
#include "SIN_Tab.h"
#include "PWM_Functions.h"
//#include "EncResMeasure.h"
//#include "HarmonicsCalibration.h"
#include "PWM_Functions.h"
#include "CurrentControl.h"
#include "PositionVelocityControl.h"
#include "REFGenFunctions.h"
//#include "InertiaIdentification.h"
//#include "Observer.h"
#include "Menu.h"
#include "SIN_Tab.h"
#include "TypesDef.h"
#include "ADC_functions.h"
#include "PCA9632_IIC_functions.h"
#include "USART_functions.h"
#include "Plus-R_v.8_Slave.h"

extern int16_t ExtVar[];
extern s16 UqRef;
extern uint16_t ROM_MDB_Ptr;
int16_t Servo_En;
uint16_t USBSendEn, ComSendEn;
extern int32_t Speed_Buff; 	// command maximum speed (from RS232)
extern sDW PhAInterp;

uint32_t T5CNTR, _View;
uint16_t T5SR;
extern uint16_t Pos2;
uint16_t KeyTest;

//extern GPIO_TypeDef*   AL_Res_Port;
extern uint16_t        AL_Res_Pin;
int16_t Duty = 0;

extern int16_t FlagUmotsens, FlagTpm, FlagFaultTpm,FlagFault_Regen_Over;

extern volatile uint32_t TimingDelay, TimeOut;

extern uint32_t IOInputReg, IOInputRegMask, IOOutputReg;

//extern int16_t DevMass[50], DevMass2[50];
//int16_t DevMassptr=0;
//int32_t sInp, sOut;
int16_t _PhA = 0, _PhB, _PhC, _inc = 5, mul = 0, _delaytimer, delaytime = 50;
int32_t Uaref = 0, Ubref = 0, Ucref = 0;
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
void Background(void)
{

  //------ Measures and calculates Drivers' Voltages ------
    VoltagesCalculation();

  //-------------------- Alarm Reset ----------------------
    if(AL.ResetRqstOld != AlarmClear()){ // Detect Falling/Raising Front
      AL.ResetRqstOld =  AlarmClear();
      AlarmInit();
    }

  //---------------- Command Read Function ----------------
    CommandRead();

  //---------------- Command Read Function ----------------
    CommandManagement();
 
  //--------- Set Alarm Output  ------------
    AlarmOutSet();
    
  //--------------- Multiple Init Functions ----------------
    CurrentControlMultipleInit();  // Set the parameter for Current control
    PositionVelocityControlMultipleInit();   
    
  //------------------------------------
    if(ServoON()) Servo_En = 1;
    else          Servo_En = 0;
    
    
  //-------------------- Servo ON/OFF ---------------------
    if((Servo_En) && (AL.Source == 0)) Servo_ON();
    else                               Servo_OFF();
    
    
  //--------------------------
    InPositionMonitor();
}
/*****************************************************************************/




void VCP_bkg(uint32_t Time_mS)
{

}


extern int16_t MonMass1[], MonMass2[];
uint32_t MMPtr2=0;

void Bkg_1mS(void)
{
  static uint32_t Time_mS=0, Cnt_mS=0, Cnt_S=0; 
  
  if (Cnt_mS < 1000) Cnt_mS++;
  else { 
    Cnt_mS = 0; 
    Cnt_S++;
  }
  
  //---------- Alarm Status & Source Displaing ------------
    AlarmLedFlash();
  
  // 1mS routines
  VCP_bkg(Time_mS++);
  
  //
  ReceiveCommand();
  
  
  if(/*VelocityFBController.Ref*/USBSendEn)  { /*En = 0;*/ 
    //ExtVar[0] = MonMass1[MMPtr2];
    //ExtVar[1] = MonMass2[MMPtr2];
    //if(MMPtr2 < 19999)MMPtr2++;
    //else En = 0;
    //USB_DataSend(); 
  }
 
    //--------- Send Data via RS232 port ------------
    if(ComSendEn) USART_DataSend(COM2); 
}
/******************************** END OF FILE *********************************/
