/**
  ******************************************************************************
  * @file    SysMonitor.c 
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
#include "SysMonitor.h"
#include "Flags.h"
#include "GPIO_functions.h"
//#include "TCN75_IIC_functions.h"
#include "HardwareConfiguration.h"
#include "CurrentControl.h"
#include "Velocity_Measure.h"
#include "Motor.h"
#include "PositionVelocityControl.h"
#include "ADS101x_IIC_functions.h"
#include "Parameters.h"
#include "CommInterface.h"
    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

struct Alarm AL;
struct Warninig WRN;
struct CommunicationErr COMMERR;
struct SysMonitor SM;

uint32_t status_word=0;

extern int16_t ExtVar[];
extern int8_t OutputsOverride;
extern GPIO_TypeDef* AL_Port;
extern uint16_t AL_Pin;
extern BitAction AL_Level;
extern int16_t Tpm, FAULTpm, FAULT_REGEN_Over;
extern int16_t FlagTpm, FlagFaultTpm, FlagFault_Regen_Over;
extern uint32_t IOInputReg;
extern int32_t DriverTemperature;

extern uint16_t        AL_Res_Pin;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void AlarmInit(void)
{
  REF.Velocity = 0;
  VelocityFBController.Fdbk = 0;
  AL.Source = 0;			// Clear Alarm Source
  AL.ResetRqstOld = AlarmClear();       // Set the Initial Alarm Reset Value
  AL.UmotTimer = 0;			// Reset Delay Timer Counter
  AL.Timer = 0;				// Reset Delay Timer2 Counter
  AL.TimeIndex = 0; 			// Reset Time Index Pointer
  AL.OverTemperatureTimer = 1000;
  AL.OverTemperatureTestNum = 0;
  MOT.Icurrenttime = 0;
    
  LED_ALM_OFF();			// Turn off Alarm Led
  
  AlarmRecordsLoad();
  
  WarningRecordsLoad();
  //------------------------- Flags Register -------------------------
  //Alarm_DATA(!Flags.bit.AlarmOutActiveLevel); // Alarm Reset
}
/*****************************************************************************/




/*****************************************************************************
// Alarm Led Flash Function
// Note: AL.TimeIndex increments to 1 at the beggining of every new 
// alarm event - Initial state with "ppTime" time and Led Off.
*****************************************************************************/
void AlarmLedFlash(void)
{
  if(AL.Source){ // Alarm is Turned on
    if(AL.Timer) AL.Timer--; // Decrementing Delay Timer Counter
    else{
      if((AL.Source << 1) <= (++AL.TimeIndex)) AL.TimeIndex = 0; // Reset Time Index Pointer
      if(AL.TimeIndex)  AL.Timer = ppTime;  // Reload Delay Timer Counter with Pulse/ Pause Time
      else		AL.Timer = pBreak;  // Reload Delay Timer Counter with Break Time
      LED_ALM_TOGGLE();			    // Alarm Led Flashing
    }
  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void AlarmOutSet(void)
{
  if(AL.Source){
     //if(!OutputsOverride) AlarmOut_Set;  // Alarm Set
     status_word |= FFLAG_ERRORALL;
  }
  else{}
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void VoltagesMonitor(void)
{
//++++++++++++++++++++++++ Motor OverVoltage Alarm +++++++++++++++++++++++++++
  //-- Generation Conditions: Motor Voltage over OV ---
  if(HC.Umot > DrvFlags1.bit.OverVoltageAlarmLevel){
    if(!AL.Source)  AL.Source = 2;	// Source Set
  }

//+++++++++++++++++++++ Motor/Input UnderVoltage Alarms ++++++++++++++++++++++
//-Generation Conditions: Under UV for more than 0.3 S or Instantly under UV/2-
  if(HC.Umot < DrvFlags1.bit.UnderVoltageAlarmLevel)	AL.UmotTimer++;
  else				AL.UmotTimer = 0;

  if((HC.Umot < (DrvFlags1.bit.UnderVoltageAlarmLevel >> 1)) || (AL.UmotTimer > 12000)){
    if((!AL.Source) && (CC.ServoOn))	AL.Source = 3;	// Source Set(works only in Servo On state to avoid fault alarms during Power off!!!)
  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void SysMonitorInit(void)
{
  SM.REFCNTOld = TIM_GetCounter(TIM4); 
  SM.Enable = 0;
  SM.TimeCntr = Timeout;
  SM.TestCntr = 0;//SMDelay;  // Init test counter
  SM.CurrModSum = 0;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void OverLoadMonitor(int16_t Temp)
{
  //ExtVar[0] = Temp;//<<<<<<<<<<<<<<<<<<<<<<<
    if((Temp <= -CC.RefLimit) || (Temp >= CC.RefLimit)){
    //if((Temp <= -400) || (Temp >= 400)){
      MOT.Icurrenttime++;
      if(MOT.Icurrenttime >= MOT.Itimeout)
        if(!AL.Source)	AL.Source = 1;    // Source Set
        status_word |= FFLAG_ERROVERLOAD;
      
    }
    else if(MOT.Icurrenttime > 0) MOT.Icurrenttime--;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void PowerModuleFaultMonitor(void)
{
  if(!FlagFaultTpm) return;
  FlagFaultTpm = 0;
  
  if((FAULTpm < 1024) && (CC.ServoOn)){
//    if(!AL.Source) AL.Source = 6;    // Source Set(works only in Servo On state to avoid fault alarms during Power off!!!)
  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void FaultRegenMonitor(void)
{
  if(!FlagFault_Regen_Over) return;
  FlagFault_Regen_Over = 0;
  
  if(FAULT_REGEN_Over > 1024){
    //if(!AL.Source) AL.Source = 8;    // Source Set
  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void AlarmRecordManagement(void)
{
uint16_t ALSource = AL.Source;
static uint16_t ALSourceOld = 0;

  if(ALSource && (ALSource != ALSourceOld)){

    AL.AlarmRecords[2].all <<= 8; // Throw away Record #11
    
    AL.AlarmRecords[2].bit.Record_0 = AL.AlarmRecords[1].bit.Record_3; // Save Record #7
    
    AL.AlarmRecords[1].all <<= 8; // Throw away Record #7
      
    AL.AlarmRecords[1].bit.Record_0 = AL.AlarmRecords[0].bit.Record_3; // Save Record #3
    
    AL.AlarmRecords[0].all <<= 8; // Throw away Record #3
    
    AL.AlarmRecords[0].bit.Record_0 = ALSource; // Save the present alarm
    
    AlarmRecordSave();
  }
  
  ALSourceOld = ALSource;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void WarningRecordManagement(void)
{
uint16_t WRNSource = WRN.Source;
static uint16_t WRNSourceOld = 0;

  if(WRNSource && (WRNSource != WRNSourceOld)){

    WRN.WarningRecords[2].all <<= 8; // Throw away Record #11
    
    WRN.WarningRecords[2].bit.Record_0 = WRN.WarningRecords[1].bit.Record_3; // Save Record #7
    
    WRN.WarningRecords[1].all <<= 8; // Throw away Record #7
      
    WRN.WarningRecords[1].bit.Record_0 = WRN.WarningRecords[0].bit.Record_3; // Save Record #3
    
    WRN.WarningRecords[0].all <<= 8; // Throw away Record #3
    
    WRN.WarningRecords[0].bit.Record_0 = WRNSource; // Save the present Warning
    
    WarningRecordSave();
  }
  
  WRNSourceOld = WRNSource;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void CommunicationErrorCodeSave(void)
{
uint16_t COMMERRSource = COMMERR.Source;
static uint16_t COMMERRSourceOld = 0;

  if(COMMERRSource && (COMMERRSource != COMMERRSourceOld)){

    COMMERR.Records[2].all <<= 8; // Throw away Record #11
    
    COMMERR.Records[2].bit.Record_0 = COMMERR.Records[1].bit.Record_3; // Save Record #7
    
    COMMERR.Records[1].all <<= 8; // Throw away Record #7
      
    COMMERR.Records[1].bit.Record_0 = COMMERR.Records[0].bit.Record_3; // Save Record #3
    
    COMMERR.Records[0].all <<= 8; // Throw away Record #3
    
    COMMERR.Records[0].bit.Record_0 = COMMERRSource; // Save the present Warning
  }
  
  COMMERRSourceOld = COMMERRSource;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void InPositionMonitor(void)
{
  if(PositionFBController.Ref == PositionFBController.Fdbk)
    InPos(Bit_RESET/*!Flags.bit.InPos*/);	    // "Out of Position" Signal
  else
    InPos(Bit_SET/*!Flags.bit.InPos*/);	    // "Out of Position" Signal
}
/******************************** END OF FILE *********************************/
