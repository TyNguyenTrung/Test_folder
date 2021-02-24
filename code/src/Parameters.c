/**
  ******************************************************************************
  * @file    Parameters.c 
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
#include "Parameters.h"
#include "Flags.h"
#include "HardwareConfiguration.h"
#include "TypesDef.h"
#include "CurrentControl.h"
#include "SysMonitor.h"
#include "eeprom.h"
#include "GPIO_functions.h"
#include "Motor.h"
#include "PositionVelocityControl.h"
//#include "InertiaIdentification.h"
//#include "Observer.h"
#include "Velocity_Measure.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

union DRVFLAGS0 DrvFlags0;
union DRVFLAGS1 DrvFlags1;
union MFLAGS MFlags;
//union INPUT_FUNCTIONS InpFunc0, InpFunc1;
//union OUTPUT_FUNCTIONS OutFunc;
//union NET_IN_FUNCTIONS Net_In_Func[4];
//union NET_OUT_FUNCTIONS Net_Out_Func[4];

extern int16_t Kpff, ServoON;
//extern int16_t Iqref;
//extern int16_t RL_EstimateEn, RotorZeroPosEstimEn;

int32_t Reserved;
int32_t ReservedA0, ReservedA1, ReservedA2, ReservedA3;
int32_t ReservedB0, ReservedB1, ReservedB2, ReservedB3, ReservedB4;
int32_t ReservedC0, ReservedC1, ReservedC2;
int32_t ReservedD0, ReservedD1;
int32_t ReservedE0, ReservedE1;
int32_t ReservedF0, ReservedF1, ReservedF2;//, ReservedF3, ReservedF4, ReservedF5;
int32_t ReservedG0;//, ReservedG1, ReservedG2;
//int32_t /*IO_Input_Functions, */IO_Output_Functions;
int32_t /*BackEMFAlarmLevel,*/ DriverTemperature;
int32_t CurrentThreshold, AccelarationThreshold;
int32_t PPhCorrFhigh, RefPosSmootFTConst;
int32_t ZeroSpeed;
int32_t OvercurrentAlarmLevel, OverspeedAlarmLevel;
int32_t CheckSum;

//uint32_t NET_IN_Function_Reg[4], NET_OUT_Function_Reg[4]; 

uint16_t VirtAddVarTab[NB_OF_VAR];

//int32_t DriverTemperature;
//int32_t CurrentThreshold, AccelarationThreshold;
//int32_t PPhCorrFhigh, RefPosSmootFTConst;
int32_t ZeroSpeed;
int32_t OvercurrentAlarmLevel, OverspeedAlarmLevel;
int32_t CntrlFlags2, PosCompSignalRange, SpeedFilter, MovingAverageTime;
//int32_t JOG_Operating_Speed, JOG_Operating_AccDecel, JOG_Operating_Torque, JOG_Operating_Distance;
int32_t HomeSeekSpeed, HomeSeekAccDecel, HomeSeekPosOffset;
int32_t PosDevAlarm, OverVoltageWarning, PosDevWarning;
int32_t PositiveSoftLimit, NegativeSoftLimit, WrapRange;
int32_t SpeedReductionRatio, SpeedIncreasingRatio, OveloadWarningLevel;
int32_t AnalogSpeedGain, AnalogSpeedOffset, AnalogTorqueLimGain;
int32_t AnalogTorqueLimOffset, AnalogMaxSpeed, AnalogTorqueMaxLimit;
int32_t MS0_OpNoSelect, MS1_OpNoSelect, MS2_OpNoSelect;
int32_t MS3_OpNoSelect, MS4_OpNoSelect, MS5_OpNoSelect;

int32_t TotalInertia;                           // 8Q24 bit format;

extern int16_t EDCPtr;
extern int16_t EDCMas[];
extern uint16_t ROM_MDB_Ptr;
uint16_t DispAccelTime[16], DispDecelTime[16], DispDir, DisplayStart;
int16_t RefVelocity[16], TorqueLimit[16];
extern uint32_t IOInputsActiveLevel, IOOutputsActiveLevel;
extern uint16_t LevelPtr;
extern int16_t TorqueRatio;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/

void ParameterDefault(void)
{
      
    // Motor Parameters
      MOT.Number = 2;
      MOT.Type = 1;
      MOT.Ra = 358;  // 8Q8   1.4 Ohm 
      MOT.La = 37;   // 1Q15  1.12 mH
      MOT.Unom = 24;
      MOT.Inom = 1126;
      MOT.Imax = 3379; // 7Q9      3379/ (2^9) =~~ 6.5996 A
      MOT.Itimeout = 100000;
      MOT.TorqueConstant = 4021;  // ??? Maybe 16Q16 ; 0.061
      MOT.RotorInertia = 144;     // ??? Maybe 8Q24  ;  8.583x10^(-6)
      MOT.InertiaRatio = 196608; // 16Q16 bit format;  ==3
      MOT.ViscosityFriction = 20000; // 8Q24 bit format;
      MOT.RatedVelocity = 3000;
      MOT.EncRes = 2500;
      MOT.PolePairs = 5;
      
    // Current Control
      CC.Bandwidth = 1000;
      RefCurrentLPF.Bandwidth = 5000;
      
      
    // Velocity Control
      VelocityFBController.Kp1 = 255876;   // 16Q16  // 3.9
      VelocityFBController.Ki = 200;       // 16Q16  // 0.00305
      VelocityFBController.Kd = 32768;     // = 0.5*2^16  // //16Q16

      //LSM.InertiaRatio = 0;
      
    // Position Control
      PositionFBController.Kp = 198304;   //16Q16  // 3.025
      PositionFBController.Ki = 0;
      PositionFBController.Kd = 251072;   //16Q16  // 3.83
      Kpff = 16384;
      PositionFFLPF.Bandwidth = 900;	// [Hz] 16Q0 bit format
      
    // Extended Functions
      RefVelocity[0] = 50;      
      RefVelocity[1] = 50;     
      RefVelocity[2] = 50;     
      RefVelocity[3] = 50;   
      RefVelocity[4] = 50;   
      RefVelocity[5] = 50;      
      RefVelocity[6] = 50;       
      RefVelocity[7] = 50;    
      RefVelocity[8] = 50;      
      RefVelocity[9] = 50;     
      RefVelocity[10] = 50;     
      RefVelocity[11] = 50;   
      RefVelocity[12] = 50;   
      RefVelocity[13] = 50;      
      RefVelocity[14] = 50;       
      RefVelocity[15] = 50;      
      
      DispAccelTime[0] = 10000;
      DispAccelTime[1] = 10000;
      DispAccelTime[2] = 10000;
      DispAccelTime[3] = 10000;
      DispAccelTime[4] = 10000;
      DispAccelTime[5] = 10000;
      DispAccelTime[6] = 10000;
      DispAccelTime[7] = 10000;
      DispAccelTime[8] = 10000;
      DispAccelTime[9] = 10000;
      DispAccelTime[10] = 10000;
      DispAccelTime[11] = 10000;
      DispAccelTime[12] = 10000;
      DispAccelTime[13] = 10000;
      DispAccelTime[14] = 10000;
      DispAccelTime[15] = 10000;
      
      DispDecelTime[0] = 10000; 
      DispDecelTime[1] = 10000;
      DispDecelTime[2] = 10000;
      DispDecelTime[3] = 10000;
      DispDecelTime[4] = 10000;
      DispDecelTime[5] = 10000;
      DispDecelTime[6] = 10000;
      DispDecelTime[7] = 10000;
      DispDecelTime[8] = 10000; 
      DispDecelTime[9] = 10000;
      DispDecelTime[10] = 10000;
      DispDecelTime[11] = 10000;
      DispDecelTime[12] = 10000;
      DispDecelTime[13] = 10000;
      DispDecelTime[14] = 10000;
      DispDecelTime[15] = 10000;
      
      TorqueLimit[0] = 300;   
      TorqueLimit[1] = 300; 
      TorqueLimit[2] = 300; 
      TorqueLimit[3] = 300; 
      TorqueLimit[4] = 300; 
      TorqueLimit[5] = 300; 
      TorqueLimit[6] = 300; 
      TorqueLimit[7] = 300; 
      TorqueLimit[8] = 300;   
      TorqueLimit[9] = 300; 
      TorqueLimit[10] = 300; 
      TorqueLimit[11] = 300; 
      TorqueLimit[12] = 300; 
      TorqueLimit[13] = 300; 
      TorqueLimit[14] = 300; 
      TorqueLimit[15] = 300; 
      
    // Driver Parameters
      HC.Isensor = 655; // 1Q15 bit format
      HC.Kiamp = 12974;	// 6.3348 in 5Q11 bit format
      HC.UsensorUp = 9760;
      HC.UsensorDown = 240;
      HC.BackEMFCntrlLevel = 350;
      AL.DriverOvertempAlLevel = 70;
     
    // Control Parameters
      DrvFlags0.all = 0;
      DrvFlags1.all = 0x973F2064;
      
      DrvFlags0.bit.InterfaceMode = 0;  
      DrvFlags0.bit.MotionDirection = 0;

      DrvFlags1.bit.UnderVoltageAlarmLevel = 12;
      DrvFlags1.bit.OverVoltageAlarmLevel = 80;
      DrvFlags1.bit.AlarmResetActiveLevel = 1;
      DrvFlags1.bit.AlarmOutActiveLevel = 1;	
      DrvFlags1.bit.ServoOnActiveLevel = 1;

      // Other parameters

}

/*
void ParameterDefault(void)
{
      
    // Motor Parameters
      MOT.Number = 2;
      MOT.Type = 1;
      MOT.Ra = 77;//358;
      MOT.La = 14;//37;
      MOT.Unom = 24;
      MOT.Inom = 3686;//1126;
      MOT.Imax = 11059;//3379; // 7Q9
      MOT.Itimeout = 100000;
      MOT.TorqueConstant = 4011;//4021;
      MOT.RotorInertia = 1023;//144;
      MOT.InertiaRatio = 196608; // 16Q16 bit format;
      MOT.ViscosityFriction = 20000; // 8Q24 bit format;
      MOT.RatedVelocity = 3000;
      MOT.EncRes = 2500;
      MOT.PolePairs = 5;
      
    // Current Control
      CC.Bandwidth = 1000;
      RefCurrentLPF.Bandwidth = 5000;
      
      
    // Velocity Control
      VelocityFBController.Kp1 = 255876;
      VelocityFBController.Ki = 200;
      VelocityFBController.Kd = 32768;

      //LSM.InertiaRatio = 0;
      
    // Position Control
      PositionFBController.Kp = 198304;
      PositionFBController.Ki = 0;
      PositionFBController.Kd = 251072;
      Kpff = 16384;
      PositionFFLPF.Bandwidth = 900;	// [Hz] 16Q0 bit format
      
    // Extended Functions
      RefVelocity[0] = 50;      
      RefVelocity[1] = 50;     
      RefVelocity[2] = 50;     
      RefVelocity[3] = 50;   
      RefVelocity[4] = 50;   
      RefVelocity[5] = 50;      
      RefVelocity[6] = 50;       
      RefVelocity[7] = 50;    
      RefVelocity[8] = 50;      
      RefVelocity[9] = 50;     
      RefVelocity[10] = 50;     
      RefVelocity[11] = 50;   
      RefVelocity[12] = 50;   
      RefVelocity[13] = 50;      
      RefVelocity[14] = 50;       
      RefVelocity[15] = 50;      
      
      DispAccelTime[0] = 10000;
      DispAccelTime[1] = 10000;
      DispAccelTime[2] = 10000;
      DispAccelTime[3] = 10000;
      DispAccelTime[4] = 10000;
      DispAccelTime[5] = 10000;
      DispAccelTime[6] = 10000;
      DispAccelTime[7] = 10000;
      DispAccelTime[8] = 10000;
      DispAccelTime[9] = 10000;
      DispAccelTime[10] = 10000;
      DispAccelTime[11] = 10000;
      DispAccelTime[12] = 10000;
      DispAccelTime[13] = 10000;
      DispAccelTime[14] = 10000;
      DispAccelTime[15] = 10000;
      
      DispDecelTime[0] = 10000; 
      DispDecelTime[1] = 10000;
      DispDecelTime[2] = 10000;
      DispDecelTime[3] = 10000;
      DispDecelTime[4] = 10000;
      DispDecelTime[5] = 10000;
      DispDecelTime[6] = 10000;
      DispDecelTime[7] = 10000;
      DispDecelTime[8] = 10000; 
      DispDecelTime[9] = 10000;
      DispDecelTime[10] = 10000;
      DispDecelTime[11] = 10000;
      DispDecelTime[12] = 10000;
      DispDecelTime[13] = 10000;
      DispDecelTime[14] = 10000;
      DispDecelTime[15] = 10000;
      
      TorqueLimit[0] = 300;   
      TorqueLimit[1] = 300; 
      TorqueLimit[2] = 300; 
      TorqueLimit[3] = 300; 
      TorqueLimit[4] = 300; 
      TorqueLimit[5] = 300; 
      TorqueLimit[6] = 300; 
      TorqueLimit[7] = 300; 
      TorqueLimit[8] = 300;   
      TorqueLimit[9] = 300; 
      TorqueLimit[10] = 300; 
      TorqueLimit[11] = 300; 
      TorqueLimit[12] = 300; 
      TorqueLimit[13] = 300; 
      TorqueLimit[14] = 300; 
      TorqueLimit[15] = 300; 
      
    // Driver Parameters
      HC.Isensor = 655; // 1Q15 bit format
      HC.Kiamp = 12974;	// 6.3348 in 5Q11 bit format
      HC.UsensorUp = 9760;
      HC.UsensorDown = 240;
      HC.BackEMFCntrlLevel = 350;
      AL.DriverOvertempAlLevel = 70;
     
    // Control Parameters
      DrvFlags0.all = 0;
      DrvFlags1.all = 0x973F2064;
      
      DrvFlags0.bit.InterfaceMode = 0;  
      DrvFlags0.bit.MotionDirection = 0;

      DrvFlags1.bit.UnderVoltageAlarmLevel = 12;
      DrvFlags1.bit.OverVoltageAlarmLevel = 80;
      DrvFlags1.bit.AlarmResetActiveLevel = 1;
      DrvFlags1.bit.AlarmOutActiveLevel = 1;	
      DrvFlags1.bit.ServoOnActiveLevel = 1;

      // Other parameters

}
*/
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t ParameterSet(int16_t ptr, int32_t Value)
{
int16_t Err = 0;

  switch(ptr){
      
case 17:            Reserved = Value;             break;
    // Motor Parameters
      case Addr_MotorNumber:            MOT.Number = Value;             break;
      case Addr_MotorType:              MOT.Type = Value;               break;
      case Addr_LinetoLineResistance:   MOT.Ra = Value;                 break;
      case Addr_LinetoLineInductance:	MOT.La = Value;                 break;
      case Addr_RatedVoltage:	        MOT.Unom = Value;               break;
      case Addr_RatedCurrent:	        MOT.Inom = Value;               break;
      case Addr_MaxPeakCurrent:	        MOT.Imax = Value;               break;
      case Addr_PeakCurrentAlrmTimeout:	MOT.Itimeout = Value;           break;
      case Addr_TorqueConstant:	        MOT.TorqueConstant = Value;     break;
      case Addr_RotorInertia:	        MOT.RotorInertia = Value;       break;
      case Addr_InertiaRatio:	        MOT.InertiaRatio = Value;	break;
      case Addr_ViscosityFriction:	MOT.ViscosityFriction = Value;  break;
      case Addr_RatedVelocity:	        MOT.RatedVelocity = Value;      break;
      case Addr_EncoderResolution:	MOT.EncRes = Value;             break;
      case Addr_MotorPolePairs:	        MOT.PolePairs = Value;          break;
      case Addr_ReservedF0:	        Reserved = Value;               break;
      case Addr_ReservedF1:	        Reserved = Value;               break;
      
    // Current Control
      case Addr_LoopBandwidth:	        CC.Bandwidth = Value;	        break;
      case Addr_ReferenceLPFBandwidth:	RefCurrentLPF.Bandwidth = Value; break;
      case Addr_ReservedE0:	        Reserved = Value;	        break;
      case Addr_ReservedE1:	        Reserved = Value;	        break;
      case Addr_ReservedE2:	        Reserved = Value;	        break;
      
      // Velocity Control
      case Addr_VelocityKp:	VelocityFBController.Kp1 = Value;    break;
      case Addr_VelocityKi:	VelocityFBController.Ki = Value;     break;
      case Addr_VelocityKd:	VelocityFBController.Kd = Value;     break;
      case Addr_ReservedD0:	Reserved = Value;	             break;
      case Addr_ReservedD1:	Reserved = Value;	             break;
      
    // Observer Parameters
      //case Addr_F12:		        OBSRV.F12 = Value;	break;
      //case Addr_F23:		        OBSRV.F23 = Value;	break;
      //case Addr_Lc11:		        OBSRV.Lc11 = Value;	break;
      //case Addr_Lc21:		        OBSRV.Lc21 = Value;	break;
      //case Addr_Lc31:		        OBSRV.Lc31 = Value;	break;
      case Addr_ReservedD2:		Reserved = Value;	break;
      
    // Position Control
      case Addr_PositionKp:	PositionFBController.Kp = Value;     break;
      case Addr_PositionKi:	PositionFBController.Ki = Value;     break;
      case Addr_PositionKd:	PositionFBController.Kd = Value;     break;
      case Addr_PFFGain:	Kpff = Value;			     break;
      case Addr_PFFLPFBandwidth:PositionFFLPF.Bandwidth = Value;     break;
      case Addr_ReservedD3:	ReservedD0 = Value;                  break;
      case Addr_ReservedD4:	ReservedD0 = Value;                  break;
      
    // Extended Functions
      case Addr_Velocity0:	RefVelocity[0] = Value;         break;
      case Addr_Velocity1:	RefVelocity[1] = Value;         break;
      case Addr_Velocity2:	RefVelocity[2] = Value;         break;
      case Addr_Velocity3:	RefVelocity[3] = Value;         break;
      case Addr_Velocity4:	RefVelocity[4] = Value;         break;
      case Addr_Velocity5:	RefVelocity[5] = Value;         break;
      case Addr_Velocity6:	RefVelocity[6] = Value;         break;
      case Addr_Velocity7:	RefVelocity[7] = Value;         break;
      case Addr_Velocity8:	RefVelocity[8] = Value;         break;
      case Addr_Velocity9:	RefVelocity[9] = Value;         break;
      case Addr_Velocity10:	RefVelocity[10] = Value;         break;
      case Addr_Velocity11:	RefVelocity[11] = Value;         break;
      case Addr_Velocity12:	RefVelocity[12] = Value;         break;
      case Addr_Velocity13:	RefVelocity[13] = Value;         break;
      case Addr_Velocity14:	RefVelocity[14] = Value;         break;
      case Addr_Velocity15:	RefVelocity[15] = Value;         break;
      
      
      case Addr_Acceleration0:	DispAccelTime[0] = Value;       break;
      case Addr_Acceleration1:	DispAccelTime[1] = Value;       break;
      case Addr_Acceleration2:	DispAccelTime[2] = Value;       break;
      case Addr_Acceleration3:	DispAccelTime[3] = Value;       break;
      case Addr_Acceleration4:	DispAccelTime[4] = Value;       break;
      case Addr_Acceleration5:	DispAccelTime[5] = Value;       break;
      case Addr_Acceleration6:	DispAccelTime[6] = Value;       break;
      case Addr_Acceleration7:	DispAccelTime[7] = Value;       break;
      case Addr_Acceleration8:	DispAccelTime[8] = Value;       break;
      case Addr_Acceleration9:	DispAccelTime[9] = Value;       break;
      case Addr_Acceleration10:	DispAccelTime[10] = Value;      break;
      case Addr_Acceleration11:	DispAccelTime[11] = Value;      break;
      case Addr_Acceleration12:	DispAccelTime[12] = Value;      break;
      case Addr_Acceleration13:	DispAccelTime[13] = Value;      break;
      case Addr_Acceleration14:	DispAccelTime[14] = Value;      break;
      case Addr_Acceleration15:	DispAccelTime[15] = Value;      break;
      
      case Addr_Deceleration0:	DispDecelTime[0] = Value;       break;
      case Addr_Deceleration1:	DispDecelTime[1] = Value;       break;
      case Addr_Deceleration2:	DispDecelTime[2] = Value;       break;
      case Addr_Deceleration3:	DispDecelTime[3] = Value;       break;
      case Addr_Deceleration4:	DispDecelTime[4] = Value;       break;
      case Addr_Deceleration5:	DispDecelTime[5] = Value;       break;
      case Addr_Deceleration6:	DispDecelTime[6] = Value;       break;
      case Addr_Deceleration7:	DispDecelTime[7] = Value;       break;
      case Addr_Deceleration8:	DispDecelTime[8] = Value;       break;
      case Addr_Deceleration9:	DispDecelTime[9] = Value;       break;
      case Addr_Deceleration10:	DispDecelTime[10] = Value;      break;
      case Addr_Deceleration11:	DispDecelTime[11] = Value;      break;
      case Addr_Deceleration12:	DispDecelTime[12] = Value;      break;
      case Addr_Deceleration13:	DispDecelTime[13] = Value;      break;
      case Addr_Deceleration14:	DispDecelTime[14] = Value;      break;
      case Addr_Deceleration15:	DispDecelTime[15] = Value;      break;
      
      case Addr_TorqueLimit0:	TorqueLimit[0] = Value;         break;
      case Addr_TorqueLimit1:	TorqueLimit[1] = Value;         break;
      case Addr_TorqueLimit2:	TorqueLimit[2] = Value;         break;
      case Addr_TorqueLimit3:	TorqueLimit[3] = Value;         break;
      case Addr_TorqueLimit4:	TorqueLimit[4] = Value;         break;
      case Addr_TorqueLimit5:	TorqueLimit[5] = Value;         break;
      case Addr_TorqueLimit6:	TorqueLimit[6] = Value;         break;
      case Addr_TorqueLimit7:	TorqueLimit[7] = Value;         break;
      case Addr_TorqueLimit8:	TorqueLimit[8] = Value;         break;
      case Addr_TorqueLimit9:	TorqueLimit[9] = Value;         break;
      case Addr_TorqueLimit10:	TorqueLimit[10] = Value;        break;
      case Addr_TorqueLimit11:	TorqueLimit[11] = Value;        break;
      case Addr_TorqueLimit12:	TorqueLimit[12] = Value;        break;
      case Addr_TorqueLimit13:	TorqueLimit[13] = Value;        break;
      case Addr_TorqueLimit14:	TorqueLimit[14] = Value;        break;
      case Addr_TorqueLimit15:	TorqueLimit[15] = Value;        break;
      
      case Addr_ReservedC0:	Reserved = Value;               break;
      case Addr_ReservedC1:	Reserved = Value;               break;
      
    // Driver Parameters
      case Addr_Isensor:		HC.Isensor = Value;	break;
      case Addr_Kiamp:			HC.Kiamp = Value;	break;
      case Addr_Usensorup:		HC.UsensorUp = Value;	break;
      case Addr_Usensordown:		HC.UsensorDown = Value;	break;
      case Addr_BackEMFCntrlLevel:	HC.BackEMFCntrlLevel = Value;	break;
      case Addr_DriverOvertempAlLevel:	AL.DriverOvertempAlLevel  = Value;	break;
      case Addr_ReservedG0:		Reserved = Value;	break;
     
    // Control Parameters
      case Addr_DrvFlags0:		DrvFlags0.all  = Value;	break;
      case Addr_DrvFlags1:		DrvFlags1.all  = Value;	break;
      case Addr_ReservedG1:		Reserved = Value;	break;
      case Addr_ReservedG2:		Reserved = Value;	break;
     
      case Addr_CheckSum:		CheckSum = Value;	break;
      
    // Alarms and Worning records, Monitoring Read Only Parameters
      case Addr_PresentAlarm:	AL.Source                       = Value;        break;
      case Addr_AlarmRecord1:   AL.AlarmRecords[0].bit.Record_1 = Value;	break;
      case Addr_AlarmRecord2:	AL.AlarmRecords[0].bit.Record_2 = Value;	break;
      case Addr_AlarmRecord3:	AL.AlarmRecords[0].bit.Record_3 = Value;	break;
      case Addr_AlarmRecord4:	AL.AlarmRecords[1].bit.Record_0 = Value;	break;
      case Addr_AlarmRecord5:	AL.AlarmRecords[1].bit.Record_1 = Value;	break;
      case Addr_AlarmRecord6:	AL.AlarmRecords[1].bit.Record_2 = Value;	break;
      case Addr_AlarmRecord7:	AL.AlarmRecords[1].bit.Record_3 = Value;	break;
      case Addr_AlarmRecord8:	AL.AlarmRecords[2].bit.Record_0 = Value;	break;
      case Addr_AlarmRecord9:	AL.AlarmRecords[2].bit.Record_1 = Value;	break;
      case Addr_AlarmRecord10:	AL.AlarmRecords[2].bit.Record_2 = Value;	break;
      case Addr_AlarmRecord11:	AL.AlarmRecords[2].bit.Record_3 = Value;	break;
      
      case Addr_AlarmRecordRegister0:	AL.AlarmRecords[0].all = Value;	        break;
      case Addr_AlarmRecordRegister1:	AL.AlarmRecords[1].all = Value;	        break;
      case Addr_AlarmRecordRegister2:	AL.AlarmRecords[2].all = Value;	        break;
      
      case Addr_PresentWarning:	 WRN.Source                         = Value;	break;
      case Addr_WarningRecord1:	 WRN.WarningRecords[0].bit.Record_1 = Value;	break;
      case Addr_WarningRecord2:	 WRN.WarningRecords[0].bit.Record_2 = Value;	break;
      case Addr_WarningRecord3:	 WRN.WarningRecords[0].bit.Record_3 = Value;	break;
      case Addr_WarningRecord4:	 WRN.WarningRecords[1].bit.Record_0 = Value;	break;
      case Addr_WarningRecord5:	 WRN.WarningRecords[1].bit.Record_1 = Value;	break;
      case Addr_WarningRecord6:	 WRN.WarningRecords[1].bit.Record_2 = Value;	break;
      case Addr_WarningRecord7:	 WRN.WarningRecords[1].bit.Record_3 = Value;	break;
      case Addr_WarningRecord8:	 WRN.WarningRecords[2].bit.Record_0 = Value;	break;
      case Addr_WarningRecord9:	 WRN.WarningRecords[2].bit.Record_1 = Value;	break;
      case Addr_WarningRecord10: WRN.WarningRecords[2].bit.Record_2 = Value;	break;
      case Addr_WarningRecord11: WRN.WarningRecords[2].bit.Record_3 = Value;	break;
      
      case Addr_WarningRecordRegister0: WRN.WarningRecords[0].all = Value;	break;
      case Addr_WarningRecordRegister1: WRN.WarningRecords[1].all = Value;	break;
      case Addr_WarningRecordRegister2: WRN.WarningRecords[2].all = Value;	break;
      
      case Addr_CommunicationErrorCode:	        COMMERR.Source                  = Value;  break;
      case Addr_CommunicationErrorCodeRecord1:	COMMERR.Records[0].bit.Record_1 = Value;  break;
      case Addr_CommunicationErrorCodeRecord2:	COMMERR.Records[0].bit.Record_2 = Value;  break;
      case Addr_CommunicationErrorCodeRecord3:	COMMERR.Records[0].bit.Record_3 = Value;  break;
      case Addr_CommunicationErrorCodeRecord4:	COMMERR.Records[1].bit.Record_0 = Value;  break;
      case Addr_CommunicationErrorCodeRecord5:	COMMERR.Records[1].bit.Record_1 = Value;  break;
      case Addr_CommunicationErrorCodeRecord6:	COMMERR.Records[1].bit.Record_2 = Value;  break;
      case Addr_CommunicationErrorCodeRecord7:	COMMERR.Records[1].bit.Record_3 = Value;  break;
      case Addr_CommunicationErrorCodeRecord8:	COMMERR.Records[2].bit.Record_0 = Value;  break;
      case Addr_CommunicationErrorCodeRecord9:	COMMERR.Records[2].bit.Record_1 = Value;  break;
      case Addr_CommunicationErrorCodeRecord10:	COMMERR.Records[2].bit.Record_2 = Value;  break;
      case Addr_CommunicationErrorCodeRecord11:	COMMERR.Records[2].bit.Record_3 = Value;  break;
      
      case Addr_CommunicationErrorCodeRegister0:COMMERR.Records[0].all = Value;  break;
      case Addr_CommunicationErrorCodeRegister1:COMMERR.Records[1].all = Value;  break;
      case Addr_CommunicationErrorCodeRegister2:COMMERR.Records[2].all = Value;  break;
      
    // Adresses for Parameters Direct Access
      //case Addr_StartStop:	        DrvFlags0.bit.StartStop = Value;	break;
      case Addr_InterfaceModes:	        DrvFlags0.bit.InterfaceMode = Value;	break;
      case Addr_MotionDirection:	DrvFlags0.bit.MotionDirection = Value;	break;
      case Addr_BrakeFunctionAtAlarm:	DrvFlags0.bit.BrakeFunctionAtAlarm              = Value;	break;
      case Addr_NoOperateAtInitAlarmEnable: DrvFlags0.bit.NoOperateAtInitAlarmEnable    = Value;	break;
      case Addr_InitialThermalInputDetection: DrvFlags0.bit.InitialThermalInputDetection = Value;	break;
      case Addr_OverLoadWarningEnable: DrvFlags0.bit.OverLoadWarningEnable              = Value;        break;
      case Addr_AnalogInputSignalSelect: DrvFlags0.bit.AnalogInputSignalSelect          = Value;	break;
      
      case Addr_UnderVoltageAlarmLevel:	DrvFlags1.bit.UnderVoltageAlarmLevel = Value;	break;
      case Addr_OverVoltageAlarmLevel:	DrvFlags1.bit.OverVoltageAlarmLevel  = Value;	break;
      case Addr_AlarmResetActiveLevel:	DrvFlags1.bit.AlarmResetActiveLevel  = Value;	break;
      case Addr_AlarmOutActiveLevel:	DrvFlags1.bit.AlarmOutActiveLevel    = Value;	break;
      case Addr_ServoOnActiveLevel:	DrvFlags1.bit.ServoOnActiveLevel     = Value;	break;
             

      // Other parameters
      
      default:		          Err = -1;
    }
	
    return(Err);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t ParameterGet(int16_t ptr, uint32_t *Value)
{
int16_t Err = 0;

  switch(ptr){
      
    // Motor Parameters
      case Addr_MotorNumber:            *Value = MOT.Number;             break;
      case Addr_MotorType:              *Value = MOT.Type;               break;
      case Addr_LinetoLineResistance:   *Value = MOT.Ra;                 break;
      case Addr_LinetoLineInductance:	*Value = MOT.La;                 break;
      case Addr_RatedVoltage:	        *Value = MOT.Unom;               break;
      case Addr_RatedCurrent:	        *Value = MOT.Inom;               break;
      case Addr_MaxPeakCurrent:	        *Value = MOT.Imax;               break;
      case Addr_PeakCurrentAlrmTimeout:	*Value = MOT.Itimeout;           break;
      case Addr_TorqueConstant:	        *Value = MOT.TorqueConstant;     break;
      case Addr_RotorInertia:	        *Value = MOT.RotorInertia;       break;
      case Addr_InertiaRatio:	        *Value = MOT.InertiaRatio;       break;
      case Addr_ViscosityFriction:      *Value = MOT.ViscosityFriction;  break;
      case Addr_RatedVelocity:	        *Value = MOT.RatedVelocity;      break;
      case Addr_EncoderResolution:	*Value = MOT.EncRes;             break;
      case Addr_MotorPolePairs:	        *Value = MOT.PolePairs;          break;
      case Addr_ReservedF0:	        *Value = Reserved;               break;
      case Addr_ReservedF1:	        *Value = Reserved;               break;
      
    // Current Control
      case Addr_LoopBandwidth:	        *Value = CC.Bandwidth;	        break;
      case Addr_ReferenceLPFBandwidth:	*Value = RefCurrentLPF.Bandwidth; break;
      case Addr_ReservedE0:	        *Value = Reserved;	        break;
      case Addr_ReservedE1:	        *Value = Reserved;	        break;
      case Addr_ReservedE2:	        *Value = Reserved;	        break;
      
      // Velocity Control
      case Addr_VelocityKp:	*Value = VelocityFBController.Kp1;    break;
      case Addr_VelocityKi:	*Value = VelocityFBController.Ki;     break;
      case Addr_VelocityKd:	*Value = VelocityFBController.Kd;     break;
      case Addr_ReservedD0:	*Value = Reserved;	              break;
      case Addr_ReservedD1:	*Value = Reserved;	              break;
           
    // Observer Parameters
      //case Addr_F12:		*Value = OBSRV.F12;	break;
      //case Addr_F23:		*Value = OBSRV.F23;	break;
      //case Addr_Lc11:		*Value = OBSRV.Lc11;	break;
      //case Addr_Lc21:		*Value = OBSRV.Lc21;	break;
      //case Addr_Lc31:		*Value = OBSRV.Lc31;	break;
      case Addr_ReservedD2:	*Value = Reserved;	break;
      
    // Position Control
      case Addr_PositionKp:	*Value = PositionFBController.Kp;     break;
      case Addr_PositionKi:	*Value = PositionFBController.Ki;     break;
      case Addr_PositionKd:	*Value = PositionFBController.Kd;     break;
      case Addr_PFFGain:	*Value = Kpff;			      break;
      case Addr_PFFLPFBandwidth:*Value = PositionFFLPF.Bandwidth;     break;
      case Addr_ReservedD3:	*Value = ReservedD0;                  break;
      case Addr_ReservedD4:	*Value = ReservedD0;                  break;
   
    // Extended Functions
      case Addr_Velocity0:	*Value = RefVelocity[0];         break;
      case Addr_Velocity1:	*Value = RefVelocity[1];         break;
      case Addr_Velocity2:	*Value = RefVelocity[2];         break;
      case Addr_Velocity3:	*Value = RefVelocity[3];         break;
      case Addr_Velocity4:	*Value = RefVelocity[4];         break;
      case Addr_Velocity5:	*Value = RefVelocity[5];         break;
      case Addr_Velocity6:	*Value = RefVelocity[6];         break;
      case Addr_Velocity7:	*Value = RefVelocity[7];         break;
      case Addr_Velocity8:	*Value = RefVelocity[8];         break;
      case Addr_Velocity9:	*Value = RefVelocity[9];         break;
      case Addr_Velocity10:	*Value = RefVelocity[10];        break;
      case Addr_Velocity11:	*Value = RefVelocity[11];        break;
      case Addr_Velocity12:	*Value = RefVelocity[12];        break;
      case Addr_Velocity13:	*Value = RefVelocity[13];        break;
      case Addr_Velocity14:	*Value = RefVelocity[14];        break;
      case Addr_Velocity15:	*Value = RefVelocity[15];        break;
      
      
      case Addr_Acceleration0:	*Value = DispAccelTime[0];       break;
      case Addr_Acceleration1:	*Value = DispAccelTime[1];       break;
      case Addr_Acceleration2:	*Value = DispAccelTime[2];       break;
      case Addr_Acceleration3:	*Value = DispAccelTime[3];       break;
      case Addr_Acceleration4:	*Value = DispAccelTime[4];       break;
      case Addr_Acceleration5:	*Value = DispAccelTime[5];       break;
      case Addr_Acceleration6:	*Value = DispAccelTime[6];       break;
      case Addr_Acceleration7:	*Value = DispAccelTime[7];       break;
      case Addr_Acceleration8:	*Value = DispAccelTime[8];       break;
      case Addr_Acceleration9:	*Value = DispAccelTime[9];       break;
      case Addr_Acceleration10:	*Value = DispAccelTime[10];      break;
      case Addr_Acceleration11:	*Value = DispAccelTime[11];      break;
      case Addr_Acceleration12:	*Value = DispAccelTime[12];      break;
      case Addr_Acceleration13:	*Value = DispAccelTime[13];      break;
      case Addr_Acceleration14:	*Value = DispAccelTime[14];      break;
      case Addr_Acceleration15:	*Value = DispAccelTime[15];      break;
      
      case Addr_Deceleration0:	*Value = DispDecelTime[0];       break;
      case Addr_Deceleration1:	*Value = DispDecelTime[1];       break;
      case Addr_Deceleration2:	*Value = DispDecelTime[2];       break;
      case Addr_Deceleration3:	*Value = DispDecelTime[3];       break;
      case Addr_Deceleration4:	*Value = DispDecelTime[4];       break;
      case Addr_Deceleration5:	*Value = DispDecelTime[5];       break;
      case Addr_Deceleration6:	*Value = DispDecelTime[6];       break;
      case Addr_Deceleration7:	*Value = DispDecelTime[7];       break;
      case Addr_Deceleration8:	*Value = DispDecelTime[8];       break;
      case Addr_Deceleration9:	*Value = DispDecelTime[9];       break;
      case Addr_Deceleration10:	*Value = DispDecelTime[10];      break;
      case Addr_Deceleration11:	*Value = DispDecelTime[11];      break;
      case Addr_Deceleration12:	*Value = DispDecelTime[12];      break;
      case Addr_Deceleration13:	*Value = DispDecelTime[13];      break;
      case Addr_Deceleration14:	*Value = DispDecelTime[14];      break;
      case Addr_Deceleration15:	*Value = DispDecelTime[15];      break;
      
      case Addr_TorqueLimit0:	*Value = TorqueLimit[0];         break;
      case Addr_TorqueLimit1:	*Value = TorqueLimit[1];         break;
      case Addr_TorqueLimit2:	*Value = TorqueLimit[2];         break;
      case Addr_TorqueLimit3:	*Value = TorqueLimit[3];         break;
      case Addr_TorqueLimit4:	*Value = TorqueLimit[4];         break;
      case Addr_TorqueLimit5:	*Value = TorqueLimit[5];         break;
      case Addr_TorqueLimit6:	*Value = TorqueLimit[6];         break;
      case Addr_TorqueLimit7:	*Value = TorqueLimit[7];         break;
      case Addr_TorqueLimit8:	*Value = TorqueLimit[8];         break;
      case Addr_TorqueLimit9:	*Value = TorqueLimit[9];         break;
      case Addr_TorqueLimit10:	*Value = TorqueLimit[10];        break;
      case Addr_TorqueLimit11:	*Value = TorqueLimit[11];        break;
      case Addr_TorqueLimit12:	*Value = TorqueLimit[12];        break;
      case Addr_TorqueLimit13:	*Value = TorqueLimit[13];        break;
      case Addr_TorqueLimit14:	*Value = TorqueLimit[14];        break;
      case Addr_TorqueLimit15:	*Value = TorqueLimit[15];        break;
      
      case Addr_ReservedC0:	*Value = Reserved;               break;
      case Addr_ReservedC1:	*Value = Reserved;               break;
      
    // Driver Parameters
      case Addr_Isensor:		*Value = HC.Isensor;	break;
      case Addr_Kiamp:			*Value = HC.Kiamp;	break;
      case Addr_Usensorup:		*Value = HC.UsensorUp;	break;
      case Addr_Usensordown:		*Value = HC.UsensorDown;	   break;
      case Addr_BackEMFCntrlLevel:	*Value = HC.BackEMFCntrlLevel;	   break;
      case Addr_DriverOvertempAlLevel:	*Value = AL.DriverOvertempAlLevel; break;
      case Addr_ReservedG0:		*Value = Reserved;	           break;
     
    // Control Parameters
      case Addr_DrvFlags0:		*Value = DrvFlags0.all;	        break;
      case Addr_DrvFlags1:		*Value = DrvFlags1.all;	        break;
      case Addr_ReservedG1:		*Value = Reserved;	        break;
      case Addr_ReservedG2:		*Value = Reserved;	        break;
      
      case Addr_CheckSum:		*Value = CheckSum;	        break;
      
    // Alarms and Worning records, Monitoring Read Only Parameters
      case Addr_PresentAlarm:	*Value = AL.Source;                             break;
      case Addr_AlarmRecord1:   *Value = AL.AlarmRecords[0].bit.Record_1;	break;
      case Addr_AlarmRecord2:	*Value = AL.AlarmRecords[0].bit.Record_2;	break;
      case Addr_AlarmRecord3:	*Value = AL.AlarmRecords[0].bit.Record_3;	break;
      case Addr_AlarmRecord4:	*Value = AL.AlarmRecords[1].bit.Record_0;	break;
      case Addr_AlarmRecord5:	*Value = AL.AlarmRecords[1].bit.Record_1;	break;
      case Addr_AlarmRecord6:	*Value = AL.AlarmRecords[1].bit.Record_2;	break;
      case Addr_AlarmRecord7:	*Value = AL.AlarmRecords[1].bit.Record_3;	break;
      case Addr_AlarmRecord8:	*Value = AL.AlarmRecords[2].bit.Record_0;	break;
      case Addr_AlarmRecord9:	*Value = AL.AlarmRecords[2].bit.Record_1;	break;
      case Addr_AlarmRecord10:	*Value = AL.AlarmRecords[2].bit.Record_2;	break;
      case Addr_AlarmRecord11:	*Value = AL.AlarmRecords[2].bit.Record_3;	break;
      
      case Addr_AlarmRecordRegister0:	*Value = AL.AlarmRecords[0].all;	break;
      case Addr_AlarmRecordRegister1:	*Value = AL.AlarmRecords[1].all;	break;
      case Addr_AlarmRecordRegister2:	*Value = AL.AlarmRecords[2].all;	break;
      
      case Addr_PresentWarning:	 *Value = WRN.Source;	break;
      case Addr_WarningRecord1:	 *Value = WRN.WarningRecords[0].bit.Record_1;	break;
      case Addr_WarningRecord2:	 *Value = WRN.WarningRecords[0].bit.Record_2;	break;
      case Addr_WarningRecord3:	 *Value = WRN.WarningRecords[0].bit.Record_3;	break;
      case Addr_WarningRecord4:	 *Value = WRN.WarningRecords[1].bit.Record_0;	break;
      case Addr_WarningRecord5:	 *Value = WRN.WarningRecords[1].bit.Record_1;	break;
      case Addr_WarningRecord6:	 *Value = WRN.WarningRecords[1].bit.Record_2;	break;
      case Addr_WarningRecord7:	 *Value = WRN.WarningRecords[1].bit.Record_3;	break;
      case Addr_WarningRecord8:	 *Value = WRN.WarningRecords[2].bit.Record_0;	break;
      case Addr_WarningRecord9:	 *Value = WRN.WarningRecords[2].bit.Record_1;	break;
      case Addr_WarningRecord10: *Value = WRN.WarningRecords[2].bit.Record_2;	break;
      case Addr_WarningRecord11: *Value = WRN.WarningRecords[2].bit.Record_3;	break;
      
      case Addr_WarningRecordRegister0: *Value = WRN.WarningRecords[0].all;	break;
      case Addr_WarningRecordRegister1: *Value = WRN.WarningRecords[1].all;	break;
      case Addr_WarningRecordRegister2: *Value = WRN.WarningRecords[2].all;	break;
      
      case Addr_CommunicationErrorCode:	        *Value = COMMERR.Source;                   break;
      case Addr_CommunicationErrorCodeRecord1:	*Value = COMMERR.Records[0].bit.Record_1;  break;
      case Addr_CommunicationErrorCodeRecord2:	*Value = COMMERR.Records[0].bit.Record_2;  break;
      case Addr_CommunicationErrorCodeRecord3:	*Value = COMMERR.Records[0].bit.Record_3;  break;
      case Addr_CommunicationErrorCodeRecord4:	*Value = COMMERR.Records[1].bit.Record_0;  break;
      case Addr_CommunicationErrorCodeRecord5:	*Value = COMMERR.Records[1].bit.Record_1;  break;
      case Addr_CommunicationErrorCodeRecord6:	*Value = COMMERR.Records[1].bit.Record_2;  break;
      case Addr_CommunicationErrorCodeRecord7:	*Value = COMMERR.Records[1].bit.Record_3;  break;
      case Addr_CommunicationErrorCodeRecord8:	*Value = COMMERR.Records[2].bit.Record_0;  break;
      case Addr_CommunicationErrorCodeRecord9:	*Value = COMMERR.Records[2].bit.Record_1;  break;
      case Addr_CommunicationErrorCodeRecord10:	*Value = COMMERR.Records[2].bit.Record_2;  break;
      case Addr_CommunicationErrorCodeRecord11:	*Value = COMMERR.Records[2].bit.Record_3;  break;
      
      case Addr_CommunicationErrorCodeRegister0:*Value = COMMERR.Records[0].all;  break;
      case Addr_CommunicationErrorCodeRegister1:*Value = COMMERR.Records[1].all;  break;
      case Addr_CommunicationErrorCodeRegister2:*Value = COMMERR.Records[2].all;  break;
      
      case Addr_DriverTemperature:	          *Value = DriverTemperature;  break;
      
    // Adresses for Parameters Direct Access
      //case Addr_StartStop:	        *Value = DrvFlags0.bit.StartStop;	                        break;
      case Addr_InterfaceModes:	        *Value = DrvFlags0.bit.InterfaceMode;	                        break;
      case Addr_MotionDirection:	*Value = DrvFlags0.bit.MotionDirection;	                        break;
      case Addr_BrakeFunctionAtAlarm:	*Value = DrvFlags0.bit.BrakeFunctionAtAlarm;	                break;
      case Addr_NoOperateAtInitAlarmEnable: *Value = DrvFlags0.bit.NoOperateAtInitAlarmEnable;	        break;
      case Addr_InitialThermalInputDetection: *Value = DrvFlags0.bit.InitialThermalInputDetection;	break;
      case Addr_OverLoadWarningEnable: *Value = DrvFlags0.bit.OverLoadWarningEnable;                    break;
      case Addr_AnalogInputSignalSelect: *Value = DrvFlags0.bit.AnalogInputSignalSelect;	        break;
      
      case Addr_UnderVoltageAlarmLevel:	*Value = DrvFlags1.bit.UnderVoltageAlarmLevel;	break;
      case Addr_OverVoltageAlarmLevel:	*Value = DrvFlags1.bit.OverVoltageAlarmLevel;	break;
      case Addr_AlarmResetActiveLevel:	*Value = DrvFlags1.bit.AlarmResetActiveLevel;	break;
      case Addr_AlarmOutActiveLevel:	*Value = DrvFlags1.bit.AlarmOutActiveLevel;	break;
      case Addr_ServoOnActiveLevel:	*Value = DrvFlags1.bit.ServoOnActiveLevel;	break;
             

      // Other parameters
      
      default:		          Err = -1;
    }
  
  return(Err);
}
/*****************************************************************************/



uint32_t CheckSumViewer;
/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t ParametersLoad(uint16_t DBType)
{
uDW Value;
uint16_t i, EndAddr, Offset;
int16_t Err = 0;
uint32_t CheckSumTmp = 0;

    // Parameters range
/*    if(DBType)  {EndAddr = Addr_Motor_CheckSum;  i = Addr_Motor_Type; Offset = MDBSelect(); }
    else*/        {EndAddr = Addr_CheckSum; i = 100; Offset = 0; }
  
    // Read Long Data from ROM    
    for(; i < EndAddr; i++){
      Err |= EE_ReadVariable((200*Offset + i*2),   &Value.w[0]);  // Read Data from Flash; returns "0" on success
      Err |= EE_ReadVariable((200*Offset + i*2+1), &Value.w[1]);  // Read Data from Flash; // returns "0" on success
      ParameterSet(i,Value.dw);	                                  // Write Data to RAM
      CheckSumTmp += Value.dw;			                  // CheckSum Calculation
    }

    Err |= EE_ReadVariable((200*Offset + i*2),   &Value.w[0]);	  // Read CheckSum from Flash; returns "0" on success
    Err |= EE_ReadVariable((200*Offset + i*2+1), &Value.w[1]);    // Read CheckSum from Flash; returns "0" on success
    ParameterSet(i,Value.dw);			                  // Write Data to RAM
    
    // CheckSum verification
    if((CheckSumTmp == 0) || (CheckSumTmp != Value.dw)){ Err |= -1;}

    if(Err){AL.Source = 12;}	// Set Alarm Source
//ParameterDefaultGGM90();
    return Err;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t ParametersSave(uint16_t DBType)
{
uDW Value1, Value2;
uint16_t i, EndAddr, Offset;
int16_t Err = 0, ErrRead = 0;
uint32_t CheckSumTmp = 0;

     // Parameters range
/*    if(DBType)  {EndAddr = Addr_Motor_CheckSum;  i = Addr_Motor_Type; Offset = ROM_MDB_Ptr; }
    else*/        {EndAddr = Addr_CheckSum; i = 100; Offset = 0; }
    
    FLASH_Unlock();                       // Unlock the Flash Program Erase controller

    for(; i < EndAddr; i++){
      Err |= ParameterGet(i, &Value1.dw);	    // Read Data from RAM; returns "0" on success
      
      CheckSumTmp += Value1.dw;			    // CheckSum Calculation
      
      // Read Data from EEPROM; returns "0" on success, "1" - variable was not found, "0xAB" - no Valide page
      ErrRead  = EE_ReadVariable((200*Offset + i*2),   &Value2.w[0]);
      ErrRead |= EE_ReadVariable((200*Offset + i*2+1), &Value2.w[1]);
      
      // Do not save the variable if read is successful and it is same
      if((Value1.dw == Value2.dw) && (ErrRead == 0))  continue;

      // Save Data to EEPROM
      if(FLASH_COMPLETE != EE_WriteVariable((200*Offset + i*2),Value1.w[0]))// returns "FLASH_COMPLETE" on success
        { Err |= 1; } 
      if(FLASH_COMPLETE != EE_WriteVariable((200*Offset + i*2+1),Value1.w[1])) // returns "FLASH_COMPLETE" on success
        { Err |= 1; } 
        
      // Read Data from EEPROM; returns "0" on success, "1" - variable was not found, "0xAB" - no Valide page
      Err |= EE_ReadVariable((200*Offset + i*2),   &Value2.w[0]);
      Err |= EE_ReadVariable((200*Offset + i*2+1), &Value2.w[1]);
      
      // Verify the written data
      if(Value1.dw != Value2.dw)  
        Err |= 12;
    }
    
    // Read CheckSum from EEPROM; returns "0" on success, "1" - variable was not found, "0xAB" - no Valide page
    ErrRead  = EE_ReadVariable((200*Offset + i*2),   &Value2.w[0]);
    ErrRead |= EE_ReadVariable((200*Offset + i*2+1), &Value2.w[1]);

    // Do not save the CheckSum if it is same
    if((Value2.dw != CheckSumTmp) || ErrRead){
      if(FLASH_COMPLETE != EE_WriteVariable((200*Offset + i*2),CheckSumTmp))       // returns "FLASH_COMPLETE" on success
        { Err |= 1; } 
      if(FLASH_COMPLETE != EE_WriteVariable((200*Offset + i*2+1),CheckSumTmp >> 16)) // returns "FLASH_COMPLETE" on success
        { Err |= 1; } 
    }

    FLASH_Lock();             // Lock the Flash Program Erase controller

    if(Err) AL.Source = 12;			// Set Alarm Source
CheckSumViewer = CheckSumTmp;
    return Err;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t DataVerify(uint16_t DBType)
{
uint16_t i, EndAddr;
int16_t Err = 0;
uint32_t Value, CheckSumTmp = 0;

     // Parameters range
    /*if(DBType)  {EndAddr = Addr_Motor_CheckSum;  i = Addr_Motor_Type;}
    else */       {EndAddr = Addr_CheckSum; i = 100;}
    
    for(; i < EndAddr; i++){
      Err |= ParameterGet(i, &Value);	          // Read Data from RAM; returns "0" on success
      CheckSumTmp += Value;			  // CheckSum Calculation
    }

    Err |= ParameterGet(i, &Value);	          // Read CheckSum from RAM; returns "0" on success

    if(Value != CheckSumTmp) Err |= -1;

    if(Err) AL.Source = 12;			// Set Alarm Source
    
    return Err;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t CheckSumCalculate(uint16_t DBType)
{
uint16_t i, EndAddr;
int16_t Err = 0;
uint32_t Value, CheckSumTmp = 0;

     // Parameters range
    /*if(DBType)  {EndAddr = Addr_Motor_CheckSum;  i = Addr_Motor_Type;}
    else */       {EndAddr = Addr_CheckSum; i = 100;}
    
    for(; i < EndAddr; i++){
      Err |= ParameterGet(i, &Value);	          // Read Data from RAM; returns "0" on success
      CheckSumTmp += Value;			  // CheckSum Calculation
    }

    CheckSum = CheckSumTmp;
    
    if(Err) AL.Source = 12;			// Set Alarm Source
    
    return Err;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void EnumToArray(void)
{
  uint16_t i;
  
  for (i = Addr_MotorNumber; i < (Addr_CheckSum+1); i++){
    VirtAddVarTab[i*2] = i*2;
    VirtAddVarTab[i*2+1] = i*2+1;
  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void AlarmRecordSave(void)
{
uDW Value;
int16_t Err = 0, i, Addr;

  FLASH_Unlock();                       // Unlock the Flash Program Erase controller
  
  Addr = Addr_AlarmRecordRegister0;
  
  for(i=0; i < 3; i++){
    Value.dw = AL.AlarmRecords[i].all;
    
    // Save Data to EEPROM
    if(FLASH_COMPLETE != EE_WriteVariable((Addr*2),Value.w[0]))// returns "FLASH_COMPLETE" on success
      { Err |= 1; }
    if(FLASH_COMPLETE != EE_WriteVariable((Addr*2+1),Value.w[1])) // returns "FLASH_COMPLETE" on success
      { Err |= 1; }
    Addr++;
  }
  
  FLASH_Lock();             // Lock the Flash Program Erase controller
  
  if(Err) AL.Source = 12;			// Set Alarm Source

  //return Err;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void AlarmRecordsLoad(void)
{
uDW Value;

int16_t i, Err = 0, Addr = Addr_AlarmRecordRegister0;
  
    // Read Long Data from ROM    
    for(i=0; i < 3; i++){
      Err |= EE_ReadVariable((Addr*2),   &Value.w[0]);  // Read Data from Flash; returns "0" on success
      Err |= EE_ReadVariable((Addr*2+1), &Value.w[1]);  // Read Data from Flash; // returns "0" on success
      Addr++;
      AL.AlarmRecords[i].all = Value.dw;                // Write Data to RAM

    }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void WarningRecordSave(void)
{
uDW Value;
int16_t Err = 0, i, Addr;

  FLASH_Unlock();                       // Unlock the Flash Program Erase controller
  
  Addr = Addr_WarningRecordRegister0;
  
  for(i=0; i < 3; i++){
    Value.dw = WRN.WarningRecords[i].all;
    
    // Save Data to EEPROM
    if(FLASH_COMPLETE != EE_WriteVariable((Addr*2),Value.w[0]))// returns "FLASH_COMPLETE" on success
      { Err |= 1; }
    if(FLASH_COMPLETE != EE_WriteVariable((Addr*2+1),Value.w[1])) // returns "FLASH_COMPLETE" on success
      { Err |= 1; }
    Addr++;
  }
  
  FLASH_Lock();             // Lock the Flash Program Erase controller
  
  if(Err) AL.Source = 12;			// Set Alarm Source

  //return Err;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void WarningRecordsLoad(void)
{
uDW Value;

int16_t i, Err = 0, Addr = Addr_WarningRecordRegister0;
  
    // Read Long Data from ROM    
    for(i=0; i < 3; i++){
      Err |= EE_ReadVariable((Addr*2),   &Value.w[0]);  // Read Data from Flash; returns "0" on success
      Err |= EE_ReadVariable((Addr*2+1), &Value.w[1]);  // Read Data from Flash; // returns "0" on success
      Addr++;
      WRN.WarningRecords[i].all = Value.dw;                // Write Data to RAM

    }
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
