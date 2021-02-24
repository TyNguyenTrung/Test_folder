/**
  ******************************************************************************
  * @file    Menu.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2015-10-05
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
#include "Menu.h"
#include "PositionVelocityControl.h"
#include "SSD1306.h"
#include "PositionVelocityControl.h"
#include "CurrentControl.h"
#include "HardwareConfiguration.h"
#include "Motor.h"
//#include "InertiaIdentification.h"
#include "SysMonitor.h"
#include "Parameters.h"
#include "Flags.h"
#include "CurrentControl.h"
#include "Velocity_Measure.h"
#include "GPIO_functions.h"
#include "REFGenFunctions.h"
#include "ADC_functions.h"
//extern uint32_t BackEMFAlarmLevel;

extern uint32_t IOInputReg;
extern int32_t OperationSpeed;
extern int32_t Gear_Conversion_Rate; // 16Q16 bit format
extern int32_t Conveyor_Pulley_Diameter;
extern int32_t ConveyorTransferSpeed;

uint8_t bPrevious_key;
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

/*****************************************************************************/


#define TimeOutRated 5000
#define TimeOutMin 150
uint16_t TimeoutInit = TimeOutRated;
int32_t Dummy32;
int32_t *DestinationPtr32 = &Dummy32;
int16_t Dummy;
int16_t *DestinationPtr = &Dummy;
int16_t RefVelocity[16]={100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
int16_t DispAccelTime[16] = {10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000};
int16_t DispDecelTime[16] = {10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000};

int16_t MonitorEn = 0, MonitorDelay;
int16_t TorqueLimitBuff, TorqueLimit[16] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
int16_t TorqueRatio = 0, Iaverage;
uint32_t IncDec = 1, Mask = 0;
int8_t DisplayLock=1;
int16_t OptionsOld = 100;
int16_t OutBuff = 0, OutputsOverride = 0;

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
u8 KEYS_Read(void)
{
static uint16_t TimeOut;

  if(TimeOut){TimeOut--; return NOKEY;}

  /* "RIGHT" key is pressed */
  if(!GPIO_ReadInputDataBit(KEY_RIGHT_PORT, KEY_RIGHT_BIT)){
    if(bPrevious_key == RIGHT){
      if(TimeoutInit > TimeOutMin) TimeoutInit-=TimeOutMin;
      TimeOut = TimeoutInit;//If a key is pressed the value remains and there will be delay
      return RIGHT/*KEY_HOLD*/;
    }
    else{
      TimeoutInit = TimeOutRated; // Set the original value
      TimeOut = TimeoutInit;//If a key is pressed the value remains and there will be delay
      bPrevious_key = RIGHT;
      return RIGHT;
    }
  }
  /* "LEFT" key is pressed */
  else if(!GPIO_ReadInputDataBit(KEY_LEFT_PORT, KEY_LEFT_BIT)){
    if(bPrevious_key == LEFT){
      if(TimeoutInit > TimeOutMin) TimeoutInit-=TimeOutMin;
      TimeOut = TimeoutInit;//If a key is pressed the value remains and there will be delay
      return LEFT/*KEY_HOLD*/;
    }
    else{
      TimeoutInit = TimeOutRated; // Set the original value
      TimeOut = TimeoutInit;//If a key is pressed the value remains and there will be delay
      bPrevious_key = LEFT;
      return LEFT;
    }
  }
  /* "SEL" key is pressed */
  else if(!GPIO_ReadInputDataBit(KEY_SELECT_PORT, KEY_SELECT_BIT)){
    if(bPrevious_key == SELECT){
      TimeOut = TimeOutRated; // Set the original value
      return KEY_HOLD;
    }
    else{
      TimeOut = TimeOutRated; // Set the original value
      bPrevious_key = SELECT;
      return SELECT;
    }
  }
   /* "UP" key is pressed */
  else if(!GPIO_ReadInputDataBit(KEY_UP_PORT, KEY_UP_BIT)){
    if(bPrevious_key == UP){
      TimeOut = TimeOutRated; // Set the original value
      return KEY_HOLD;
    }
    else{
      TimeOut = TimeOutRated; // Set the original value
      bPrevious_key = UP;
      return UP;
    }
  }
  /* "DOWN" key is pressed */
  else if(!GPIO_ReadInputDataBit(KEY_DOWN_PORT, KEY_DOWN_BIT)){
    if(bPrevious_key == DOWN){
      TimeOut = TimeOutRated; // Set the original value
      return KEY_HOLD;
    }
    else{
      TimeOut = TimeOutRated; // Set the original value
      bPrevious_key = DOWN;
      return DOWN;
    }
  }
  /* No key is pressed */
  else{
    //TimeOut = 0;// Reset the Timeout if no key si pressed
    TimeoutInit = TimeOutRated; // Set the original value
    bPrevious_key = NOKEY;
    return NOKEY;
  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void Menu(void)
{
#define DriverParameters 6
uint8_t key;
int32_t Temp, Temp2;
static int8_t Refresh;
static int16_t Level = 0, Increase = 0;
static int16_t OptionsMinInit = 0, OptionsMaxInit = DriverParameters;
static int16_t OptionsMin = 0, OptionsMax = DriverParameters;
static int32_t MonitorFilterVelocity = 0;

#define Output_0_Test 160

#define Label_AlarmReset 12
#define Label_ParamSave 13

  key = KEYS_Read();

  if((key == NOKEY) && DisplayLock) return;
  DisplayLock=0;
  
       if(key == DOWN)
         Level--;
  else if(key == UP)
         Level++;
  
       if(Level < OptionsMin) Level = OptionsMax;
  else if(Level > OptionsMax) Level = OptionsMin;
  
  if((OptionsOld != Level) || Refresh){
    OptionsOld = Level;
    Refresh = 0;
    MonitorEn = 0; // Disable displaing
    switch(Level){
      case(0):  
        _ControlParameters();
        Increase = 10;
        OptionsMinInit = 10;
        OptionsMaxInit = 14;
      break;
          
      case(1): 
        _MotionParameters();
        Increase = 20;
        OptionsMinInit = 20;
        OptionsMaxInit = 25;
      break;
          
      case(2):  
        _ExtendedFunctions();
        Increase = 30;
        OptionsMinInit = 30;
        OptionsMaxInit = 36;
      break;
      
      case(3):  
        _VelocityControl();
        Increase = 40;
        OptionsMinInit = 40;
        OptionsMaxInit = 44;
      break;
      
      case(4):  
        _CurrentControl();
        Increase = 50;
        OptionsMinInit = 50;
        OptionsMaxInit = 53;
      break;
      
      case(5):  
        _MotorParameters();
        Increase = 60;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
      break;
      
      case(6):  
        _DriverParameters();
        Increase = 100;
        OptionsMinInit = 100;
        OptionsMaxInit = 107;
      break;
      
      //----- Control Parameters -----
      case(10): 
        Increase = 10; 
        OptionsMinInit = 10;
        OptionsMaxInit = 14;
        _StartStop();      
      break;
      
      case(11): 
        Increase = 11; 
        OptionsMinInit = 10;
        OptionsMaxInit = 14;
        _InterfaceModes(); 
      break;
      
      case(Label_AlarmReset): 
        Increase = Label_AlarmReset; 
        OptionsMinInit = 10;
        OptionsMaxInit = 14;
        _AlarmReset(); 
      break;
      
      case(Label_ParamSave): 
        Increase = Label_ParamSave; 
        OptionsMinInit = 10;
        OptionsMaxInit = 14;
        _ParametersSave(); 
      break;
      
      case(14): 
        Increase = 0;  
        OptionsMinInit = 0; 
        OptionsMaxInit = DriverParameters;
        _Return();
      break;
      
      //----- Motion Parameters -----
      case(20): 
        Increase = 20; 
        OptionsMinInit = 20;
        OptionsMaxInit = 25;
        _Acceleration();        
      break;
      
      case(21): 
        Increase = 21; 
        OptionsMinInit = 20;
        OptionsMaxInit = 25;
        _Deceleration();        
      break;
      
      case(22): 
        Increase = 22; 
        OptionsMinInit = 20;
        OptionsMaxInit = 25;
        _Velocity();            
      break;
      
      case(23): 
        Increase = 23; 
        OptionsMinInit = 20;
        OptionsMaxInit = 25;
        _Direction();           
      break;
      
      case(24): 
        Increase = 23; 
        OptionsMinInit = 20;
        OptionsMaxInit = 25;
        _TorqueLimit();           
      break;
      
      case(25): 
        Increase = 0; 
        _Return();  
        OptionsMinInit = 0; 
        OptionsMaxInit = DriverParameters; 
        break;
                
      //----- Extended Functions -----
      case(30):  
        _Velocities();
        Increase = 80;
        OptionsMinInit = 80;
        OptionsMaxInit = 88;
      break;

      case(31): 
        Increase = 110;
        OptionsMinInit = 110;
        OptionsMaxInit = 118;
        _Accelerations(); 
      break;
      
      case(32): 
        Increase = 120;
        OptionsMinInit = 120;
        OptionsMaxInit = 128;
        _Decelerations(); 
      break;
      
      case(33): 
        Increase = 130;
        OptionsMinInit = 130;
        OptionsMaxInit = 138;
        _TorqueLimits(); 
      break;
      
      case(34): 
        Increase = 90; 
        OptionsMinInit = 90; 
        OptionsMaxInit = 95; 
        _Monitor();
      break;
      
      case(35): 
        Increase = 160; 
        OptionsMinInit = 160; 
        OptionsMaxInit = 163; 
        _OutputsTest();
      break;
      
      case(36): 
        Increase = 0; 
        _Return(); 
        OptionsMinInit = 0; 
        OptionsMaxInit = DriverParameters;
      break;
      
      //----- Velocity Control Parameters -----
      case(40): 
        Increase = 40; 
        OptionsMinInit = 40; 
        OptionsMaxInit = 44;
        _VelocityKp(); 
      break;
      
      case(41): 
        Increase = 41; 
        OptionsMinInit = 40; 
        OptionsMaxInit = 44;
        _VelocityKi(); 
      break;
      
      case(42): 
        Increase = 42; 
        OptionsMinInit = 40; 
        OptionsMaxInit = 44;
        _VelocityKd(); 
      break;
      
      case(43): 
        Increase = 43; 
        OptionsMinInit = 40; 
        OptionsMaxInit = 44;
        _InertiaRatio(); 
      break;
      
      case(44): 
        Increase = 0; 
        _Return(); 
        OptionsMinInit = 0; 
        OptionsMaxInit = DriverParameters;
      break;
      
      //----- Current Control Parameters -----
      case(50): 
        Increase = 50; 
        OptionsMinInit = 50; 
        OptionsMaxInit = 53;
        _LoopBandwidth(); 
      break;
      
      case(51): 
        Increase = 51; 
        OptionsMinInit = 50; 
        OptionsMaxInit = 53;
        _RefLPFBandwidth(); 
      break;
      
      case(52): 
        Increase = 52; 
        OptionsMinInit = 50; 
        OptionsMaxInit = 53;
        _CurrentLimit();
      break;
      
      case(53): 
        Increase = 0;  
        OptionsMinInit = 0; 
        OptionsMaxInit = DriverParameters;
        _Return();
      break;
      
      //----- Motor Parameters -----
      case(60):
        Increase = 60;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
        _MotorNumber();   
      break;
      
      case(61):
        Increase = 61;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
        _MotorType();
      break;
      
      case(62):  
        Increase = 62;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
        _Ra();      
      break;

      case(63):
        Increase = 63;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
        _La();       
      break;
      
      case(64):
        Increase = 64;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
        _ViscosityFriction();
      break;
        
      case(65):
        Increase = 65;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
        _RatedVoltage();
      break;
      
      case(66):
        Increase = 66;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
        _RatedCurrent();
      break;
      
      case(67): 
        Increase = 67;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
        _MaxCurrent();
      break;
      
      case(68):  
        Increase = 68;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
        _TorqueConstant();
      break;
      
      case(69):
        Increase = 69;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
        _RotorInertia();
      break;
      
      case(70):  
        _RatedVelocity();
        Increase = 70;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
      break;
      
      case(71):  
        _EncoderResolution();
        Increase = 71;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
      break;
      
      case(72):  
        _PolePairs();
        Increase = 72;
        OptionsMinInit = 60;
        OptionsMaxInit = 73;
      break;
        
      case(73):
        Increase = 0;
        OptionsMinInit = 0;
        OptionsMaxInit = DriverParameters;
        _Return();
      break;
      
       //----- Driver Parameters -----
      case(100):
        Increase = 100;
        OptionsMinInit = 100;
        OptionsMaxInit = 107;
        _UnderVoltageAlarmLevel();
      break;
      
      case(101):
        Increase = 101;
        OptionsMinInit = 100;
        OptionsMaxInit = 107;
        _OverVoltageAlarmLevel();
      break;
      
      case(102):
        Increase = 102;
        OptionsMinInit = 100;
        OptionsMaxInit = 107;
        _AlarmResetActiveLevel();
      break;
      
      case(103):
        Increase = 103;
        OptionsMinInit = 100;
        OptionsMaxInit = 107;
        _AlarmOutActiveLevel();
      break;
      
      case(104):
        Increase = 104;
        OptionsMinInit = 100;
        OptionsMaxInit = 107;
        _ServoOnActiveLevel();
      break;
      
      case(105):
        Increase = 105;
        OptionsMinInit = 100;
        OptionsMaxInit = 107;
        _BackEMFCntrlLevel();
      break;
      
      case(106):
        Increase = 106;
        OptionsMinInit = 100;
        OptionsMaxInit = 107;
        _DriverOvertempAlLevel();
      break;
      
      case(107):  
        Increase = 0;
        OptionsMinInit = 0;
        OptionsMaxInit = DriverParameters;
        _Return();
      break;
      
       //-------- Velocity Levels ---------
      case(80): 
        Increase = 80;
        OptionsMinInit = 80;
        OptionsMaxInit = 88;
        _Velocity1();
      break;
      
      case(81): 
        Increase = 81;
        OptionsMinInit = 80;
        OptionsMaxInit = 88;
        _Velocity2();
      break;
      
      case(82): 
        Increase = 82;
        OptionsMinInit = 80;
        OptionsMaxInit = 88;
        _Velocity3();
      break;

      case(83): 
        Increase = 83; 
        OptionsMinInit = 80;
        OptionsMaxInit = 88;
        _Velocity4();
      break;
      
      case(84): 
        Increase = 84; 
        OptionsMinInit = 80;
        OptionsMaxInit = 88;
        _Velocity5();
      break;
      
      case(85): 
        Increase = 85;
        OptionsMinInit = 80;
        OptionsMaxInit = 88;
        _Velocity6();
      break;
      
      case(86): 
        Increase = 86; 
        OptionsMinInit = 80;
        OptionsMaxInit = 88;
        _Velocity7();
      break;
      
      case(87): 
        Increase = 87; 
        OptionsMinInit = 80;
        OptionsMaxInit = 88;
        _Velocity8();
      break;
      
      case(88):  
        _Return();
        Increase = 30;
        OptionsMinInit = 30;
        OptionsMaxInit = 36;
      break;
      
      //-------- Monitoring Functions ---------
      case(90): 
        Increase = 90;
        OptionsMinInit = 90;
        OptionsMaxInit = 95;
        _VelocityMonitor();
        
      break;
      
      case(91): 
        Increase = 91; 
        OptionsMinInit = 90;
        OptionsMaxInit = 95;
        _ErrorsMonitor();
        
      break;
      
      case(92):  
        Increase = 92;
        OptionsMinInit = 90;
        OptionsMaxInit = 95;
        _TorqueRatio();
      break;
      
      case(93):  
        Increase = 140;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DIO_Monitor();
      break;
      
      case(94):  
        Increase = 153;
        OptionsMinInit = 153;
        OptionsMaxInit = 158;
        _AIO_Monitor();
      break;
      
      case(95):  
        Increase = 30;
        OptionsMinInit = 30;
        OptionsMaxInit = 36;
        _Return();
      break;
    
    //-------- Acceleration Levels ---------
    case(110): 
        Increase = 110;
        OptionsMinInit = 110;
        OptionsMaxInit = 118;
        _Acceleration1();
      break;
      
      case(111): 
        Increase = 111;
        OptionsMinInit = 110;
        OptionsMaxInit = 118;
        _Acceleration2();
      break;
      
      case(112): 
        Increase = 112;
        OptionsMinInit = 110;
        OptionsMaxInit = 118;
        _Acceleration3();
      break;

      case(113): 
        Increase = 113; 
        OptionsMinInit = 110;
        OptionsMaxInit = 118;
        _Acceleration4();
      break;
      
      case(114): 
        Increase = 114; 
        OptionsMinInit = 110;
        OptionsMaxInit = 118;
        _Acceleration5();
      break;
      
      case(115): 
        Increase = 115;
        OptionsMinInit = 110;
        OptionsMaxInit = 118;
        _Acceleration6();
      break;
      
      case(116): 
        Increase = 116; 
        OptionsMinInit = 110;
        OptionsMaxInit = 118;
        _Acceleration7();
      break;
      
      case(117): 
        Increase = 117; 
        OptionsMinInit = 110;
        OptionsMaxInit = 118;
        _Acceleration8();
      break;
      
      case(118):  
        _Return();
        Increase = 30;
        OptionsMinInit = 30;
        OptionsMaxInit = 36;
      break;
      
    //-------- Deceleration Levels ---------
    case(120): 
        Increase = 120;
        OptionsMinInit = 120;
        OptionsMaxInit = 128;
        _Deceleration1();
      break;
      
      case(121): 
        Increase = 121;
        OptionsMinInit = 120;
        OptionsMaxInit = 128;
        _Deceleration2();
      break;
      
      case(122): 
        Increase = 122;
        OptionsMinInit = 120;
        OptionsMaxInit = 128;
        _Deceleration3();
      break;

      case(123): 
        Increase = 123; 
        OptionsMinInit = 120;
        OptionsMaxInit = 128;
        _Deceleration4();
      break;
      
      case(124): 
        Increase = 124; 
        OptionsMinInit = 120;
        OptionsMaxInit = 128;
        _Deceleration5();
      break;
      
      case(125): 
        Increase = 125;
        OptionsMinInit = 120;
        OptionsMaxInit = 128;
        _Deceleration6();
      break;
      
      case(126): 
        Increase = 126; 
        OptionsMinInit = 120;
        OptionsMaxInit = 128;
        _Deceleration7();
      break;
      
      case(127): 
        Increase = 127; 
        OptionsMinInit = 120;
        OptionsMaxInit = 128;
        _Deceleration8();
      break;
      
      case(128):  
        _Return();
        Increase = 30;
        OptionsMinInit = 30;
        OptionsMaxInit = 36;
      break;
      
    //-------- Torque Limit Levels ---------
    case(130): 
        Increase = 130;
        OptionsMinInit = 130;
        OptionsMaxInit = 138;
        _TorqueLimit1();
      break;
      
      case(131): 
        Increase = 131;
        OptionsMinInit = 130;
        OptionsMaxInit = 138;
        _TorqueLimit2();
      break;
      
      case(132): 
        Increase = 132;
        OptionsMinInit = 130;
        OptionsMaxInit = 138;
        _TorqueLimit3();
      break;

      case(133): 
        Increase = 133; 
        OptionsMinInit = 130;
        OptionsMaxInit = 138;
        _TorqueLimit4();
      break;
      
      case(134): 
        Increase = 134; 
        OptionsMinInit = 130;
        OptionsMaxInit = 138;
        _TorqueLimit5();
      break;
      
      case(135): 
        Increase = 135;
        OptionsMinInit = 130;
        OptionsMaxInit = 138;
        _TorqueLimit6();
      break;
      
      case(136): 
        Increase = 136; 
        OptionsMinInit = 130;
        OptionsMaxInit = 138;
        _TorqueLimit7();
      break;
      
      case(137): 
        Increase = 137; 
        OptionsMinInit = 130;
        OptionsMaxInit = 138;
        _TorqueLimit8();
      break;
      
      case(138):  
        _Return();
        Increase = 30;
        OptionsMinInit = 30;
        OptionsMaxInit = 36;
      break;
      
      //-------- DIO Monitor ---------
     case(140): 
        Increase = 140;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DInput_0();
      break;
      
      case(141): 
        Increase = 141;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DInput_1();
      break;
      
      case(142): 
        Increase = 142;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DInput_2();
      break;
      
      case(143): 
        Increase = 143;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DInput_3();
      break;
      
      case(144): 
        Increase = 144;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DInput_4();
      break;
      
      case(145): 
        Increase = 145;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DInput_5();
      break;
      
      case(146): 
        Increase = 146;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DInput_6();
      break;
      
      case(147): 
        Increase = 147;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DInput_7();
      break;
      
      case(148): 
        Increase = 148;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DOutput_0();
      break;
      
      case(149): 
        Increase = 149;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DOutput_1();
      break;
      
      case(150): 
        Increase = 150;
        OptionsMinInit = 140;
        OptionsMaxInit = 151;
        _DOutput_2();
      break;
      
      case(151):  
        _Return();
        Increase = 90;
        OptionsMinInit = 90;
        OptionsMaxInit = 95;
      break;
      
      //-------- AIO Monitor ---------
     case(153): 
        Increase = 153;
        OptionsMinInit = 153;
        OptionsMaxInit = 158;
        _ExternalCommandSpeed();
      break;
      
      case(154): 
        Increase = 154;
        OptionsMinInit = 153;
        OptionsMaxInit = 158;
        _ExternalCommandVoltage();
      break;
      
     case(155): 
        Increase = 155;
        OptionsMinInit = 153;
        OptionsMaxInit = 158;
        _InternalCommandSpeed();
      break;
      
      case(156): 
        Increase = 156;
        OptionsMinInit = 153;
        OptionsMaxInit = 158;
        _InternalAccelerationTime();
      break;
      
      case(157): 
        Increase = 157;
        OptionsMinInit = 153;
        OptionsMaxInit = 158;
        _InternalDecelerationTime();
      break;
      
      case(158):  
        _Return();
        Increase = 90;
        OptionsMinInit = 90;
        OptionsMaxInit = 95;
      break;
      
      //-------- Outputs Test ---------
      case(Output_0_Test): 
        Increase = Output_0_Test;
        OptionsMinInit = Output_0_Test;
        OptionsMaxInit = Output_0_Test + 3;
        _Output_0();
      break;
      
      case(161): 
        Increase = 161;
        OptionsMinInit = Output_0_Test;
        OptionsMaxInit = Output_0_Test + 3;;
        _Output_1();
      break;
      
      case(162): 
        Increase = 162;
        OptionsMinInit = Output_0_Test;
        OptionsMaxInit = Output_0_Test + 3;;
        _Output_2();
      break;
      
      case(163):  
        _Return();
        Increase = 30;
        OptionsMinInit = 30;
        OptionsMaxInit = 36;
      break;
    }
  }
  
  //-------- Test Mode Outputs Override Enable/Disable --------
  if((Output_0_Test <= Level)&&(Level <= (Output_0_Test+3)))
    OutputsOverride = 1;
  else{
    OutputsOverride = 0;
    OutBuff = 0;        // Reset out buffer after exiting the test mode
  }
  
  //
  if(key == SELECT){
    Level = Increase;
    OptionsMin = OptionsMinInit;
    OptionsMax = OptionsMaxInit;
    
    if(Level == Label_AlarmReset){  // Alarm Reset
      AlarmInit();
      _ErrorsMonitor();
    }
    
    if(Level == Label_ParamSave){
      ParametersSave(DriverParam); // Save Driver Parameters
      _ErrorsMonitor();
    }
  }
  
  //-------------------------------------
  if(key == RIGHT){
    Temp = (*DestinationPtr32) & Mask;
    Temp += IncDec;
    Temp &= Mask;
    Temp2 = 0xFFFF ^ Mask;
    Temp2 &= (*DestinationPtr32);
    (*DestinationPtr32) = Temp2 | Temp;
    
    Temp = (*DestinationPtr) & Mask;
    Temp += IncDec;
    Temp &= Mask;
    Temp2 = 0xFFFF ^ Mask;
    Temp2 &= (*DestinationPtr);
    (*DestinationPtr) = Temp2 | Temp;
    
    //(*DestinationPtr)+=IncDec;
    //(*DestinationPtr32)+=IncDec;
    //(*DestinationPtr) ^= Mask;
    //(*DestinationPtr32) ^= Mask;
    Refresh = 1;
  }
  else if(key == LEFT){
    Temp = (*DestinationPtr32) & Mask;
    Temp -= IncDec;
    Temp &= Mask;
    Temp2 = 0xFFFF ^ Mask;
    Temp2 &= (*DestinationPtr32);
    (*DestinationPtr32) = Temp2 | Temp;
    
    Temp = (*DestinationPtr) & Mask;
    Temp -= IncDec;
    Temp &= Mask;
    Temp2 = 0xFFFF ^ Mask;
    Temp2 &= (*DestinationPtr);
    (*DestinationPtr) = Temp2 | Temp;
    
    //(*DestinationPtr)-=IncDec;
    //(*DestinationPtr32)-=IncDec;
    //(*DestinationPtr) ^= Mask;
    //(*DestinationPtr32) ^= Mask;
    Refresh = 1;
  }

  //----------- Velocity filter -----------
  Temp = FB.VelocityMonitor;
  if(Temp < 0) Temp = -Temp;
  MonitorFilterVelocity += Temp;
  MonitorFilterVelocity >>= 1;

  //----------- Operation Speed ------------
  Temp = MonitorFilterVelocity << 16;
  OperationSpeed = _IQ16div(Temp,Gear_Conversion_Rate) >> 16;
    
  //------- Conveyor Transfer Speed [m/min]
  //------ D*pi -----
  Temp = (Conveyor_Pulley_Diameter*804) >> 8;   // 16Q16*24Q8=8Q24 -> 16Q16 bit format 
  ConveyorTransferSpeed = OperationSpeed*Temp;  // 32Q0*16Q16=16Q16 bit format
  
  //----------- Torque Ratio Calculation ------------
  if(Iaverage < 0) Iaverage = -Iaverage; // Absolute value

  // Simple Filter
  TorqueRatio += ((int32_t)Iaverage*100)/MOT.Inom;
  TorqueRatio >>= 1;
      
  //--------- Velocity displaing ----------
  if(!MonitorDelay--){
    MonitorDelay = 10000;

    // Value display
    if(MonitorEn == 1){
      // Clear the line
      //ssd1306_set_location(0, 10); //x,y
      //ssd1306_write_text_scaled("                                ", 1, 1);
    
      ssd1306_set_location(0, 10); //x,y
      NumberDisplay(MonitorFilterVelocity);
      ssd1306_write_text_scaled("                            ", 1, 1);
    }
    else if(MonitorEn == 2){
      // Clear the line
      //ssd1306_set_location(0, 10); //x,y
      //ssd1306_write_text_scaled("                                ", 1, 1);

      ssd1306_set_location(0, 10); //x,y
      NumberDisplay(TorqueRatio);
      ssd1306_write_text_scaled("                            ", 1, 1);
    }
    else if(MonitorEn == _IN0){
      ssd1306_set_location(0, 10); //x,y
      if(IO_IN0)  ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _IN1){
      ssd1306_set_location(0, 10); //x,y
      if(IO_IN1)  ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _IN2){
      ssd1306_set_location(0, 10); //x,y
      if(IO_IN2)  ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _IN3){
      ssd1306_set_location(0, 10); //x,y
      if(IO_IN3)  ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _IN4){
      ssd1306_set_location(0, 10); //x,y
      if(IO_IN4)  ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _IN5){
      ssd1306_set_location(0, 10); //x,y
      if(IO_IN5)  ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _IN6){
      ssd1306_set_location(0, 10); //x,y
      if(IO_IN6)  ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _IN7){
      ssd1306_set_location(0, 10); //x,y
      if(IO_IN7)  ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _OUT0){
      ssd1306_set_location(0, 10); //x,y
      if(IO_OUT0) ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _OUT1){
      ssd1306_set_location(0, 10); //x,y
      if(IO_OUT1) ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _OUT2){
      ssd1306_set_location(0, 10); //x,y
      if(IO_OUT2) ssd1306_write_text_scaled("1                         ", 1, 1);
      else        ssd1306_write_text_scaled("0                         ", 1, 1);
    }
    else if(MonitorEn == _EXT_CMND_SPEED){
      ssd1306_set_location(0, 10); //x,y
      //NumberDisplay(Uextpot >> 1);
      ssd1306_write_text_scaled(" [RPM]               ", 1, 1);
    }
    else if(MonitorEn == _EXT_CMNM_VOLTAGE){
      ssd1306_set_location(0, 10); //x,y
      //Temp = (long)Uextpot*20485;  // 7Q9*1Q15=8Q24 bit format
      Temp >>= 15;                 // 8Q24 -> 23Q9 bit format
  
      NumberDisplay(Temp >> 9);
      ssd1306_write_text_scaled(".", 1, 1);

      for(int16_t i=0;i<2;i++){
        Temp &= 0x1FF;
        Temp *= 10;
        NumberDisplay(Temp >> 9);
      }
  
      ssd1306_write_text_scaled(" [V]               ", 1, 1);
    }
    else if(MonitorEn == _INT_CMNM_SPEED){
      ssd1306_set_location(0, 10); //x,y
      //NumberDisplay(Uintpot >> 1);
      ssd1306_write_text_scaled(" [RPM]               ", 1, 1);
    }
    else if(MonitorEn == _INT_ACCEL_TIME){
      ssd1306_set_location(0, 10); //x,y
      
      //Temp = Uacc*ADCtoT;       // 16Q08Q8=24Q8 bit format
      Temp >>= 9;               // 24Q8 -> 32Q0 /2 bit format
      if(Temp < 50) Temp = 50;  //
      NumberDisplay(Temp);
      
      ssd1306_write_text_scaled(" [mS]               ", 1, 1);
    }
    else if(MonitorEn == _INT_DECEL_TIME){
      ssd1306_set_location(0, 10); //x,y
      
      //Temp = Udec*ADCtoT;       // 16Q08Q8=24Q8 bit format
      Temp >>= 9;               // 24Q8 -> 32Q0 /2 bit format
      if(Temp < 50) Temp = 50;  //
      NumberDisplay(Temp);
      
      ssd1306_write_text_scaled(" [mS]               ", 1, 1);
    }
  }
  
  ParametersLimitation();
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void NumberDisplay(uint16_t Number) 
{
uint16_t Digit, Divisor;

  Divisor = 10000; 			// Divisor Initialization

  while(!(Number/Divisor)){  // Decreasing of the Divisor if the upper Digits are "0"
    Divisor /= 10;
    if(Divisor == 1) break;
  }

  while(Divisor){	// Repeate untill Divisor != 0
    Digit = Number/Divisor;	// Digit for Displaing
    Number -=(Digit*Divisor);	// Remaining Part of the Number

    switch(Digit){
      case 0: _0;  break;
      case 1: _1;  break;
      case 2: _2;  break;
      case 3: _3;  break;
      case 4: _4;  break;
      case 5: _5;  break;
      case 6: _6;  break;
      case 7: _7;  break;
      case 8: _8;  break;
      case 9: _9;  break;
    }

    Divisor /= 10;			// Decreasing of the Divisor
  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _ControlParameters(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("CONTROL                         ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("PARAMETERS                      ", 1, 1);
  /*if(DisplayEn & 1)
    ssd1306_write_text_scaled("ENABLED         ", 2, 1);
  else
    ssd1306_write_text_scaled("DISABLED        ", 2, 1);
  
  DestinationPtr = &DisplayEn;*/ // Set destination pointer to the velocity
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Velocities(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITIES                      ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("SET                             ", 1, 1);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Accelerations(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ACCELERATIONS                   ", 1, 1);
    
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("SET                             ", 1, 1);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Decelerations(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DECELERATIONS                   ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("SET                             ", 1, 1);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueLimits(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE LIMITS                   ", 1, 1);  
  
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("SET                             ", 1, 1);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Monitor(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("MONITOR                         ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  /*if(DisplayEn & 1)
    ssd1306_write_text_scaled("ENABLED         ", 2, 1);
  else
    ssd1306_write_text_scaled("DISABLED        ", 2, 1);
  
  DestinationPtr = &DisplayEn;*/ // Set destination pointer to the velocity
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _OutputsTest(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("OUTPUTS TEST                    ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  /*if(DisplayEn & 1)
    ssd1306_write_text_scaled("ENABLED         ", 2, 1);
  else
    ssd1306_write_text_scaled("DISABLED        ", 2, 1);
  
  DestinationPtr = &DisplayEn;*/ // Set destination pointer to the velocity
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _ExtIO(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("EXT IO                          ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  /*if(DisplayEn & 1)
    ssd1306_write_text_scaled("ENABLED         ", 2, 1);
  else
    ssd1306_write_text_scaled("DISABLED        ", 2, 1);
  
  DestinationPtr = &DisplayEn;*/ // Set destination pointer to the velocity
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _MotionParameters(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("MOTION                          ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("PARAMETERS                      ", 1, 1);
  /*if(DisplayEn & 1)
    ssd1306_write_text_scaled("ENABLED         ", 2, 1);
  else
    ssd1306_write_text_scaled("DISABLED        ", 2, 1);
  
  DestinationPtr = &DisplayEn;*/ // Set destination pointer to the velocity
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _ExtendedFunctions(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("EXTENDED                        ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("FUNCTIONS                       ", 1, 1);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _VelocityControl(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY                        ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("CONTROL                         ", 1, 1);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _CurrentControl(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("CURRENT                         ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("CONTROL                         ", 1, 1);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _MotorParameters(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("MOTOR                           ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("PARAMETERS                      ", 1, 1);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DriverParameters(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DRIVER                          ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("PARAMETERS                      ", 1, 1);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
/*void _Display(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DISPLAY         ", 2, 1);
 
  ssd1306_set_location(0, 10); //x,y
  if(DisplayEn & 1)
    ssd1306_write_text_scaled("ENABLED         ", 2, 1);
  else
    ssd1306_write_text_scaled("DISABLED        ", 2, 1);
}*/
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _StartStop(void)
{
  ssd1306_set_location(0, 1); //x,y
  ssd1306_write_text_scaled("START/STOP                 ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  if(StartStop)
    ssd1306_write_text_scaled("RUNNING                         ", 1, 1);
  else
    ssd1306_write_text_scaled("STOPPED                         ", 1, 1);
  
  DestinationPtr = &StartStop;//Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32;//DrvFlags0.all; // Set destination pointer to the dummy variable
  IncDec = 1;                        // Increasing/Decreasing Step
  Mask = 1;                          // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Acceleration(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ACCELERATION                    ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispAccelTime[0]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
  
  DestinationPtr = &DispAccelTime[0]; // Set destination pointer to the acceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Deceleration(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DECELERATION                    ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispDecelTime[0]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
  
  DestinationPtr = &DispDecelTime[0]; // Set destination pointer to the deceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Velocity(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY                        ", 1, 1);

  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(RefVelocity[0]);
  ssd1306_write_text_scaled(" [RPM]                          ", 1, 1);

  DestinationPtr = &RefVelocity[0]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                       // Increasing/Decreasing Step
  Mask = 0xFFFF;                    // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Direction(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DIRECTION                       ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  if(DrvFlags0.bit.MotionDirection & 1)
    ssd1306_write_text_scaled("CCW                           ", 1, 1);
  else
    ssd1306_write_text_scaled("CW                            ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the direction
  DestinationPtr32 = &DrvFlags0.all; // Set destination pointer to the dummy variable
  IncDec = 4;                   // Increasing/Decreasing Step
  Mask = 4;                     // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueLimit(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE LIMIT                    ", 1, 1);  

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(TorqueLimit[0]);
  ssd1306_write_text_scaled(" [%]                        ", 1, 1);

  DestinationPtr = &TorqueLimit[0]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _VelocityMonitor(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("Velocity Monitor [RPM]          ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  MonitorEn = 1; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _ErrorsMonitor(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ERRORS MONITOR                  ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else if(AL.Source == 1)
    ssd1306_write_text_scaled("OVERLOAD                        ", 1, 1);
  else if(AL.Source == 2)
    ssd1306_write_text_scaled("OVERVOLTAGE                     ", 1, 1);
  else if(AL.Source == 3)
    ssd1306_write_text_scaled("UNDERVOLTAGE                    ", 1, 1);
  else if(AL.Source == 4)
    ssd1306_write_text_scaled("OVERSPEED                       ", 1, 1);
  else if(AL.Source == 5)
    ssd1306_write_text_scaled("OVERCURRENT                     ", 1, 1);
  else if(AL.Source == 6)
    ssd1306_write_text_scaled("BRIDGE FAULT                    ", 1, 1);
  else if(AL.Source == 7)
    ssd1306_write_text_scaled("HALL FAULT                      ", 1, 1);
  else if(AL.Source == 8)
    ssd1306_write_text_scaled("TOO MANY LINKS                  ", 1, 1);
  else if(AL.Source == 12)
    ssd1306_write_text_scaled("EEPROM FAILURE                  ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueRatio(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE RATIO [%]                 ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  MonitorEn = 2; // Enable torque ration displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DIO_Monitor(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DIGITAL IO MONITOR               ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("                                 ", 1, 1);
  
  //MonitorEn = 2; // Enable torque ration displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _AIO_Monitor(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ANALOG IO MONITOR                ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("                                 ", 1, 1);
  
  //MonitorEn = 2; // Enable torque ration displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _InterfaceModes(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("INTERFACE                       ", 1, 1);
  
  // Clean the Line
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                    ", 2, 1);
  
  ssd1306_set_location(0, 10); //x,y
  if(DrvFlags0.bit.InterfaceMode == 0)
    ssd1306_write_text_scaled("DISPLAY & PR                  ", 1, 1);
  else if(DrvFlags0.bit.InterfaceMode == 1)
    ssd1306_write_text_scaled("EXT POT                       ", 1, 1);
  else if(DrvFlags0.bit.InterfaceMode == 2)
    ssd1306_write_text_scaled("INT POT                       ", 1, 1);
  else if(DrvFlags0.bit.InterfaceMode == 3)
    ssd1306_write_text_scaled("EXT IO                        ", 1, 1);
  
  DestinationPtr = &Dummy;        // Set destination pointer to the direction
  DestinationPtr32 = &DrvFlags0.all;  // Set destination pointer to the dummy variable
  IncDec = 1;                     // Increasing/Decreasing Step
  Mask = 3;                       // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _AlarmReset(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ALARM RESET                     ", 1, 1);
  // Clean the Line
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                    ", 2, 1);
  
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  
  DestinationPtr = &Dummy;     // Set destination pointer to the dummy variable
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                  // Increasing/Decreasing Step
  Mask = 0;                    // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _ParametersSave(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("PARAMETERS SAVE              ", 1, 1);

  // Clean the Line
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                    ", 2, 1);
  
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  DestinationPtr = &Dummy;      // Set destination pointer to the dummy variable
  DestinationPtr32 = &Dummy32;  // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Velocity1(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY 1                      ", 1, 1);

  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(RefVelocity[0]);
  //ssd1306_write_text_scaled("                            ", 1, 1);
  ssd1306_write_text_scaled(" [RPM]                      ", 1, 1);
  
  DestinationPtr = &RefVelocity[0]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Velocity2(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY 2                      ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(RefVelocity[1]);
  ssd1306_write_text_scaled(" [RPM]                      ", 1, 1);
  
  DestinationPtr = &RefVelocity[1]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Velocity3(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY 3                      ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(RefVelocity[2]);
  ssd1306_write_text_scaled(" [RPM]                      ", 1, 1);
  
  DestinationPtr = &RefVelocity[2]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Velocity4(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY 4                      ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(RefVelocity[3]);
  ssd1306_write_text_scaled(" [RPM]                      ", 1, 1);
  
  DestinationPtr = &RefVelocity[3]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Velocity5(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY 5                      ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(RefVelocity[4]);
  ssd1306_write_text_scaled(" [RPM]                      ", 1, 1);
  
  DestinationPtr = &RefVelocity[4]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Velocity6(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY 6                      ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(RefVelocity[5]);
  ssd1306_write_text_scaled(" [RPM]                      ", 1, 1);
  
  DestinationPtr = &RefVelocity[5]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Velocity7(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY 7                      ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(RefVelocity[6]);
  ssd1306_write_text_scaled(" [RPM]                      ", 1, 1);
  
  DestinationPtr = &RefVelocity[6]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Velocity8(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY 8                      ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(RefVelocity[7]);
  ssd1306_write_text_scaled(" [RPM]                      ", 1, 1);
  
  DestinationPtr = &RefVelocity[7]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Acceleration1(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ACCELERATION 1                  ", 1, 1);
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispAccelTime[0]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
    
  DestinationPtr = &DispAccelTime[0]; // Set destination pointer to the acceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Acceleration2(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ACCELERATION 2                  ", 1, 1);
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispAccelTime[1]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
    
  DestinationPtr = &DispAccelTime[1]; // Set destination pointer to the acceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Acceleration3(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ACCELERATION 3                  ", 1, 1);
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispAccelTime[2]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
    
  DestinationPtr = &DispAccelTime[2]; // Set destination pointer to the acceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Acceleration4(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ACCELERATION 4                  ", 1, 1);
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispAccelTime[3]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
    
  DestinationPtr = &DispAccelTime[3]; // Set destination pointer to the acceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Acceleration5(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ACCELERATION 5                  ", 1, 1);
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispAccelTime[4]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
    
  DestinationPtr = &DispAccelTime[4]; // Set destination pointer to the acceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Acceleration6(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ACCELERATION 6                  ", 1, 1);
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispAccelTime[5]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
    
  DestinationPtr = &DispAccelTime[5]; // Set destination pointer to the acceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Acceleration7(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ACCELERATION 7                  ", 1, 1);
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispAccelTime[6]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
    
  DestinationPtr = &DispAccelTime[6]; // Set destination pointer to the acceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Acceleration8(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ACCELERATION 8                  ", 1, 1);
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispAccelTime[7]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
    
  DestinationPtr = &DispAccelTime[7]; // Set destination pointer to the acceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Deceleration1(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DECELERATION 1                  ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispDecelTime[0]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);

  DestinationPtr = &DispDecelTime[0]; // Set destination pointer to the deceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Deceleration2(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DECELERATION 2                  ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispDecelTime[1]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
  
  DestinationPtr = &DispDecelTime[1]; // Set destination pointer to the deceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Deceleration3(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DECELERATION 3                  ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispDecelTime[2]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
  
  DestinationPtr = &DispDecelTime[2]; // Set destination pointer to the deceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Deceleration4(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DECELERATION 4                  ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispDecelTime[3]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
  
  DestinationPtr = &DispDecelTime[3]; // Set destination pointer to the deceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Deceleration5(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DECELERATION 5                  ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispDecelTime[4]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
  
  DestinationPtr = &DispDecelTime[4]; // Set destination pointer to the deceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Deceleration6(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DECELERATION 6                  ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispDecelTime[5]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
  
  DestinationPtr = &DispDecelTime[5]; // Set destination pointer to the deceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Deceleration7(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DECELERATION 7                  ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispDecelTime[6]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
  
  DestinationPtr = &DispDecelTime[6]; // Set destination pointer to the deceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Deceleration8(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("DECELERATION 8                  ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DispDecelTime[7]);
  ssd1306_write_text_scaled(" [mS]                           ", 1, 1);
  
  DestinationPtr = &DispDecelTime[7]; // Set destination pointer to the deceleration
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 10;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueLimit1(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE LIMIT 1                  ", 1, 1);  

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(TorqueLimit[0]);
  ssd1306_write_text_scaled(" [%]                        ", 1, 1);
  
  DestinationPtr = &TorqueLimit[0]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueLimit2(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE LIMIT 2                  ", 1, 1);  

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(TorqueLimit[1]);
  ssd1306_write_text_scaled(" [%]                        ", 1, 1);
  
  DestinationPtr = &TorqueLimit[1]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueLimit3(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE LIMIT 3                  ", 1, 1);  

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(TorqueLimit[2]);
  ssd1306_write_text_scaled(" [%]                        ", 1, 1);
  
  DestinationPtr = &TorqueLimit[2]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueLimit4(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE LIMIT 4                  ", 1, 1);  

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(TorqueLimit[3]);
  ssd1306_write_text_scaled(" [%]                        ", 1, 1);
  
  DestinationPtr = &TorqueLimit[3]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueLimit5(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE LIMIT 5                  ", 1, 1);  

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(TorqueLimit[4]);
  ssd1306_write_text_scaled(" [%]                        ", 1, 1);
  
  DestinationPtr = &TorqueLimit[4]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueLimit6(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE LIMIT 6                  ", 1, 1);  

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(TorqueLimit[5]);
  ssd1306_write_text_scaled(" [%]                        ", 1, 1);
  
  DestinationPtr = &TorqueLimit[5]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueLimit7(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE LIMIT 7                  ", 1, 1);  

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(TorqueLimit[6]);
  ssd1306_write_text_scaled(" [%]                        ", 1, 1);
  
  DestinationPtr = &TorqueLimit[6]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueLimit8(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE LIMIT 8                  ", 1, 1);  

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(TorqueLimit[7]);
  ssd1306_write_text_scaled(" [%]                        ", 1, 1);
  
  DestinationPtr = &TorqueLimit[7]; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _VelocityKp(void)
{
sDW Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY Kp                ", 1, 1);
  
  Temp.dw = VelocityFBController.Kp1; // 16Q16 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp.w[1]);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<3;i++){
    Temp.w[1] = 0; // clear the upper word of 16Q16 bit format variable
    Temp.dw *= 10;
    NumberDisplay(Temp.w[1]);
  }
  
  ssd1306_write_text_scaled("                     ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  DestinationPtr32 = &VelocityFBController.Kp1; // Set destination pointer to the velocity
  IncDec = 66;                   // Increasing/Decreasing Step
  Mask = 0xFFFFFFFF;             // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _VelocityKi(void)
{
sDW Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY Ki                ", 1, 1);
  
  Temp.dw = VelocityFBController.Ki; // 16Q16 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp.w[1]);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<4;i++){
    Temp.w[1] = 0; // clear the upper word of 16Q16 bit format variable
    Temp.dw *= 10;
    NumberDisplay(Temp.w[1]);
  }
  
  ssd1306_write_text_scaled("                     ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  DestinationPtr32 = &VelocityFBController.Ki; // Set destination pointer to the velocity
  IncDec = 7;                   // Increasing/Decreasing Step
  Mask = 0xFFFFFFFF;            // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _VelocityKd(void)
{
sDW Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("VELOCITY Kd                ", 1, 1);
  
  Temp.dw = VelocityFBController.Kd; // 16Q16 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp.w[1]);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<3;i++){
    Temp.w[1] = 0; // clear the upper word of 16Q16 bit format variable
    Temp.dw *= 10;
    NumberDisplay(Temp.w[1]);
  }
  
  ssd1306_write_text_scaled("                     ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  DestinationPtr32 = &VelocityFBController.Kd; // Set destination pointer to the velocity
  IncDec = 66;                   // Increasing/Decreasing Step
  Mask = 0xFFFFFFFF;             // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _InertiaRatio(void)
{
sDW Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("INERTIA RATIO              ", 1, 1);

  //Temp.dw = LSM.InertiaRatio; // 16Q16 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp.w[1]);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<3;i++){
    Temp.w[1] = 0; // clear the upper word of 16Q16 bit format variable
    Temp.dw *= 10;
    NumberDisplay(Temp.w[1]);
  }
  
  ssd1306_write_text_scaled("                     ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  //DestinationPtr32 = &LSM.InertiaRatio; // Set destination pointer to the velocity
  IncDec = 66;                   // Increasing/Decreasing Step
  Mask = 0xFFFFFFFF;             // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _LoopBandwidth(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("LOOP BANDWIDTH             ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(CC.Bandwidth);
  ssd1306_write_text_scaled(" [Hz]                   ", 1, 1);

  DestinationPtr = &CC.Bandwidth; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _RefLPFBandwidth(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("REF LPF BANDWIDTH          ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(RefCurrentLPF.Bandwidth);
  ssd1306_write_text_scaled(" [Hz]                   ", 1, 1);

  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  DestinationPtr32 = &RefCurrentLPF.Bandwidth; // Set destination pointer to Current LPF
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFFFFFF;            // Set/Reset bits
}
/*****************************************************************************/

      


/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _CurrentLimit(void)
{
int16_t Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("CURRENT LIMIT              ", 1, 1);

  Temp = CC.RefLimit; // 7Q9 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp >> 9);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<3;i++){
    Temp &= 0x1FF;
    Temp *= 10;
    NumberDisplay(Temp >> 9);
  }
  
  ssd1306_write_text_scaled(" [A]                 ", 1, 1);

  DestinationPtr = &CC.RefLimit; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _MotorNumber(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("MOTOR NUMBER                    ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(MOT.Number);
  ssd1306_write_text_scaled("                              ", 1, 1);
  
  DestinationPtr = &MOT.Number; // Set destination pointer to the Motor Number
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _MotorType(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("MOTOR TYPE                      ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  if(MOT.Type == 0)
    ssd1306_write_text_scaled("STEP MOTOR                      ", 1, 1);
  else if(MOT.Type == 1)
    ssd1306_write_text_scaled("BLDC MOTOR                      ", 1, 1);
  else if(MOT.Type == 2)
    ssd1306_write_text_scaled("PMSM MOTOR                      ", 1, 1);
  else //if(MOT.Type == 3)
    ssd1306_write_text_scaled("UNKNOWN                         ", 1, 1);
  
  DestinationPtr = &MOT.Type; // Set destination pointer to the Motor Type
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Ra(void)
{
int16_t Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("RESISTANCE                    ", 1, 1);

  Temp = MOT.Ra; // 8Q8 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp >> 8);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<2;i++){
    Temp &= 0xFF;
    Temp *= 10;
    NumberDisplay(Temp >> 8);
  }
  
  ssd1306_write_text_scaled(" [Ohm]               ", 1, 1);

  DestinationPtr = &MOT.Ra; // Set destination pointer to the velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 3;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/


  

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _La(void)
{
uDW Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("INDUCTANCE                    ", 1, 1);

  Temp.dw = MOT.La << 1; // 1Q15 -> 16Q16 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp.w[1]);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<5;i++){
    Temp.w[1] = 0;
    Temp.dw *= 10;
    NumberDisplay(Temp.w[1]);
  }
  
  ssd1306_write_text_scaled(" [H]                 ", 1, 1);

  DestinationPtr = &MOT.La; // Set destination pointer to the Motor Inductance
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _ViscosityFriction(void)
{
uDW Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("Viscosity Friction            ", 1, 1);
    
  //Temp.dw = LSM.ViscosityFriction; // 8Q24 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp.w[1] >> 8);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<7;i++){
    Temp.w[1] &= 0xFF;
    Temp.dw *= 10;
    NumberDisplay(Temp.w[1] >> 8);
  }
  
  ssd1306_write_text_scaled("                     ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  //DestinationPtr32 = &LSM.ViscosityFriction; // Set destination pointer to the Motor Rotor Inertia
  IncDec = 2;                   // Increasing/Decreasing Step
  Mask = 0xFFFFFFFF;            // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _RatedVoltage(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("RATED VOLTAGE                   ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(MOT.Unom);
  ssd1306_write_text_scaled(" [V]                     ", 1, 1);

  DestinationPtr = &MOT.Unom; // Set destination pointer to the Motor Rated Voltage
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _RatedCurrent(void)
{
int16_t Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("RATED CURRENT                   ", 1, 1);

  Temp = MOT.Inom; // 7Q9 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp >> 9);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<3;i++){
    Temp &= 0x1FF;
    Temp *= 10;
    NumberDisplay(Temp >> 9);
  }
  
  ssd1306_write_text_scaled(" [A]                 ", 1, 1);

  DestinationPtr = &MOT.Inom; // Set destination pointer to the Rated Current
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _MaxCurrent(void)
{
int16_t Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("MAX CURRENT                     ", 1, 1);

  Temp = MOT.Imax; // 7Q9 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp >> 9);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<3;i++){
    Temp &= 0x1FF;
    Temp *= 10;
    NumberDisplay(Temp >> 9);
  }
  
  ssd1306_write_text_scaled(" [A]                 ", 1, 1);

  DestinationPtr = &MOT.Imax; // Set destination pointer to the Max Current
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _TorqueConstant(void)
{
sDW Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("TORQUE CONSTANT       ", 1, 1);

  Temp.dw = MOT.TorqueConstant; // 16Q16 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp.w[1]);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<4;i++){
    Temp.w[1] = 0;
    Temp.dw *= 10;
    NumberDisplay(Temp.w[1]);
  }
  
  ssd1306_write_text_scaled("                     ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  DestinationPtr32 = &MOT.TorqueConstant; // Set destination pointer to the Torque Constant
  IncDec = 7;                   // Increasing/Decreasing Step
  Mask = 0xFFFFFFFF;            // Set/Reset bits
}
/*****************************************************************************/

 


/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _RotorInertia(void)
{
uDW Temp;
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ROT INERTIA [Kg*m^2]  ", 1, 1);

  Temp.dw = MOT.RotorInertia; // 8Q24 bit format
    
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(Temp.w[1] >> 8);
  ssd1306_write_text_scaled(".", 1, 1);

  for(int16_t i=0;i<7;i++){
    Temp.w[1] &= 0xFF;
    Temp.dw *= 10;
    NumberDisplay(Temp.w[1] >> 8);
  }
  
  ssd1306_write_text_scaled("                     ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  DestinationPtr32 = &MOT.RotorInertia; // Set destination pointer to the Motor Rotor Inertia
  IncDec = 2;                   // Increasing/Decreasing Step
  Mask = 0xFFFFFFFF;            // Set/Reset bits
}
/*****************************************************************************/
      



/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _RatedVelocity(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("Rated Velocity               ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(MOT.RatedVelocity);
  ssd1306_write_text_scaled(" [RPM]                        ", 1, 1);

  DestinationPtr = &MOT.RatedVelocity; // Set destination pointer to the Rated Velocity
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/
      



/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _EncoderResolution(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ENCODER RES                     ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(MOT.EncRes);
  ssd1306_write_text_scaled(" [PPR]                        ", 1, 1);

  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  DestinationPtr32 = &MOT.EncRes; // Set destination pointer to the encoder resolution
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFFFFFF;            // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _PolePairs(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("POLE PAIRS               ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(MOT.PolePairs);
  ssd1306_write_text_scaled("                     ", 1, 1);
  
  DestinationPtr = &MOT.PolePairs; // Set destination pointer to the pole pairs
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _UnderVoltageAlarmLevel(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("UV ALARM LEVEL           ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DrvFlags1.bit.UnderVoltageAlarmLevel);
  ssd1306_write_text_scaled(" [V]                     ", 1, 1);

  DestinationPtr = &Dummy; // Set destination pointer to the pole pairs
  DestinationPtr32 = &DrvFlags1.all; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0x1FF;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _OverVoltageAlarmLevel(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("OV ALARM LEVEL           ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DrvFlags1.bit.OverVoltageAlarmLevel);
  ssd1306_write_text_scaled(" [V]                     ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the pole pairs
  DestinationPtr32 = &DrvFlags1.all; // Set destination pointer to the dummy variable
  IncDec = 0x200;                 // Increasing/Decreasing Step
  Mask = 0x3FE00;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _AlarmResetActiveLevel(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ALARM RESET LEVEL        ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DrvFlags1.bit.AlarmResetActiveLevel);
  ssd1306_write_text_scaled("                         ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the pole pairs
  DestinationPtr32 = &DrvFlags1.all; // Set destination pointer to the dummy variable
  IncDec = 0x40000;               // Increasing/Decreasing Step
  Mask = 0x40000;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _AlarmOutActiveLevel(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("ALARM OUT LEVEL          ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DrvFlags1.bit.AlarmOutActiveLevel);
  ssd1306_write_text_scaled("                         ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the pole pairs
  DestinationPtr32 = &DrvFlags1.all; // Set destination pointer to the dummy variable
  IncDec = 0x80000;               // Increasing/Decreasing Step
  Mask = 0x80000;                 // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _ServoOnActiveLevel(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("SERVO ON LEVEL           ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(DrvFlags1.bit.ServoOnActiveLevel);
  ssd1306_write_text_scaled("                         ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the pole pairs
  DestinationPtr32 = &DrvFlags1.all; // Set destination pointer to the dummy variable
  IncDec = 0x200000;              // Increasing/Decreasing Step
  Mask = 0x200000;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _BackEMFCntrlLevel(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("BEMF CONTROL LEVEL       ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(HC.BackEMFCntrlLevel);
  ssd1306_write_text_scaled(" [V]                     ", 1, 1);
  
  DestinationPtr = &HC.BackEMFCntrlLevel; // Set destination pointer to 
  DestinationPtr32 = &Dummy32;  // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFF;                // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
/*void _BackEMFAlarmLevel(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("BEMF ALARM LEVEL         ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(BackEMFAlarmLevel);
  ssd1306_write_text_scaled("                     ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to
  DestinationPtr32 = &BackEMFAlarmLevel; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
}*/
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DriverOvertempAlLevel(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("OV TEMP ALARM LEVEL     ", 1, 1);

  ssd1306_set_location(0, 10); //x,y
  NumberDisplay(AL.DriverOvertempAlLevel);
  ssd1306_write_text_scaled(" [0C]                   ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  DestinationPtr32 = &AL.DriverOvertempAlLevel; // Set destination pointer to
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 0xFFFFFFFF;            // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DInput_0(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("IN0 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else*/ if(InpFunc0.bit.FuncInput_0 == 1)
    ssd1306_write_text_scaled("FWD", 1, 1);
  else if(InpFunc0.bit.FuncInput_0 == 2)
    ssd1306_write_text_scaled("REV                            ", 1, 1);
  else if(InpFunc0.bit.FuncInput_0 == 3)
    ssd1306_write_text_scaled("M0                             ", 1, 1);
  else if(InpFunc0.bit.FuncInput_0 == 4)
    ssd1306_write_text_scaled("M1                             ", 1, 1);
  else if(InpFunc0.bit.FuncInput_0 == 5)
    ssd1306_write_text_scaled("M2                             ", 1, 1);
  else if(InpFunc0.bit.FuncInput_0 == 6)
    ssd1306_write_text_scaled("STOP MODE                 ", 1, 1);
  else if(InpFunc0.bit.FuncInput_0 == 7)
    ssd1306_write_text_scaled("ALARM RESET             ", 1, 1);
  else if(InpFunc0.bit.FuncInput_0 == 8)
    ssd1306_write_text_scaled("EXT ERROR                 ", 1, 1);
  else if(InpFunc0.bit.FuncInput_0 == 9)
    ssd1306_write_text_scaled("BREAK FREE               ", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _IN0; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DInput_1(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("IN1 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else*/ if(InpFunc0.bit.FuncInput_1 == 1)
    ssd1306_write_text_scaled("FWD", 1, 1);
  else if(InpFunc0.bit.FuncInput_1 == 2)
    ssd1306_write_text_scaled("REV", 1, 1);
  else if(InpFunc0.bit.FuncInput_1 == 3)
    ssd1306_write_text_scaled("M0", 1, 1);
  else if(InpFunc0.bit.FuncInput_1 == 4)
    ssd1306_write_text_scaled("M1", 1, 1);
  else if(InpFunc0.bit.FuncInput_1 == 5)
    ssd1306_write_text_scaled("M2", 1, 1);
  else if(InpFunc0.bit.FuncInput_1 == 6)
    ssd1306_write_text_scaled("STOP MODE", 1, 1);
  else if(InpFunc0.bit.FuncInput_1 == 7)
    ssd1306_write_text_scaled("ALARM RESET", 1, 1);
  else if(InpFunc0.bit.FuncInput_1 == 8)
    ssd1306_write_text_scaled("EXT ERROR", 1, 1);
  else if(InpFunc0.bit.FuncInput_1 == 9)
    ssd1306_write_text_scaled("BREAK FREE", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _IN1; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DInput_2(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("IN2 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else*/ if(InpFunc0.bit.FuncInput_2 == 1)
    ssd1306_write_text_scaled("FWD", 1, 1);
  else if(InpFunc0.bit.FuncInput_2 == 2)
    ssd1306_write_text_scaled("REV", 1, 1);
  else if(InpFunc0.bit.FuncInput_2 == 3)
    ssd1306_write_text_scaled("M0", 1, 1);
  else if(InpFunc0.bit.FuncInput_2 == 4)
    ssd1306_write_text_scaled("M1", 1, 1);
  else if(InpFunc0.bit.FuncInput_2 == 5)
    ssd1306_write_text_scaled("M2", 1, 1);
  else if(InpFunc0.bit.FuncInput_2 == 6)
    ssd1306_write_text_scaled("STOP MODE", 1, 1);
  else if(InpFunc0.bit.FuncInput_2 == 7)
    ssd1306_write_text_scaled("ALARM RESET", 1, 1);
  else if(InpFunc0.bit.FuncInput_2 == 8)
    ssd1306_write_text_scaled("EXT ERROR", 1, 1);
  else if(InpFunc0.bit.FuncInput_2 == 9)
    ssd1306_write_text_scaled("BREAK FREE", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _IN2; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DInput_3(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("IN3 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else*/ if(InpFunc0.bit.FuncInput_3 == 1)
    ssd1306_write_text_scaled("FWD", 1, 1);
  else if(InpFunc0.bit.FuncInput_3 == 2)
    ssd1306_write_text_scaled("REV", 1, 1);
  else if(InpFunc0.bit.FuncInput_3 == 3)
    ssd1306_write_text_scaled("M0", 1, 1);
  else if(InpFunc0.bit.FuncInput_3 == 4)
    ssd1306_write_text_scaled("M1", 1, 1);
  else if(InpFunc0.bit.FuncInput_3 == 5)
    ssd1306_write_text_scaled("M2", 1, 1);
  else if(InpFunc0.bit.FuncInput_3 == 6)
    ssd1306_write_text_scaled("STOP MODE", 1, 1);
  else if(InpFunc0.bit.FuncInput_3 == 7)
    ssd1306_write_text_scaled("ALARM RESET", 1, 1);
  else if(InpFunc0.bit.FuncInput_3 == 8)
    ssd1306_write_text_scaled("EXT ERROR", 1, 1);
  else if(InpFunc0.bit.FuncInput_3 == 9)
    ssd1306_write_text_scaled("BREAK FREE", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _IN3; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DInput_4(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("IN4 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else*/ if(InpFunc1.bit.FuncInput_0 == 1)
    ssd1306_write_text_scaled("FWD", 1, 1);
  else if(InpFunc1.bit.FuncInput_0 == 2)
    ssd1306_write_text_scaled("REV", 1, 1);
  else if(InpFunc1.bit.FuncInput_0 == 3)
    ssd1306_write_text_scaled("M0", 1, 1);
  else if(InpFunc1.bit.FuncInput_0 == 4)
    ssd1306_write_text_scaled("M1", 1, 1);
  else if(InpFunc1.bit.FuncInput_0 == 5)
    ssd1306_write_text_scaled("M2", 1, 1);
  else if(InpFunc1.bit.FuncInput_0 == 6)
    ssd1306_write_text_scaled("STOP MODE", 1, 1);
  else if(InpFunc1.bit.FuncInput_0 == 7)
    ssd1306_write_text_scaled("ALARM RESET", 1, 1);
  else if(InpFunc1.bit.FuncInput_0 == 8)
    ssd1306_write_text_scaled("EXT ERROR", 1, 1);
  else if(InpFunc1.bit.FuncInput_0 == 9)
    ssd1306_write_text_scaled("BREAK FREE", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _IN4; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DInput_5(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("IN5 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else*/ if(InpFunc1.bit.FuncInput_1 == 1)
    ssd1306_write_text_scaled("FWD", 1, 1);
  else if(InpFunc1.bit.FuncInput_1 == 2)
    ssd1306_write_text_scaled("REV", 1, 1);
  else if(InpFunc1.bit.FuncInput_1 == 3)
    ssd1306_write_text_scaled("M0", 1, 1);
  else if(InpFunc1.bit.FuncInput_1 == 4)
    ssd1306_write_text_scaled("M1", 1, 1);
  else if(InpFunc1.bit.FuncInput_1 == 5)
    ssd1306_write_text_scaled("M2", 1, 1);
  else if(InpFunc1.bit.FuncInput_1 == 6)
    ssd1306_write_text_scaled("STOP MODE", 1, 1);
  else if(InpFunc1.bit.FuncInput_1 == 7)
    ssd1306_write_text_scaled("ALARM RESET", 1, 1);
  else if(InpFunc1.bit.FuncInput_1 == 8)
    ssd1306_write_text_scaled("EXT ERROR", 1, 1);
  else if(InpFunc1.bit.FuncInput_1 == 9)
    ssd1306_write_text_scaled("BREAK FREE", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _IN5; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DInput_6(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("IN6 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else*/ if(InpFunc1.bit.FuncInput_2 == 1)
    ssd1306_write_text_scaled("FWD", 1, 1);
  else if(InpFunc1.bit.FuncInput_2 == 2)
    ssd1306_write_text_scaled("REV", 1, 1);
  else if(InpFunc1.bit.FuncInput_2 == 3)
    ssd1306_write_text_scaled("M0", 1, 1);
  else if(InpFunc1.bit.FuncInput_2 == 4)
    ssd1306_write_text_scaled("M1", 1, 1);
  else if(InpFunc1.bit.FuncInput_2 == 5)
    ssd1306_write_text_scaled("M2", 1, 1);
  else if(InpFunc1.bit.FuncInput_2 == 6)
    ssd1306_write_text_scaled("STOP MODE", 1, 1);
  else if(InpFunc1.bit.FuncInput_2 == 7)
    ssd1306_write_text_scaled("ALARM RESET", 1, 1);
  else if(InpFunc1.bit.FuncInput_2 == 8)
    ssd1306_write_text_scaled("EXT ERROR", 1, 1);
  else if(InpFunc1.bit.FuncInput_2 == 9)
    ssd1306_write_text_scaled("BREAK FREE", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _IN6; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DInput_7(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("IN7 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                       ", 1, 1);
  else*/ if(InpFunc1.bit.FuncInput_3 == 1)
    ssd1306_write_text_scaled("FWD", 1, 1);
  else if(InpFunc1.bit.FuncInput_3 == 2) 
    ssd1306_write_text_scaled("REV", 1, 1);
  else if(InpFunc1.bit.FuncInput_3 == 3)
    ssd1306_write_text_scaled("M0", 1, 1);
  else if(InpFunc1.bit.FuncInput_3 == 4)
    ssd1306_write_text_scaled("M1", 1, 1);
  else if(InpFunc1.bit.FuncInput_3 == 5)
    ssd1306_write_text_scaled("M2", 1, 1);
  else if(InpFunc1.bit.FuncInput_3 == 6)
    ssd1306_write_text_scaled("STOP MODE", 1, 1);
  else if(InpFunc1.bit.FuncInput_3 == 7)
    ssd1306_write_text_scaled("ALARM RESET", 1, 1);
  else if(InpFunc1.bit.FuncInput_3 == 8)
    ssd1306_write_text_scaled("EXT ERROR", 1, 1);
  else if(InpFunc1.bit.FuncInput_3 == 9)
    ssd1306_write_text_scaled("BREAK FREE", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _IN7; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DOutput_0(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("OUT0 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else*/ if(OutFunc.bit.FuncOutput_0 == 1)
    ssd1306_write_text_scaled("ALARM", 1, 1);
  else if(OutFunc.bit.FuncOutput_0 == 2)
    ssd1306_write_text_scaled("Servo On/Off", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _OUT0; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DOutput_1(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("OUT1 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else*/ if(OutFunc.bit.FuncOutput_1 == 1)
    ssd1306_write_text_scaled("ALARM", 1, 1);
  else if(OutFunc.bit.FuncOutput_1 == 2)
    ssd1306_write_text_scaled("SERVO ON/OFF", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _OUT1; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _DOutput_2(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("OUT2 - ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  //ssd1306_write_text_scaled("                                ", 1, 1);
  
  //ssd1306_set_location(0, 10); //x,y
  /*if(AL.Source == 0)
    ssd1306_write_text_scaled("NO ERROR                        ", 1, 1);
  else*/ if(OutFunc.bit.FuncOutput_2 == 1)
    ssd1306_write_text_scaled("ALARM", 1, 1);
  else if(OutFunc.bit.FuncOutput_2 == 2)
    ssd1306_write_text_scaled("Servo On/Off", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _OUT2; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _ExternalCommandSpeed(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("EXT CMND SPEED", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _EXT_CMND_SPEED; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _ExternalCommandVoltage(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("EXT CMND VOLTAGE", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _EXT_CMNM_VOLTAGE; // Enable displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _InternalCommandSpeed(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("INT CMND SPEED", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _INT_CMNM_SPEED; // Enable velocity displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _InternalAccelerationTime(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("INT ACCEL TIME", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _INT_ACCEL_TIME; // Enable displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _InternalDecelerationTime(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("INT DECEL TIME", 1, 1);
  
  DestinationPtr = &Dummy; // Set destination pointer to the Display
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 0;                   // Increasing/Decreasing Step
  Mask = 0;                     // Set/Reset bits
  
  MonitorEn = _INT_DECEL_TIME; // Enable displaing
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Output_0(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("OUTPUT 0                        ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  if(OutBuff & 1){
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
    ssd1306_write_text_scaled("1                            ", 1, 1);
  }
  else{
    GPIO_ResetBits(GPIOD, GPIO_Pin_2);
    ssd1306_write_text_scaled("0                            ", 1, 1);
  }
  DestinationPtr = &OutBuff; // Set destination pointer to the direction
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 1;                   // Increasing/Decreasing Step
  Mask = 1;                     // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Output_1(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("OUTPUT 1                        ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  if(OutBuff & 2){
    GPIO_SetBits(GPIOD, GPIO_Pin_4);
    ssd1306_write_text_scaled("1                            ", 1, 1);
  }
  else{
    GPIO_ResetBits(GPIOD, GPIO_Pin_4);
    ssd1306_write_text_scaled("0                            ", 1, 1);
  }
  DestinationPtr = &OutBuff; // Set destination pointer to the direction
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
  IncDec = 2;                   // Increasing/Decreasing Step
  Mask = 2;                     // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Output_2(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("OUTPUT 2                        ", 1, 1);
 
  ssd1306_set_location(0, 10); //x,y
  if(OutBuff & 4){
    GPIO_SetBits(GPIOD, GPIO_Pin_11);
    ssd1306_write_text_scaled("1                            ", 1, 1);
  }
  else{
    GPIO_ResetBits(GPIOD, GPIO_Pin_11);
    ssd1306_write_text_scaled("0                            ", 1, 1);
  }
  DestinationPtr = &OutBuff;    // Set destination pointer to the direction
  DestinationPtr32 = &Dummy32;  // Set destination pointer to the dummy variable
  IncDec = 4;                   // Increasing/Decreasing Step
  Mask = 4;                     // Set/Reset bits
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void _Return(void)
{
  ssd1306_set_location(0, 0); //x,y
  ssd1306_write_text_scaled("RETURN                          ", 1, 1);
  
  ssd1306_set_location(0, 10); //x,y
  ssd1306_write_text_scaled("                                ", 1, 1);
  
  // Dummy set
  DestinationPtr = &Dummy; // Set destination pointer to the dummy variable
  DestinationPtr32 = &Dummy32; // Set destination pointer to the dummy variable
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void ParametersLimitation(void)
{
  DrvFlags0.bit.InterfaceMode &= 3; // Mask from 0 to 3
  
  DrvFlags0.bit.MotionDirection &= 1; // Range from 0 to 1
  
  //DisplayStart &= 1; // Range from 0 to 1
    
  for(int16_t i=0; i<=7; i++){
    if(RefVelocity[i] > 5000)    RefVelocity[i] = 5000;
    else if(RefVelocity[i] < 50) RefVelocity[i] = 50;
  //}
  
  //for(int16_t i=0; i<=7; i++){
    if(DispDecelTime[i] > 15000)   DispDecelTime[i] = 15000;
    else if(DispDecelTime[i] < 50) DispDecelTime[i] = 50;
  //}
  
  //for(int16_t i=0; i<=7; i++){
    if(DispAccelTime[i] > 15000)   DispAccelTime[i] = 15000;
    else if(DispAccelTime[i] < 50) DispAccelTime[i] = 50;
  //}
  
  //for(int16_t i=0; i<=7; i++){
    if(TorqueLimit[i] > 300)    TorqueLimit[i] = 300;
    else if(TorqueLimit[i] < 0) TorqueLimit[i] = 0;
  }
  
  if(VelocityFBController.Kp1 > 65536000) VelocityFBController.Kp1 = 65536000;
  else if(VelocityFBController.Kp1 < 0)   VelocityFBController.Kp1 = 0;

  if(VelocityFBController.Ki > 65536000) VelocityFBController.Ki = 65536000;
  else if(VelocityFBController.Ki < 0)   VelocityFBController.Ki = 0;

  //if(LSM.InertiaRatio > 1310720)  LSM.InertiaRatio = 1310720;
  //else if(LSM.InertiaRatio < 0)   LSM.InertiaRatio = 0;
  
  if(CC.RefLimit > HC.Imax) CC.RefLimit = HC.Imax;
  else if(CC.RefLimit < 0)  CC.RefLimit = 0;
  
  if(CC.Bandwidth > 5000)    CC.Bandwidth = 5000;
  else if(CC.Bandwidth < 0)  CC.Bandwidth = 0;
  
  if(RefCurrentLPF.Bandwidth < CC.Bandwidth) RefCurrentLPF.Bandwidth = CC.Bandwidth;
  else if(RefCurrentLPF.Bandwidth > 9999)    RefCurrentLPF.Bandwidth = 9999;

  if(MOT.Type > 2)      MOT.Type = 2;
  else if(MOT.Type < 0) MOT.Type = 0;
  
  if(MOT.Ra > 32765)  MOT.Ra = 32765;
  else if(MOT.Ra < 0) MOT.Ra = 0;
  
  if(MOT.La > 32765)  MOT.La = 32765;
  else if(MOT.La < 0) MOT.La = 0;
  
  if(MOT.Unom > 500)    MOT.Unom = 500;
  else if(MOT.Unom < 0) MOT.Unom = 0;
  
  if(MOT.Inom > 32765)  MOT.Inom = 32765;
  else if(MOT.Inom < 0) MOT.Inom = 0;
  
  if(MOT.Imax > 32765)  MOT.Imax = 32765;
  else if(MOT.Imax < 0) MOT.Imax = 0;
  
  
  if(MOT.TorqueConstant > 2147352576) MOT.TorqueConstant = 2147352576;
  else if(MOT.TorqueConstant < 0)     MOT.TorqueConstant = 0;  
  
  if(MOT.RotorInertia > 2147352576) MOT.RotorInertia = 2147352576;
  else if(MOT.RotorInertia < 0)     MOT.RotorInertia = 0;
  
  if(MOT.RatedVelocity > 10000)  MOT.RatedVelocity = 10000;
  else if(MOT.RatedVelocity < 0) MOT.RatedVelocity = 0;
  
  if(MOT.EncRes > 2147352576)  MOT.EncRes = 2147352576;
  else if(MOT.EncRes < 0)      MOT.EncRes = 0;

  if(MOT.PolePairs > 100)    MOT.PolePairs = 100;
  else if(MOT.PolePairs < 0) MOT.PolePairs = 0;
  
  if(DrvFlags1.bit.UnderVoltageAlarmLevel > 510)     DrvFlags1.bit.UnderVoltageAlarmLevel = 510;
  else if(DrvFlags1.bit.UnderVoltageAlarmLevel < 50) DrvFlags1.bit.UnderVoltageAlarmLevel = 50;
  
  if(DrvFlags1.bit.OverVoltageAlarmLevel > 510)     DrvFlags1.bit.OverVoltageAlarmLevel = 510;
  else if(DrvFlags1.bit.OverVoltageAlarmLevel < 50) DrvFlags1.bit.OverVoltageAlarmLevel = 50;
  
  if(HC.BackEMFCntrlLevel > 500)    HC.BackEMFCntrlLevel = 500;
  else if(HC.BackEMFCntrlLevel < 50) HC.BackEMFCntrlLevel = 50;
  
  //if(BackEMFAlarmLevel > 500)     BackEMFAlarmLevel = 500;
  //else if(BackEMFAlarmLevel < 50) BackEMFAlarmLevel = 50;

  if(AL.DriverOvertempAlLevel > 100)     AL.DriverOvertempAlLevel = 100;
  else if(AL.DriverOvertempAlLevel < 40) AL.DriverOvertempAlLevel = 40;
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
