/**
  ******************************************************************************
  * @file    REFGenFunctions.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2015-06-24
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
#include "REFGenFunctions.h"
#include "ADC_functions.h"
#include "PositionVelocityControl.h"
#include "Flags.h"
#include "CurrentControl.h"
#include "GPIO_functions.h"
#include "Parameters.h"
#include "CommInterface.h"
#include "SysMonitor.h"
#include "delay.h"

struct ref_lpf RLPF1;//, RLPF2;
struct ExtPot EP;

// Speed calculations when nuber of microsteps and maximum speed is defined from RS232 command
//int32_t Distance;		// command distance (from RS232)
int32_t Speed_cmd[4]; 		// command maximum speed (from RS232)
int32_t Step_cmd[4];		// microsteps command (from RS232)
int32_t Speed_cmd_rem;		// step reminder in value of speed reminder
uint32_t Ns[4]; 		// count of times setting of current speed = S_cmd

int32_t Dist_Buff;		// command distance (from RS232)
int32_t Speed_Buff; 		// command maximum speed (from RS232)

uint16_t command_got;		//"1" when command is red
uint16_t REF_Stop; 		// flag=1 if motos stop motion
uint16_t speed_loop_cnt; 	// counter for periodically calling speed_loop() function
int32_t fi_count; 		// angle counting in external value
//int REF_Pos_Rel;
int32_t REF_Pos_Abs;
uint16_t i;


uint32_t dVacc, dVdec;
uint32_t PotAcc, PotDec;
//int16_t InterfaceMode = 0; // Potentiometers are enabled by default
int16_t Servo_En = 0;
int32_t RefVel = 0;
int16_t dVdir;

uint16_t LevelPtr;
uint16_t ParSet;
uint32_t _Dwell;

extern uint16_t DispAccelTime[], DispDecelTime[], DispDir, DisplayStart;
extern int16_t RefVelocity[];
extern int16_t TorqueLimitBuff, TorqueLimit[];
extern GPIO_TypeDef* ServoOn_Port;
extern uint16_t ServoOn_Pin;

extern uint16_t ROM_MDB_Ptr;
extern int16_t ExtVar[];
int8_t OutputsOverride;

extern uint32_t status_word;

extern uint32_t IOInputReg, IOInputRegMask, IOOutputReg;

//extern GPIO_TypeDef* CW_Port;
extern uint16_t      CW_Pin;

//extern GPIO_TypeDef* CCW_Port;
extern uint16_t      CCW_Pin;

//extern GPIO_TypeDef* M0_Port;
extern uint16_t      M0_Pin;

//extern GPIO_TypeDef* M1_Port;
extern uint16_t      M1_Pin;

//extern GPIO_TypeDef* M2_Port;
extern uint16_t      M2_Pin;

//extern GPIO_TypeDef*   SM_Port;
extern uint16_t        SM_Pin;


extern int32_t OpData[16][9];
uint16_t M03InState = 0, StartInNew = 0, ReadyOut = 1, MoveOut = 0, EndOut = 1;
uint16_t SStartNew = 0;
uint32_t DwellTimer;
#define MS0 1
#define MS1 2
#define MS2 4
#define MS3 8
#define MS4 16
#define MS5 32
int32_t MS0OpDataPtr = 0, MS1OpDataPtr = 1, MS2OpDataPtr = 2;
int32_t MS3OpDataPtr = 3, MS4OpDataPtr = 4, MS5OpDataPtr = 5;
uint16_t MS05;
/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
void speed_cmd(void)  // speed acceleration/ decceleration calculation function
{
    if(++speed_loop_cnt>=Kdiv){ // start each Kn time - 200 Hz
        speed_loop_cnt=0;

        if((!command_got) && REF_Stop){ // enter data and calculate time(Ns) and remaining Speed 
            speed_loop1_change();
            //speed_loop2_change();
            
            // Entering of new values
            //Step_cmd[1] = Distance;
            //Distance = 0;// Clear the Position Reference After Reading it
            
            //-----------------------------------------------------------
            Speed_cmd_rem = 0;
            for(i=0; i<4; i++){
              if (Speed_cmd[i] < 0) Speed_cmd[i] = -Speed_cmd[i]; // get absolut value if the user entered negative command speed 
              
              // Calculations of main speed and reminder speed
              if (Step_cmd[i] < 1) Speed_cmd[i] = -Speed_cmd[i];  // Negate the Command Speed if Path is Negative
              
              Step_cmd[i] += Speed_cmd_rem;
              
              Ns[i] = (uint32_t)(Step_cmd[i] / (Speed_cmd[i]/f0f));
              Speed_cmd_rem = Step_cmd[i] - ((Speed_cmd[i]/f0f) * Ns[i]);
            }
            command_got = 1; 		// set bit when command is got
        }

        if(Ns[0] > 0){
            RLPF1.S_input = (Speed_cmd[0]/f0f);	 // set a command speed as current speed Ns[0] times
            Ns[0]--;
        }
        else if(Ns[1] > 0){
            RLPF1.S_input = (Speed_cmd[1]/f0f);	 // set a command speed as current speed Ns[1] times
            Ns[1]--;
        }
        else if(Ns[2] > 0){
            RLPF1.S_input = (Speed_cmd[2]/f0f);	 // set a command speed as current speed Ns[2] times 
            Ns[2]--;
        }
        else if(Ns[3] > 0){
            RLPF1.S_input = (Speed_cmd[3]/f0f);	 // set a command speed as current speed Ns[3] times 
            Ns[3]--;
        }
        else{
            RLPF1.S_input = Speed_cmd_rem; // set reminder speed when Ns[1..3]=0  (just once)
            Speed_cmd_rem = 0;	
        }	
        
        speed_loop1(); // acc/dec speed (LPF for input speed)
        //speed_loop2(); // acc/dec speed (LPF for input speed)

        // report if motion is finished								
        if(RLPF1.Zero_speed && (Ns[0]==0) && (Ns[1]==0) && (Ns[2]==0) && (Ns[3]==0) && (Speed_cmd_rem==0)){
            REF_Stop = 1; 		// motion finished report bit
        }
        else REF_Stop = 0;
    }	

    Ref_Counter();      // count the command angle; Sampling frequency f0 Hz
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
inline void speed_loop1_change(void) // change of acc/dec time (from RS232 command string, reg[8])
{
    if(RLPF1.AccDec_time > AccDec_Time1_Max) RLPF1.AccDec_time = AccDec_Time1_Max;
    // variable nulling
    for(i=0; i<RLPF1.AccDec_time; i++) RLPF1.S_inp[i] = 0;
    RLPF1.S_sum=0;
    RLPF1.S_inp_index=0;
    RLPF1.Reminder_sum=0;
    RLPF1.Speed=0;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
inline void speed_loop1(void) // 1st order LPF for acc/dec algorithm
{	
    RLPF1.S_sum = RLPF1.S_sum + RLPF1.S_input - RLPF1.S_inp[RLPF1.S_inp_index];

    RLPF1.S_inp[RLPF1.S_inp_index] = RLPF1.S_input;
    if(++RLPF1.S_inp_index >= RLPF1.AccDec_time) RLPF1.S_inp_index = 0;

    RLPF1.Speed = RLPF1.S_sum / RLPF1.AccDec_time;

    RLPF1.Reminder_sum += (RLPF1.S_sum - RLPF1.Speed*RLPF1.AccDec_time); // Remainder Path

    // For positive reminder sum
    if(RLPF1.Reminder_sum >= (int32_t)RLPF1.AccDec_time){
            RLPF1.Speed++;
            RLPF1.Reminder_sum-=(int32_t)RLPF1.AccDec_time;
    }

    // For negative reminder sum
    if(RLPF1.Reminder_sum <= (-(int32_t)RLPF1.AccDec_time)){
            RLPF1.Speed--;
            RLPF1.Reminder_sum+=(int32_t)RLPF1.AccDec_time;
    }

    // Checking - if there is a non-zero speed, and setting flag
    if((RLPF1.Speed==0) && (RLPF1.Reminder_sum==0)) RLPF1.Zero_speed = 1;
    else RLPF1.Zero_speed = 0;

    //RLPF2.S_input = RLPF1.Speed;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
/*
inline void speed_loop2_change(void) // change of acc/dec time (from RS232 command string, reg[8])
{
    if(RLPF2.AccDec_time > AccDec_Time2_Max) RLPF2.AccDec_time = AccDec_Time2_Max;
    // variable nulling
    for(i=0; i<RLPF2.AccDec_time; i++) RLPF2.S_inp[i] = 0;
    RLPF2.S_sum[0]=0;
    RLPF2.S_sum[1]=0;
    RLPF2.S_inp_index=0;
    RLPF2.Reminder_sum=0;
    RLPF2.Speed=0;
}*/
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
/*inline void speed_loop2(void) // 2st order LPF for acc/dec algorithm
{
    RLPF2.S_sum[0] = RLPF2.S_sum[1] + RLPF2.S_input - RLPF2.S_inp[RLPF2.S_inp_index];
    RLPF2.S_sum[1] = RLPF2.S_sum[0];

    RLPF2.S_inp[RLPF2.S_inp_index] = RLPF2.S_input;
    if(++RLPF2.S_inp_index >= RLPF2.AccDec_time) RLPF2.S_inp_index = 0;

    RLPF2.Speed = RLPF2.S_sum[0] / RLPF2.AccDec_time;

    RLPF2.Reminder_sum += (RLPF2.S_sum[0] - RLPF2.Speed*RLPF2.AccDec_time); // Remainder Path

    // For positive reminder sum
    if(RLPF2.Reminder_sum >= (long)RLPF2.AccDec_time){
            RLPF2.Speed++;
            RLPF2.Reminder_sum-=(long)RLPF2.AccDec_time;
    }

    // For negative reminder sum
    if(RLPF2.Reminder_sum <= (-(long)RLPF2.AccDec_time)){
            RLPF2.Speed--;
            RLPF2.Reminder_sum+=(long)RLPF2.AccDec_time;
    }

    // Checking - if there is a non-zero speed, and setting flag
    if((RLPF2.Speed==0) && (RLPF2.Reminder_sum==0)) RLPF2.Zero_speed = 1;
    else RLPF2.Zero_speed = 0;
}*/
/*****************************************************************************/




/*****************************************************************************
  * @brief  This routine counts the command angle
  * @param  None
  * @retval None
*****************************************************************************/
void Ref_Counter(void) 
{
//sQW Temp;
  // Distance is an integral of the speed (the LPF2 output)
	fi_count+=RLPF1.Speed;
	
  // Calculate the REAL Distance by dividing of Kdiv
	//REF_Pos_Abs = fi_count/Kdiv;
        REF_Pos_Abs = _IQ1div(fi_count,Kdiv) >> 1; // Output 31Q1 -> 32Q0 bit format
        
        /*Temp.qw = (int64_t)fi_count*429496730;// 32Q0*0Q32 = 32Q32 bit format
        
        // Rounding
        if(Temp.w[3] & 0x8000) Temp.qw -= 2147483648;
        else                   Temp.qw += 2147483648;
          
        REF_Pos_Abs = Temp.dw[1];*/             // 32Q0 bit format
//REF_Pos_Abs *= 26;//<<<<<<<<<<<<<<<<<<<<<<<<<
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  This routine initializes of variables of IO-modul
  * @param  None
  * @retval None
*****************************************************************************/
void REF_Gen_Init(void) 
{
uint16_t i;
	
    Dist_Buff = 0;		// command distance (from RS232)
    Speed_Buff = 0; 		// command maximum speed[pps] (from RS232)
    command_got = 1;		// "1" when command is red	
    
    //Distance = 0;		// Command Distance
    
    for(i=0; i<=3; i++){
      Speed_cmd[i] = 0; 	// command maximum speed[pps] (from RS232)
      Step_cmd[i] = 0;		// microsteps command (from RS232)
      Ns[i] = 0; 		// count of times setting of current speed = S_cmd
    }

    Speed_cmd_rem = 0;		// step reminder in value of speed reminder
    speed_loop_cnt=0; 		// counter for periodically calling speed_loop() function  
    fi_count = 0; 		// angle counting in external value
    REF_Stop = 1; 		// flag=1 if motos stop motion
    //REF_Pos_Rel = 0;
    REF_Pos_Abs = 0;

    RLPF1.AccDec_time = AccDec_Time1_Max;	// current acc/dec time
    RLPF1.S_sum=0; 				// it`s S(k)
    RLPF1.S_inp_index=0; 			// indexing in s_inp[] array
    RLPF1.Reminder_sum=0;			// reminder sum
    RLPF1.Speed = 0; 				// integrated speed 
    RLPF1.S_input=0;				// current speed
    RLPF1.Zero_speed=0; 			// flag=0 if there is unrotating speed 

    // ---
    for(i=0; i<AccDec_Time1_Max; i++) RLPF1.S_inp[i] = 0;
    RLPF1.S_sum=0;
    RLPF1.S_inp_index=0;
    RLPF1.Reminder_sum=0;
    RLPF1.Speed=0;
             
    /*RLPF2.AccDec_time = AccDec_Time2_Max;	// current acc/dec time
    RLPF2.S_sum[0]=0; 						// it`s S(k)
    RLPF2.S_sum[1]=0; 						// it`s S(k-1)
    RLPF2.S_inp_index=0; 					// indexing in s_inp[] array
    RLPF2.Reminder_sum=0;					// reminder sum
    RLPF2.Speed = 0; 						// integrated speed 
    RLPF2.S_input=0;						// current speed
    RLPF2.Zero_speed=0; 					// flag=0 if there is unrotating speed		

    // ---
    for(i=0; i<AccDec_Time2_Max; i++) RLPF2.S_inp[i] = 0;
    RLPF2.S_sum[0]=0;
    RLPF2.S_sum[1]=0;
    RLPF2.S_inp_index=0;
    RLPF2.Reminder_sum=0;
    RLPF2.Speed=0;*/
    
    // External Potentiometer Init
    EP.Sum = 0;
    EP.ptr = EP.mass;
    for(i=0;i<Mas4SizeMax;i++){EP.mass[i]=0;}
    
    EP.MasSize = Mas4SizeMax;
    EP.MasShift = 0;
    for(i = EP.MasSize; i > 1; i >>= 1){EP.MasShift++;}
    
    //--------------------------------------------
    //RefVel = 0;
    //dVdir = 0;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
void REF_Command_Set(void) 
{
  
  if(Dist_Buff && REF_Stop && command_got && (!DwellTimer)){
    if(_Dwell){DwellTimer = _Dwell; _Dwell = 0; return;}
    //Distance = Dist_Buff;  	

    //RLPF1.AccDec_time = OpData[OpDataPtr][OpDataAcceleration];

    for(int16_t i=1;i<=3;i++){ Step_cmd[i] = 0; Speed_cmd[i] = 0;} // Clear buffers 1,2 and 3!!!

    Speed_cmd[0] = Speed_Buff;
    Step_cmd[0] = Dist_Buff;

    Dist_Buff = 0;
    command_got = 0;		//Command Transfered
    ParSet = 0;
  }
}
/*****************************************************************************/


extern int16_t TestAccDec_time;
extern int32_t TestDistance, TestSpeed_cmd;
extern int32_t VelocityKpCorrection, VelocityKiCorrection;
extern int32_t TCInteg, PTCInteg, PTCInteg1, PTCInteg2;
/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
void PSO_REF_Command_Set(void) 
{
  // Wait for the previose motion end.
  while(/*Dist_Buff ||*/ (!REF_Stop) || (!command_got) || DwellTimer){}
  
  //VelocityKpCorrection = (int32_t)(*ptr);
  //VelocityKiCorrection = (int32_t)(*(ptr+1));
    
  // Velocity Kp & Ki, Position Kp & Kd update
  PositionVelocityControlMultipleInit(); 
    
  //ExtVar[2] = VelocityKpCorrection >> 4;//<<<<<<<<<<<<<<<<<<<<<<<<<<<
  //ExtVar[3] = VelocityKiCorrection;//<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
  TCInteg = 0;
  PTCInteg = 0;
  PTCInteg1 = 0;
  PTCInteg2 = 0;
  
  RLPF1.AccDec_time = TestAccDec_time;
  Speed_Buff = TestSpeed_cmd;
  TestDistance *= -1;
  Dist_Buff = TestDistance;
  _Dwell = 200*20;//200 mS pause time

  for(int16_t i=1;i<=3;i++){ Step_cmd[i] = 0; Speed_cmd[i] = 0;} // Clear buffers 1,2 and 3!!!

  Speed_cmd[0] = Speed_Buff;
  Step_cmd[0] = Dist_Buff;

  Dist_Buff = 0;
  command_got = 0;		//Command Transfered
  ParSet = 0;
  
  delay_us(50000);            // Delay  ~ 50ms
  
  // Wait for the present motion end.
  while(/*Dist_Buff ||*/ (!REF_Stop) || (!command_got)){}
  
  DwellTimer = _Dwell;
  while(DwellTimer){} // Wait for the dwell time
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
void MotionManager(void) 
{
uint16_t i;
static uint16_t OpDataPtr, StartFlag, StartInOld, M03InStateOld=22, SStartOld;
static uint16_t MS05Old;
static uint32_t Dwell = 0;
int32_t REF_Pos_Abs_Tmp, Temp32;

  //================ Operation data number selection mode ====================
  //if(StartInNew && (!StartInOld)){ // Set Start flag on raising edge of StartIn
  if(StartInNew != StartInOld){ // Set Start flag on both edges of StartIn
    StartFlag = 1; 
    //OpDataPtr = M03InState; //
    //i=0;
  }
  StartInOld = StartInNew;
  

  if(M03InStateOld != M03InState){
     M03InStateOld = M03InState;
    OpDataPtr = M03InState;
  }
  
  if(StartFlag && REF_Stop && command_got && (!DwellTimer)){
    if(Dwell){DwellTimer = Dwell; Dwell = 0; return;}
    
    RLPF1.AccDec_time = OpData[OpDataPtr][OpDataAcceleration];
    
    for(i=0;i<=3;i++){ Step_cmd[i] = 0; Speed_cmd[i] = 0;}
    
    REF_Pos_Abs_Tmp = REF_Pos_Abs;
    i=0;
    do{
      if((i>3) && (!AL.Source)) AL.Source = 14; // Source Set
      
      if(OpData[OpDataPtr][OpDataMode] == Incremental){
        Step_cmd[i] = OpData[OpDataPtr][OpDataPosition];
        REF_Pos_Abs_Tmp += Step_cmd[i];
      }
      else{
        Temp32 = OpData[OpDataPtr][OpDataPosition];
        Step_cmd[i] = Temp32 - REF_Pos_Abs_Tmp;
        REF_Pos_Abs_Tmp += Step_cmd[i];
      }

      Speed_cmd[i] = OpData[OpDataPtr][OpDataSpeed];
      i++;
      if(OpDataPtr < 15) OpDataPtr++; else break;
    }while(OpData[OpDataPtr-1][OpDataFunction] == LinkedMotion1);
    
    if(OpData[OpDataPtr-1][OpDataFunction] == LinkedMotion2){
      Dwell = 20*OpData[OpDataPtr-1][OpDataDwellTime]; // 1000 = 1s dwell time
    }
    
    if(OpData[OpDataPtr-1][OpDataFunction] == SingleMotion){
      StartFlag = 0;
    }
    
    // different directions data check
    i=0;
    while(i < 3){
      if((int64_t)((int64_t)Step_cmd[i]*Step_cmd[i+1]) < 0){ // the result must be >= 0
        if(!AL.Source) AL.Source = 13; // Source Set
      }
      i++;
    }
    
    command_got = 0;		// Command Transfered
  }

  //========================== Direct positioning ===========================
  if(MS05Old != MS05){
    MS05Old = MS05;
    
    if(MS05 == MS0){
      for(i=0;i<=3;i++){ Step_cmd[i] = 0; Speed_cmd[i] = 0;}
      RLPF1.AccDec_time = OpData[MS0OpDataPtr][OpDataAcceleration];
      Speed_cmd[0] =      OpData[MS0OpDataPtr][OpDataSpeed];
      Step_cmd[0] =       OpData[MS0OpDataPtr][OpDataPosition];
      command_got = 0;		// Command Transfered
    }
    else if(MS05 == MS1){
      for(i=0;i<=3;i++){ Step_cmd[i] = 0; Speed_cmd[i] = 0;}
      RLPF1.AccDec_time = OpData[MS1OpDataPtr][OpDataAcceleration];
      Speed_cmd[0] =      OpData[MS1OpDataPtr][OpDataSpeed];
      Step_cmd[0] =       OpData[MS1OpDataPtr][OpDataPosition];
      command_got = 0;		// Command Transfered
    }
    else if(MS05 == MS2){
      for(i=0;i<=3;i++){ Step_cmd[i] = 0; Speed_cmd[i] = 0;}
      RLPF1.AccDec_time = OpData[MS2OpDataPtr][OpDataAcceleration];
      Speed_cmd[0] =      OpData[MS2OpDataPtr][OpDataSpeed];
      Step_cmd[0] =       OpData[MS2OpDataPtr][OpDataPosition];
      command_got = 0;		// Command Transfered
    }
    else if(MS05 == MS3){
      for(i=0;i<=3;i++){ Step_cmd[i] = 0; Speed_cmd[i] = 0;}
      RLPF1.AccDec_time = OpData[MS3OpDataPtr][OpDataAcceleration];
      Speed_cmd[0] =      OpData[MS3OpDataPtr][OpDataSpeed];
      Step_cmd[0] =       OpData[MS3OpDataPtr][OpDataPosition];
      command_got = 0;		// Command Transfered
    }
    else if(MS05 == MS4){
      for(i=0;i<=3;i++){ Step_cmd[i] = 0; Speed_cmd[i] = 0;}
      RLPF1.AccDec_time = OpData[MS4OpDataPtr][OpDataAcceleration];
      Speed_cmd[0] =      OpData[MS4OpDataPtr][OpDataSpeed];
      Step_cmd[0] =       OpData[MS4OpDataPtr][OpDataPosition];
      command_got = 0;		// Command Transfered
    }
    else if(MS05 == MS5){
      for(i=0;i<=3;i++){ Step_cmd[i] = 0; Speed_cmd[i] = 0;}
      RLPF1.AccDec_time = OpData[MS5OpDataPtr][OpDataAcceleration];
      Speed_cmd[0] =      OpData[MS5OpDataPtr][OpDataSpeed];
      Step_cmd[0] =       OpData[MS5OpDataPtr][OpDataPosition];
      command_got = 0;		// Command Transfered
    }
  }
  
  //=================== Sequential positioning operation =====================
  if(SStartNew && (!SStartOld)){ // Enter on raising edge of SStart
    // Clear Position and Velocity Massives 
    for(i=0;i<=3;i++){ Step_cmd[i] = 0; Speed_cmd[i] = 0;}
    
    RLPF1.AccDec_time = OpData[OpDataPtr][OpDataAcceleration];
    Speed_cmd[0] =      OpData[OpDataPtr][OpDataSpeed];
    Step_cmd[0] =       OpData[OpDataPtr][OpDataPosition];
    
    if(OpDataPtr < 15) OpDataPtr++;
    if(OpData[OpDataPtr][OpDataSeqPos] == Disable){OpDataPtr = M03InStateOld;}
    
    command_got = 0;		// Command Transfered
  }
  SStartOld = SStartNew;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
/*void Potentiometers_Init(void) 
{



}*/
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
uint32_t Potentiometers(uint32_t Temp) 
{
//#define ADCtoT  1875 // 8Q8 bit format
//#define Vmax  2000 // Max velocity [rpm]

uint32_t PotRefVel;

 //------- Potentiometer Reference Velocity --------
  EP.Sum -= (*EP.ptr);
  *EP.ptr = Temp;			//Filter input
  EP.Sum += Temp;
  if((&EP.mass[EP.MasSize-1])<(++EP.ptr)) EP.ptr = EP.mass;
  PotRefVel = EP.Sum >> EP.MasShift;

//ExtVar[2] = PotRefVel;//<<<<<<<<<<<<<<<<<<<<<<<<<<
  
    //--- Acceleration Time ---
  //Temp = ADCacc*ADCtoT; // 16Q08Q8=24Q8 bit format
  //Temp = Uacc*ADCtoT;         // 16Q08Q8=24Q8 bit format
  Temp >>= 8;                 // 24Q8 -> 32Q0 bit format
  if(Temp < 100) Temp = 100;  // Max Acceleration 50mS(100steps*0.0005S= 0.05S)
  PotAcc = _IQ15div(Vmax,Temp);// 16Q0/32Q0 = 17Q15 bit format;
  
  //--- Decceleration Time ---
  //Temp = ADCdec*ADCtoT; // 16Q08Q8=24Q8 bit format
  //Temp = Udec*ADCtoT;         // 16Q08Q8=24Q8 bit format
  Temp >>= 8;                 // 24Q8 -> 32Q0 bit format
  if(Temp < 100) Temp = 100;  // Max Decceleration 50mS(100steps*0.0005S= 0.05S)
  PotDec = _IQ15div(Vmax,Temp);// 16Q0/32Q0 = 17Q15 bit format;

  return PotRefVel;
}
/*****************************************************************************/



//uint16_t CW = 0, CCW = 0, _SM = 0;
/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
*****************************************************************************/
void RefGen(void)
{
int32_t Temp;
uint16_t CW, CCW;//, LevelPtr;

  TorqueLimitBuff = TorqueLimit[0];
  
  if(DrvFlags0.bit.InterfaceMode == 0){// Display
    if(StartStop) Temp = RefVelocity[0];
    else          Temp = 0;
    dVacc = _IQ15div(Vmax,2*DispAccelTime[0]);// 16Q0/32Q0 = 17Q15 bit format;
    dVdec = _IQ15div(Vmax,2*DispDecelTime[0]);// 16Q0/32Q0 = 17Q15 bit format;
    dVdir = DrvFlags0.bit.MotionDirection;
  }
  else{
    //CW = _IOEmul & 0x0002;
    //CCW = (_IOEmul & 0x0004) >> 1;
      
    CW =  IOInputReg & CW_Pin;
    CCW = IOInputReg & CCW_Pin;

    if(DrvFlags0.bit.InterfaceMode == 1){ // External Potentiometer
      //Temp = Potentiometers(Uextpot >> 1);
    }
    else if(DrvFlags0.bit.InterfaceMode == 2){ // Internal Potentiometer
      //Temp = Potentiometers(Uintpot >> 1);
    }
    else{
      LevelPtr = 0;
      if(IOInputReg & M0_Pin) LevelPtr =  1;
      if(IOInputReg & M1_Pin) LevelPtr += 2;
      if(IOInputReg & M2_Pin) LevelPtr += 4;
      switch(LevelPtr){
         case 0: 
           Temp = RefVelocity[0]; 
           PotAcc = _IQ15div(Vmax,2*DispAccelTime[0]);// 16Q0/32Q0 = 17Q15 bit format;
           PotDec = _IQ15div(Vmax,2*DispDecelTime[0]);// 16Q0/32Q0 = 17Q15 bit format;
           TorqueLimitBuff = TorqueLimit[0]; 
         break;
         case 1: 
           Temp = RefVelocity[1]; 
           PotAcc = _IQ15div(Vmax,2*DispAccelTime[1]);// 16Q0/32Q0 = 17Q15 bit format;
           PotDec = _IQ15div(Vmax,2*DispDecelTime[1]);// 16Q0/32Q0 = 17Q15 bit format;
           TorqueLimitBuff = TorqueLimit[1];
         break;
         case 2: 
           Temp = RefVelocity[2]; 
           PotAcc = _IQ15div(Vmax,2*DispAccelTime[2]);// 16Q0/32Q0 = 17Q15 bit format;
           PotDec = _IQ15div(Vmax,2*DispDecelTime[2]);// 16Q0/32Q0 = 17Q15 bit format;
           TorqueLimitBuff = TorqueLimit[2];
         break;
         case 3: 
           Temp = RefVelocity[3]; 
           PotAcc = _IQ15div(Vmax,2*DispAccelTime[3]);// 16Q0/32Q0 = 17Q15 bit format;
           PotDec = _IQ15div(Vmax,2*DispDecelTime[3]);// 16Q0/32Q0 = 17Q15 bit format;
           TorqueLimitBuff = TorqueLimit[3];
         break;
         case 4:
           Temp = RefVelocity[4]; 
           PotAcc = _IQ15div(Vmax,2*DispAccelTime[4]);// 16Q0/32Q0 = 17Q15 bit format;
           PotDec = _IQ15div(Vmax,2*DispDecelTime[4]);// 16Q0/32Q0 = 17Q15 bit format;
           TorqueLimitBuff = TorqueLimit[4];
         break;
         case 5:
           Temp = RefVelocity[5]; 
           PotAcc = _IQ15div(Vmax,2*DispAccelTime[5]);// 16Q0/32Q0 = 17Q15 bit format;
           PotDec = _IQ15div(Vmax,2*DispDecelTime[5]);// 16Q0/32Q0 = 17Q15 bit format;
           TorqueLimitBuff = TorqueLimit[5];
         break;
         case 6: 
           Temp = RefVelocity[6]; 
           PotAcc = _IQ15div(Vmax,2*DispAccelTime[6]);// 16Q0/32Q0 = 17Q15 bit format;
           PotDec = _IQ15div(Vmax,2*DispDecelTime[6]);// 16Q0/32Q0 = 17Q15 bit format;
           TorqueLimitBuff = TorqueLimit[6];
         break;
         default:
           Temp = RefVelocity[7]; 
           PotAcc = _IQ15div(Vmax,2*DispAccelTime[7]);// 16Q0/32Q0 = 17Q15 bit format;
           PotDec = _IQ15div(Vmax,2*DispDecelTime[7]);// 16Q0/32Q0 = 17Q15 bit format;
           TorqueLimitBuff = TorqueLimit[7];
         break;
      }
      //Temp = RefVelocity[1];// for temporary use
    }
    
    dVacc = PotAcc;
    dVdec = PotDec;
    
  // AND of DIR/ENABLE inputs
    if(CW&&CCW){ // if both inputs are set - Driver Instantaneous stop
      Temp = 0;
      //dVdec = RefVel;
      RefVel = 0;
    }
    else{  // Direction changening when only one inp. is set or reset 
      if(CW)       dVdir = 0; // CW
      else if(CCW) dVdir = 1; // CCW
      else{
        Temp = 0;
        // if Stop Mode input is reset - Driver Instantaneous stop
        if(!(IOInputReg & SM_Pin))  RefVel = 0;
      }
    }   
  }
  
  Temp <<= 15; // 32Q0 -> 17Q15 bit format
  if(RefVel < Temp){
    RefVel += dVacc;
    if(RefVel > Temp) RefVel = Temp;
    
    status_word |=  FFLAG_MOTIONACCEL;    // Set Acceleration flag
    status_word &= ~FFLAG_MOTIONDECEL;    // Reset Deceleration flag
    status_word &= ~FFLAG_MOTIONCONST;    // Reset Constant speed flag
  }
  else if(RefVel > Temp){
    RefVel -= dVdec;
    if(RefVel < Temp) RefVel = Temp;
    
    status_word &= ~FFLAG_MOTIONACCEL;    // Reset Acceleration flag
    status_word |=  FFLAG_MOTIONDECEL;    // Set Deceleration flag
    status_word &= ~FFLAG_MOTIONCONST;    // Reset Constant speed flag
  }
  else if(Temp){
    status_word &= ~FFLAG_MOTIONACCEL;    // Reset Acceleration flag
    status_word &= ~FFLAG_MOTIONDECEL;    // Reset Deceleration flag
    status_word |=  FFLAG_MOTIONCONST;    // Set Constant speed flag
  }
  if(RefVel < 0) RefVel = 0;

  //---- Reference Velosity Set ----
  Temp = RefVel >> 15; // 17Q15 -> 32Q0 bit format

  //-------- Servo On/Off Hysteresys ------
  //if(Temp >= 50)      Servo_En = 1;
  //else if(Temp <= 30) Servo_En = 0;
  
  if(!OutputsOverride) ServoOnOutSet;  // Servo On/Off Set
  
  //---------- Reference Velocity Limitation -----------
  if(Temp > 3300)   Temp = 3300;
  else if(Temp < 0) Temp = 0;

  if(dVdir & 1) Temp*= -1;

  //VelocityFBController.Ref = Temp;

//ExtVar[3] = Temp;//<<<<<<<<<<<<<<<<<<<<<<<<<<
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/






















