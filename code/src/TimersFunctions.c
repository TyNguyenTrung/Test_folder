/**
  ******************************************************************************
  * @file    TimersFunctions.c 
  * @author  A.Andreev
  * @version V1.0.0
  * @date    2015-03-04
  * @brief
  ******************************************************************************
  * @attention
  *
  *
  * 
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "TimersFunctions.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ENCODER_PPR 5000
#define REF_PPR 200
#define ICx_FILTER          (u8) 0xF // 8 samples with Fck_int

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

union TIMERS_FILTERS TIMERS_Filters;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief Reference unit connected to TIM1, 4X mode
  * @param  None
  * @retval None
*****************************************************************************/
void TIM1_Config(void)
{
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_ICInitTypeDef TIM_ICInitStructure;
//TIM_OCInitTypeDef  TIM_OCInitStructure;
//TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

  /* TIM1 clock enable - 192 MHz */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  
  TIM_DeInit(TIM1);
  
  // Timer configuration in Encoder mode 3
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = RefTimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, 
                             TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  // Set the default configuration
//  TIM_ICStructInit(&TIM_ICInitStructure);

  // Capture Input Init
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = TIMERS_Filters.bit.FB; //ICx_FILTER;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
      
 // Clear all pending interrupts
//  TIM_ClearFlag(ENCODER_TIMER, TIM_FLAG_Update);
//  TIM_ITConfig(ENCODER_TIMER, TIM_IT_Update, ENABLE);
  //Reset counter
//  TIM1->CNT = COUNTER_RESET;
  
  TIM_Cmd(TIM1, ENABLE);
  
  // TIM1 Counter Clock runs freely when the core is halted
  DBGMCU_Config(DBGMCU_TIM1_STOP, DISABLE);
}
/******************************************************************************/




/*****************************************************************************
  * @brief TIM2 configured as a Hall interface timer
  * @param  None
  * @retval None
*****************************************************************************/
void TIM2_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  /* TIM2 clock source enable - 96 MHz */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_DeInit(TIM2);

  /* Time Base Defauts Set */  
//  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  // Timer configuration in Hall Interfacing Mode
  TIM_TimeBaseStructure.TIM_Prescaler = 0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 96000000;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 
  // Enable hall sensor; T1F_ED will be connected to  HallSensor Imputs
  // TIM2_CH1,TIM2_CH2,TIM2_CH3 
  TIM_SelectHallSensor(TIM2, ENABLE);
  
  // HallSensor event is delivered with singnal TI1F_ED
  // (this is XOR of the three hall sensor lines)
  // Signal TI1F_ED: falling and rising edge of the inputs is used 
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI1F_ED);
  
  // On every TI1F_ED event the counter is resetted and update is tiggered 
  TIM_SelectSlaveMode(TIM2,TIM_SlaveMode_Reset);
  
  // Set the default configuration
//  TIM_ICStructInit(&TIM_ICInitStructure);

   // Channel 1 in input capture mode 
  // on every TCR edge (build from TI1F_ED which is a HallSensor edge)  
  // the timervalue is copied into ccr register and a CCR1 Interrupt
  // TIM_IT_CC1 is fired
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;// listen to T1, the  HallSensorEvent
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = ICx_FILTER;// Noise filter: 1111 => 192MHz / factor (==4) / 32 / 8 -> 187.5kHz
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
  // channel 2 can be use for commution delay between hallsensor edge
  // and switching the FET into the next step. if this delay time is
  // over the channel 2 generates the commutation signal to the motor timer
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;//Enable; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM_OCInitStructure.TIM_Pulse = 38400000;// 7680000; // 25RPM min velocity;
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);//<<<<<<<<<<<<<<<<<<<<<<<<<<
  
  // Source of Update event is only counter overflow/underflow
  TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);
    
  // Clear the TIM2's pending interrupt flags
  TIM_ClearFlag(TIM2, TIM_FLAG_Update + TIM_FLAG_CC1 + TIM_FLAG_CC2 + \
                TIM_FLAG_CC3 + TIM_FLAG_CC4 + TIM_FLAG_Trigger + TIM_FLAG_CC1OF + \
                TIM_FLAG_CC2OF + TIM_FLAG_CC3OF + TIM_FLAG_CC4OF);
  
  // timer 2 output compate signal is connected to TRIGO 
//  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_OC2Ref);

  
// --------- Activate the Hall Timer ----------
  // in a project this will be done late after complete 
  // configuration of other peripherie

  // Enable channel 1 capture and channel 2 compate interrupt requests
  TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 /*| TIM_IT_Update*/, ENABLE);
  
  // Reset Timer Counter to "0"
  TIM_SetCounter(TIM2, 0);
  
  // HallSensor is now configured, if BLDC Timer is also configured after 
  // enabling timer 2 the motor will start after next overflow of the hall 
  // timer because this generates the first startup motor cummutation event
//  TIM_Cmd(TIM2, ENABLE); Hard fault happes after that!!!!!
}
/******************************************************************************/




/*****************************************************************************
  * @brief Encoder unit connected to TIM3, 4X mode
  * @param  None
  * @retval None
*****************************************************************************/
void TIM3_Config(void)
{

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_ICInitTypeDef TIM_ICInitStructure;
  
  // TIM3 clock source enable - 96 MHz
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_DeInit(TIM3);

  // Time Base Defauts Set
//  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  // Timer configuration in Encoder mode 3
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, 
                             TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  // Set the default configuration
//  TIM_ICStructInit(&TIM_ICInitStructure);

  // Capture Input Init
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = TIMERS_Filters.bit.FB; //ICx_FILTER;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
      
 // Clear all pending interrupts
//  TIM_ClearFlag(ENCODER_TIMER, TIM_FLAG_Update);
//  TIM_ITConfig(ENCODER_TIMER, TIM_IT_Update, ENABLE);
  //Reset counter
//  TIM3->CNT = COUNTER_RESET;
  
  TIM_Cmd(TIM3, ENABLE);
}
/******************************************************************************/




/*****************************************************************************
  * @brief Reference unit connected to TIM5, 4X mode
  * @param  None
  * @retval None
*****************************************************************************/
void TIM5_Config(void)
{

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_ICInitTypeDef TIM_ICInitStructure;
  
  // TIM5 clock source enable - 96 MHz
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  TIM_DeInit(TIM5);

  // Time Base Defauts Set
//  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  // Timer configuration in Encoder mode 3
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
 
  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, 
                             TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  // Set the default configuration
//  TIM_ICStructInit(&TIM_ICInitStructure);

  // Capture Input Init
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = TIMERS_Filters.bit.FB; //ICx_FILTER;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
      
 // Clear all pending interrupts
//  TIM_ClearFlag(ENCODER_TIMER, TIM_FLAG_Update);
//  TIM_ITConfig(ENCODER_TIMER, TIM_IT_Update, ENABLE);
  //Reset counter
//  TIM5->CNT = COUNTER_RESET;
  
  TIM_Cmd(TIM5, ENABLE);
}
/******************************************************************************/



                       
/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void TIM6_Config(void)
{
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  // TIM6 clock source enable - 96 MHz ???????????
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  TIM_DeInit(TIM6);

  // Time Base Defauts Set
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  /* Timer configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 960 - 1;  // 96MHz / 960 = 100KHz = 10us
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 1000;  	// 10us * 1000 = 10ms
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM6, DISABLE);	// No reload

  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void TIM6_Enable(int bEnable)
{
  TIM_Cmd(TIM6, (FunctionalState)bEnable);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void TIM6_Reset(void)
{
  TIM_SetCounter(TIM6, 0);
  TIM_Cmd(TIM6, ENABLE);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void TIM7_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  /* TIM7 clock source enable - 60 MHz */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

  TIM_DeInit(TIM7);

  /* Time Base Defauts Set */  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  /* Timer configuration in Encoder mode 3 */
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 65535;//Timer7Period;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM7,ENABLE);
  
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

  //TIM_Cmd(TIM7, ENABLE);
}
/******************************************************************************/




/*****************************************************************************
  * @brief TIM8 is configured for PWM generation 
  * @param  None
  * @retval None
*****************************************************************************/
void TIM8_Config(void)
{
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

  /* TIM8 clock enable - 192 MHz */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 , ENABLE);
  
  TIM_DeInit(TIM8);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  TIM_TimeBaseStructure.TIM_Period = PWMTimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;// Initial condition is REP=0 to set the UPDATE only on the underflow
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

  /* Channel 1, 2, 3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWMTimerPeriodMiddle;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  TIM_OC1Init(TIM8, &TIM_OCInitStructure);
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);
  TIM_OC3Init(TIM8, &TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_Pulse = PWMTimerPeriod-1; //
  TIM_OC4Init(TIM8, &TIM_OCInitStructure); // Trigger event for Injected ADC channels

  /* Capture/Compare registers preload ENABLE/DISABLE */
  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

  
  /* ENABLE/DISABLE preload of CCxE, CCxNE and OCxM bits */
  TIM_CCPreloadControl(TIM8, ENABLE);
  
  /* Automatic Output enable, Break, dead time and lock configuration*/
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  TIM_BDTRInitStructure.TIM_DeadTime = 0;// - Hardware Dead time
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
  
  //TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);

  TIM_ClearITPendingBit(TIM8, TIM_IT_Break);
  TIM_ITConfig(TIM8, TIM_IT_Break,ENABLE);
  
  // activate COM (Commutation) Event from Slave (HallSensor timer)
  // through TRGI(enable the connection between HallTimer and MotorTimer)
  //TIM_SelectCOM(TIM8, ENABLE);
  
  // Internal connection from Hall/Enc Timer to Motor Timer
  // eg. TIM8 (BLDC Motor Timer) is Slave of TIM2 (Hall Timer)
  TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
  
// --------- activate the bldc bridge ctrl. ----------
  // in a project this will be done late after complete 
  // configuration of other periphery
  
  // Clear Pending Update Interrupt Flag
  //TIM_ClearFlag(TIM8, TIM_FLAG_Update);<<<<<<<<<<????????????
    
  // enable Update IRQ
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
  
  // TIM8 counter enable
  //TIM_Cmd(TIM8, ENABLE);

  // TIM8 Main Output Enable
  TIM_CtrlPWMOutputs(TIM8, ENABLE);
  
  // TIM8 Counter Clock runs freely when the core is halted
  DBGMCU_Config(DBGMCU_TIM8_STOP, DISABLE);
}
/******************************************************************************/




/*****************************************************************************
  * @brief TIM14 is configured for delay generation 
  * @param  None
  * @retval None
*****************************************************************************/
void TIM14_Config(void)
{
  //NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  RCC_ClocksTypeDef sRCC;
  
  /* TIM14 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
  
  /* Get Core and Peripherals clock values */
  SystemCoreClockUpdate();
  RCC_GetClocksFreq(&sRCC);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((sRCC.PCLK2_Frequency / 1000000UL) - 1); // Make TMR clock equall to 1MHz (uS)
  TIM_TimeBaseStructure.TIM_Period = (1000) - 1; // Make TMR period 1kHz (1 mS Interrupt rate)
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
 
  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
 

  //----------------------------------
  /* Enable the TIM14 global Interrupt */
  /*NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/
  
  
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
  //---------------------------------
 
  /* TIM14 enable counter */
  //TIM_Cmd(TIM14, ENABLE);
}
/******************************** END OF FILE ********************************/
