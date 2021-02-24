/**
  ******************************************************************************
  * @file    stm32fxxx_it.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2016-01-14
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  *
  *
  *
  *
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm32fxxx_it.h"
#include "PWM_Functions.h"
#include "ADC_functions.h"
#include "USART_functions.h"
#include "USART_functions_2.h"
#include "Command_functions.h"
#include "GPIO_functions.h"
#include "Velocity_Measure.h"
//#include "usb_core.h"
//#include "usbd_core.h"
#include "Background.h"
#include "TypesDef.h"
#include <stdlib.h>
#include "SysMonitor.h"
#include "IQmathlib.h"         	/* include header for IQmath library 		*/
#include "Motor.h"
#include "TimersFunctions.h"
#include "InertiaIdentification.h"

#include "arm_itm.h" // Ton adding

extern int16_t NewCommand;

extern int16_t ExtVar[];
extern uint16_t Hall;
extern int16_t PhA, Pcorr;
extern uint32_t PWMTimerCounterOld;
uint32_t hallccr1;
uint16_t MDir, HallOld, SkipFirst, Ttimer , Htimer;
uint16_t cn = 0;
sDW dPhAc;

//uint16_t Hoang = 5;
//uint16_t Hoang1 = 9;

#if defined (USE_STM324xG_BLDC_HAL)
 #include "stm324xg_bldc_hal.h"

#elif defined (USE_STM322xG_EVAL)
 #include "stm322xg_eval.h"
 #include "stm322xg_eval_ioe.h"
#elif defined(USE_STM324xG_EVAL)
 #include "stm324xg_eval.h"
 #include "stm324xg_eval_ioe.h"
#elif defined (USE_STM3210C_EVAL)
 #include "stm3210c_eval.h"
 #include "stm3210c_eval_ioe.h"
#else
 #error "Missing define: Evaluation board (ie. USE_STM322xG_EVAL)"
#endif

//#include "usbd_cdc_core.h"

//#include "lcd_log.h"
    
#include "Config.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
//extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
//extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED 
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif

/******************************************************************************/
/*             Cortex-M Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  /* Information panel */
  //LCD_SetTextColor(Green);
  //LCD_DisplayStringLine( LCD_PIXEL_HEIGHT - 42, USER_INFORMATION[x]);  
  //LCD_SetTextColor(LCD_LOG_DEFAULT_COLOR);
}



void TIM14_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
    
    Bkg_1mS();
  }
}



void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  TIM14_IRQHandler();
}



/**
  * @brief  This function handles EXTI15_10_IRQ Handler.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_OTG_FS  
void OTG_FS_WKUP_IRQHandler(void)
{
  /*if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ; 
    SystemInit();
    //USB_OTG_UngateClock(&USB_OTG_dev);
  }*/
  EXTI_ClearITPendingBit(EXTI_Line18);
}
#endif

/**
  * @brief  This function handles EXTI15_10_IRQ Handler.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_OTG_HS  
void OTG_HS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ; 
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line20);
}
#endif

/**
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_OTG_HS  
void OTG_HS_IRQHandler(void)
#else
void OTG_FS_IRQHandler(void)
#endif
{
  //USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED 
/**
  * @brief  This function handles EP1_IN Handler.
  * @param  None
  * @retval None
  */
void OTG_HS_EP1_IN_IRQHandler(void)
{
  USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev);
}

/**
  * @brief  This function handles EP1_OUT Handler.
  * @param  None
  * @retval None
  */
void OTG_HS_EP1_OUT_IRQHandler(void)
{
  USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev);
}
#endif


/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
/******************************************************************************/

/******************************************************************************
  * @brief  This function handles ADC Handler.
  * @param  None
  * @retval None
******************************************************************************/
void ADC_IRQHandler(void)
{
  GPIOA->BSRRH = GPIO_Pin_4; // Reset pin
  PWMTimerCounterOld = TIM_GetCounter(PWMTimer);
    
  // ADC1(+ADC2 and ADC3) regular chanels Software Conversion
  ADC_SoftwareStartConv(ADC1);

  ADC1_ISR();

  //ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
  ADC1->SR &= ~(uint32_t)ADC_FLAG_JEOC;  // ADC1 Clear IT JEOC Pending Bit
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  This function handles TIM6 Handler.
  * @param  None
  * @retval None
*****************************************************************************/
void TIM6_DAC_IRQHandler(void)
{
  TIM6_Enable(DISABLE);
  TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

  // Reset USART Rx DMA
  USART3_DMA_Reset();
}
/*****************************************************************************/




/******************************************************************************
  * @brief  This function handles TIM8 update Handler.
  * @param  None
  * @retval None
******************************************************************************/
void TIM8_UP_TIM13_IRQHandler(void)
{
  GPIOA->BSRRL = GPIO_Pin_4; // Set pin
  
  Ttimer++;
  Htimer++;
  if(Ttimer == 1){ // 2kHz sampling frequency
    FeedbackPosition();
    VelocityMeas();  
    
    //Inertia_total();
    
    
    //Hoang++;
    //Hoang1++;
    //ITM_EVENT16_WITH_PC(1, Hoang);
    //ITM_EVENT16_WITH_PC(2, Hoang1);
  }
  else if(Ttimer >= 10)
  {
    Ttimer = 0;
  }
  
  if(Htimer == 1)
  {     // 20Hz sampling frequency  
    cn++;
    //ITM_EVENT16_WITH_PC(1, cn); 
    Inertia_total();
  }
  else if(Htimer >= 100) 
  {
    Htimer = 0;
  } 
  
   TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
}
/*****************************************************************************/




/******************************************************************************
  * @brief  This function handles USART2 RX/TX interrupt requests.
  * @param  None
  * @retval None
******************************************************************************/
void USART2_IRQHandler(void)
{
  //USART_DataRead();
  ReceiveCommand();
  
}
/*****************************************************************************/




/******************************************************************************
  * @brief  This function handles USART3 RX/TX interrupt requests.
  * @param  None
  * @retval None
******************************************************************************/
void USART3_IRQHandler(void)
{
  TIM6_Reset();
}
/*****************************************************************************/




/******************************************************************************
  * @brief  This function handles USART3_DMA_RX_IRQHandler
  * @param  None
  * @retval None
******************************************************************************/
void DMA1_Stream1_IRQHandler(void)
{
  TIM6_Enable(DISABLE);

  NewCommand = 1;
  
  /*CommandManagement();
  
  // USART3 configuration
  //USART3_Config();
  COM3ReInit();
  
  // Initialize DMA in Memory to Periferal mode(USART transmit)
  USART_Tx_DMA_Init();*/
  
  // Clear all DMA Rx Streams flags
  DMA_USART_RX_Flags_Clear(COM3);
}
/*****************************************************************************/




/******************************************************************************
  * @brief  This function handles USART3_DMA_TX_IRQHandler
  * @param  None
  * @retval None
******************************************************************************/
void DMA1_Stream3_IRQHandler(void)
{  
  // Initialize DMA in Periferal to Memory mode(USART receive)
  USART_Rx_DMA_Init(COM3);

  // Clear all DMA Tx Streams flags
  DMA_USART_TX_Flags_Clear(COM3);

}
/*****************************************************************************/




/******************************************************************************
  * @brief  This function handles USART2_DMA_TX_IRQHandler
  * @param  None
  * @retval None
******************************************************************************/
void USART2_DMA_TX_IRQHandler(void)
{
  // Clear all DMA Tx Streams flags
  DMA_USART_TX_Flags_Clear(COM2);
}


/******************************************************************************
  * @brief  This function handles Timer 1 Commutation Handler.
  * @param  None
  * @retval None
******************************************************************************/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  if(TIM1->SR & TIM_IT_COM){
    // Clear the IT pending Bit
    TIM1->SR = ~TIM_IT_COM;
  }
}
/*****************************************************************************/


int16_t PhAOld;
int16_t Ph = 15;//, LastPhA, delta; // Range from 0 to 166
/******************************************************************************
* Function Name : TIM2_IRQHandler
* Description   : This function handles Timer 2 Chanal 1 Capture and Chanal 2 Compare Handler.
* Input         : None
* Output        : None
* Return        : None
******************************************************************************/
void TIM2_IRQHandler(void)
{
static int32_t Vel, VelF;
static int16_t VelocityHallFilter;//, InterpolationEn=0;
uint16_t Hall = def_HALL;
//sQW Temp;

//ExtVar[1] = TIM2->SR;//<<<<<<<<<<<<<<<<<<<<<

#define Degree60  1000.0/6.0

  if(HallOld != Hall){
    //SinAngle.dw = 0; // Clear Position only ones per every Hall change!!!
    //PhAInterp.dw = 0;// Clear Position only ones per every Hall change!!!
    
    PhAOld = PhA;
  }
  
  switch(Hall){
    case Hall4:
      PhA = Ph + (int16_t)(5*Degree60); 
      if(HallOld == Hall5) MDir = 1; else 
      if(HallOld == Hall3) MDir = 0;
    break;
    
    case Hall3:
      PhA = Ph + (int16_t)(0*Degree60); 
      if(HallOld == Hall4) MDir = 1; else 
      if(HallOld == Hall2) MDir = 0;
    break;
    
    case Hall2:
      PhA = Ph + (int16_t)(1*Degree60); 
      if(HallOld == Hall3) MDir = 1; else 
      if(HallOld == Hall1) MDir = 0;
    break;
    
    case Hall1:
      PhA = Ph + (int16_t)(2*Degree60); 
      if(HallOld == Hall2) MDir = 1; else 
      if(HallOld == Hall0) MDir = 0;
    break;
    
    case Hall0:
      PhA = Ph + (int16_t)(3*Degree60); 
      if(HallOld == Hall1) MDir = 1; else 
      if(HallOld == Hall5) MDir = 0;
    break;
    
    case Hall5:
      PhA = Ph + (int16_t)(4*Degree60); 
      if(HallOld == Hall0) MDir = 1; else 
      if(HallOld == Hall4) MDir = 0;
    break;
    default: if(!AL.Source)  AL.Source = 7;	// Alarm Source Set
  }
  
  HallOld = Hall;
  
  //------------------------------
  //Temp.w[0] = PhA + _Offset - RotorRawPosition;
  
  //--- Wrapping -----
  //if(Temp.w[0] >= 1000)  Temp.w[0] -= 1000;
  //else if(Temp.w[0] < 0) Temp.w[0] += 1000;
  
  //Pcorr = Temp.w[0];
  //------------------------------
  
  /*if(Hall == Hall5) ExtVar[0] = 1; else
  if(Hall == Hall4) ExtVar[0] = 2; else
  if(Hall == Hall6) ExtVar[0] = 3; else
  if(Hall == Hall2) ExtVar[0] = 4; else
  if(Hall == Hall3) ExtVar[0] = 5; else
  if(Hall == Hall1) ExtVar[0] = 6;*/

  
//ExtVar[3] = MDir;//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  //if(TIM2->SR & TIM_IT_CC1)
  { 
    // Clear the IT pending Bit
    //TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    TIM2->SR = (uint16_t)~TIM_IT_CC1;
    
    // calculate motor speed or else with CCR1 values
    //hallccr1 = TIM2->CCR1 >> 3;
      //---------------- Hall Sensors Velocity --------
  // Used method V = X/(t2 - t1) = (TimerCLK*60)/(2*encoder resolution*TimerCounts)[rpm]
  // V = X/(t2-t1)= (TimerCLK*60)/(2*Halls*PolePairs*counts)[rpm] = 
  // = 96e10^6*60/2*6*5*TimerCounts[rpm]=96e10^6/TimerCounts[rpm]
    // Reading TIM2->CCR1 cleans CC1IF flag in TIM2_SR register !!!!
    //Vel = 192000000/TIM2->CCR1;//hallccr1;
    Vel = 960000000/((int32_t)MOT.PolePairs*TIM2->CCR1);
    
    if(SkipFirst){ SkipFirst--; Vel = 0;}
    
    if(MDir && (Vel > 0)) Vel = -Vel; // Velocity Sign
    
    //--- Reading TIM2->CCR1 cleans CC1IF flag in TIM2_SR register !!!! ---
    //Temp.qw = (int64_t)TIM2->CCR1*894785; // 32Q0*0Q32=32Q32 bit format; f0/SYSclk = 894785
    //dX = Temp.dw[1];                      // 32Q0 bit format
  } 
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
  //else if(TIM2->SR & TIM_IT_CC2)
  { 
    // Clear the IT pending Bit
    //TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    TIM2->SR = (uint16_t)~TIM_IT_CC2;
    //ExtVar[0] = TIM2->CCR1 >> 8;//<<<<<<<<<<<<<<<<<<
    // Reading TIM2->CCR2 cleans CC2IF flag in TIM2_SR register !!!!
    Vel = 0;//120000000/TIM2->CCR2;
    //if(MDir) Vel = -Vel; // Velocity Sign
    
    //dX = 2147483647;                      // 32Q0 bit format
  } 
  else if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  //else if(TIM2->SR & TIM_IT_Update)
  {
    // Clear the IT pending Bit
    //TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    /*TIM2->SR = (uint16_t)~TIM_IT_Update;
    
    Vel = 120000000/(TIM2->ARR+1);//hallccr1;
         if(MDir) Vel = -Vel;*/ // Velocity Sign
  }
  else { 
    ; // this should not happen 
  }
   
  //ExtVar[0] = Vel; //<<<<<<<<<<<<<<<<<<<<<<<<<<
  //--------------------- 6 points Moving Average Filter ---------------------
  /*FB.Sum3 -= (*FB.ptr3);
  *FB.ptr3 = Vel;			//Filter input
  FB.Sum3 += Vel;
  if((&FB.mass3[5])<(++FB.ptr3)) FB.ptr3 = FB.mass3;
  //VelF = FB.Sum3/6;    // division by 6
  VelF = FB.Sum3*341;///6;    // 32Q0*8Q8=24Q8 bit format
  VelF >>= 11;*/ //24Q8 -> 32Q0 bit format & division by 8

//ExtVar[2] = VelF;//<<<<<<<<<<<<<<<<<<<<<<<<<<
    // Filter hysteresis
  if((VelF < -400) || (VelF > 400))      VelocityHallFilter = 1;
  else if((VelF > -300) && (VelF < 300)) VelocityHallFilter = 0;

  // ----------- Filter Enable/Disable ------------
  if(VelocityHallFilter) FB.VelocityHall = VelF;
  else                   FB.VelocityHall = Vel;

  
  //--------- Velocity Monitoring Filter -------------
  FB.SumMon -= (*FB.ptrMon);
  *FB.ptrMon = Vel;			//Filter input
  FB.SumMon += Vel;
  if((&FB.massMon[FB.MasMonSize-1])<(++FB.ptrMon)) FB.ptrMon = FB.massMon;
  FB.VelocityMonitor = (FB.SumMon*FB.MasMonShift) >> 15; // 32Q0*1Q15=17Q15 -> 32Q0 bit format
  
//ExtVar[0] = PhA;//MDir;//VelocityHallFilter;//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//ExtVar[1] = VelF;//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//ExtVar[2] = AngleScale >> 3;//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//ExtVar[0] = Vel;//FB.VelocityHall;//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/

