/**
  ******************************************************************************
  * @file    ADC_functions.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2015-03-12
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
#include "ADC_functions.h"
#include "PWM_Functions.h"
#include "CurrentControl.h"
#include "HardwareConfiguration.h"
#include "TypesDef.h"
#include "Velocity_Measure.h"
#include "PositionVelocityControl.h"
#include "REFGenFunctions.h"
#include "InertiaIdentification.h"
//#include "Observer.h"
#include "SIN_Tab.h"
#include "stm324xg_bldc_hal.h"
#include "TuningFunctions.h"
#include "Motor.h"

inline int16_t REFLPF2(register int16_t);

extern int16_t ExtVar[];
extern int16_t UqRef;
extern uint16_t Hall;
extern int32_t Speed_Buff; 	// command maximum speed (from RS232)
extern uint32_t hallccr1;
extern int16_t PhAOld;
extern uint32_t DwellTimer;

extern uint32_t RL_timer;

extern sDW dPhAc;
extern int16_t PhA;
int16_t MonMass1[20000], MonMass2[20000];
uint32_t MMPtr=0;

extern int16_t LPF2Pos;
//extern int16_t LPF2PosOld;
extern int32_t LPF2PosOv;
extern int32_t LPF2PosOvOld;
extern int32_t LPF2Sum;
extern int16_t LPF2RemSum;
extern int16_t LPF2Mas[];
extern int16_t LPF2CurrentBuffSize;
extern int16_t LPF2Index;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint16_t  volatile ADCInt = 0;
uint16_t Vtimer = 0, /*STimer = 0,*/ Vtimer2 = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/*****************************************************************************
  * @brief  ADC1 Interrupt Service Routine
  * @param  None
  * @retval None
  ***************************************************************************/
void ADC1_ISR(void)
{
//sDW Temp;
//GPIOA->BSRRL = GPIO_Pin_4; // Set pin
//LEDToggle(LED_G);
  
//FeedbackPosition();
  PMSM_ADC_Read_Iabc_2_IAlfaBeta_2_IdIq();
//GPIOA->BSRRH = GPIO_Pin_4; // Reset pin
  if(CC.ServoOn){
    Vtimer++;
    
    //FeedbackPosition();
    PMSM_CurrentController();                        // Main current control
    //GPIOA->BSRRH = GPIO_Pin_4; // Reset pin
    //Observer();
    
    if(Vtimer == 1){ /*VelocityMeas();*/ 
                     //Hoang adding tuning parameter KP-KI// 
                     //TuningPosition();
                     PositionControl();		
                   }
    else
    if(Vtimer == 2){ 
                     // Hoang adding tuning parameter KP-KI//
                     //TuningVelocity();
                     VelocityControl();                         
                   }
    else
    if(Vtimer == 3){ /*speed_cmd();*/                               }else
    if(Vtimer == 4){ /*LSM_DataCollect();*/                         }else
    if(Vtimer == 5){ /*CurrentPulses();*/                           }else
    if(Vtimer == 10){Vtimer = 0; /*TuningCriterion();*/             }else // 20kHz/10 = 2kHz sampling ferquency
                   {	                                        }
  }

  //- Ext/Internalernal Potentiometer Reading -
  /*STimer++;
  if(STimer == 10){RefGen(); STimer = 0;                }*/
  
//----- External Used Variables at 20 kHz ADC Interrupt Frequency -----
  ADCInt = -1;  // a new ADC convertion is done

  //---------------
  //RL_timer++;
        
  //------------------------- Dwell Timer ------------------------
  //if(DwellTimer) DwellTimer--;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
  ***************************************************************************/
int16_t _3_poitsMedianFilter(int16_t a, int16_t b, int16_t c)
{
int16_t out;
    if(c > b){
           if(b > a)   out = b;
           else{   
              if(c > a) out = a;
              else      out = c;
            }
    }
    else{
          if(c > a)      out = c;
          else if(b > a) out = a;
          else           out = b;
    }
  return out;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
  ***************************************************************************/
void PMSM_ADC_Read_Iabc_2_IAlfaBeta_2_IdIq(void)
{
sDW Temp;
int16_t Ia_tmp, Ib_tmp, Ic_tmp;

  // Current scaling
  Ia_tmp = _3_poitsMedianFilter(ADC1->JDR1, ADC1->JDR2, ADC1->JDR3);
  Temp.dw = (int32_t)Ia_tmp*HC.Iscaling; // 7Q9*5Q11=12Q20 bit format;
  Temp.dw >>= 11;			 // 12Q20 -> 23Q9bit format;
  Ia_tmp = CC.Zero_a - Temp.w[0];        // 7Q9bit format; Current sign inversion!!!
  
  Ib_tmp = _3_poitsMedianFilter(ADC2->JDR1, ADC2->JDR2, ADC2->JDR3);
  Temp.dw = (int32_t)Ib_tmp*HC.Iscaling; // 7Q9*5Q11=12Q20 bit format;
  Temp.dw >>= 11;			 // 12Q20 -> 23Q9bit format;
  Ib_tmp = CC.Zero_b - Temp.w[0];        // 7Q9bit format; Current sign inversion!!!
  
  Ic_tmp = _3_poitsMedianFilter(ADC3->JDR1, ADC3->JDR2, ADC3->JDR3);
  Temp.dw = (int32_t)Ic_tmp*HC.Iscaling; // 7Q9*5Q11=12Q20 bit format;
  Temp.dw >>= 11;			 // 12Q20 -> 23Q9bit format;
  Ic_tmp = CC.Zero_c - Temp.w[0];        // 7Q9bit format; Current sign inversion!!!

  if((TIM_GetCapture1(PWMTimer) > TIM_GetCapture2(PWMTimer)) && (TIM_GetCapture1(PWMTimer) > TIM_GetCapture3(PWMTimer))){
          Ia_tmp = 0 - Ib_tmp - Ic_tmp;
  } else
  if((TIM_GetCapture2(PWMTimer) > TIM_GetCapture1(PWMTimer)) && (TIM_GetCapture2(PWMTimer) > TIM_GetCapture3(PWMTimer))){
          Ib_tmp = 0 - Ia_tmp - Ic_tmp;
  } else{
          Ic_tmp = 0 - Ia_tmp - Ib_tmp;
  }

  CC.Ia = Ia_tmp;
  CC.Ib = Ib_tmp;
  CC.Ic = Ic_tmp;

MonMass1[MMPtr] = ADC_Ia;//ADC_Ia;//ADC2->JDR1;//ADC2->JDR3;//Ia_tmp;////HC.Umot;//;
MonMass2[MMPtr] = ADC_Ib;
if(MMPtr < 19999) MMPtr++;


  // -------- The (a,b,c)->(a,b) projection (Clarke transformation) ---------
  // Ialfa = (2/3)Ia - (1/3)*(Ib-Ic)
  // Ibeta = (2/sqrt(3))*(Ib-Ic)
  // When Ialfa is superposed with Ia:
  // Ialfa = Ia
  // Ibeta = (1/sqrt(3))*(Ia+2*Ib)
  
  CC.Ialfa = Ia_tmp;
  
  Temp.dw = Ia_tmp + 2*Ib_tmp;		// 25Q9 bit format
  Temp.dw *= InvSqrt3;	                // 25Q9*1Q15 = 8Q24 bit format
  Temp.dw <<= 1;			// 8Q24 -> 7Q2 bit format
  CC.Ibeta = Temp.w[1];			// 7Q9 bit format
  
  //----------- The (a,b)->(d,q) projection (Park transformation) ------------
  // Id =  Ialfa*cos(Theta) + Ibeta*sin(Theta)
  // Iq = -Ialfa*sin(Theta) + Ibeta*cos(Theta)

  CC.sin = Sin_Tab_Read(MOT.RotorPosition);     // 

  Temp.w[0] = MOT.RotorPosition + 250;

  if(Temp.w[0] >= 1000)  Temp.w[0] -= 1000;
  else if(Temp.w[0] < 0) Temp.w[0] += 1000;
//ExtVar[1] = Temp.w[0];//<<<<<<<<<<<<<<<<<<<<<<<<<<
  CC.cos = Sin_Tab_Read(Temp.w[0]);     // 
  
  Temp.dw = (int32_t)CC.Ibeta*CC.sin + (int32_t)CC.Ialfa*CC.cos; // 7Q9*1Q15=8Q24 bit format
  Temp.dw <<= 1;						 // 8Q24 -> 7Q25 bit format
  CC.Id = Temp.w[1];						 // 7Q9 bit format

  Temp.dw = (int32_t)CC.Ibeta*CC.cos - (int32_t)CC.Ialfa*CC.sin; // 7Q9*1Q15=8Q24 bit format
  Temp.dw <<= 1;						 // 8Q24 -> 7Q25 bit format
  CC.Iq = Temp.w[1];						 // 7Q9 bit format
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
 ****************************************************************************/
void ADC_Zero(void)
{
uDW Sum0, Sum1, Sum2;
unsigned cntr;

 cntr = 8192;
 Sum0.dw = 0;
 Sum1.dw = 0;
 Sum2.dw = 0;
 ADCInt &= 0xFFFE;		        // Clear BIT1

    while(cntr){ //8192 samples
      cntr--;
      //KickDog();			// Kick the Dog
      while(!(ADCInt & Flag_CC)){} 	// Test BIT1; Wait for a new ADC convertion
      Flag_CC_Clear		        // Clear BIT1
      
      //Read ADC
      Sum0.dw += ADC_Ia;		// Read ADC in 7Q9 bit format
      Sum1.dw += ADC_Ib;		// Read ADC in 7Q9 bit format
      Sum2.dw += ADC_Ic;		// Read ADC in 7Q9 bit format
    }

    Sum0.dw >>= 8;			// 10Q22 -> 18Q14 bit format
    Sum0.dw *= HC.Iscaling;		// 18Q14*5Q11=7Q25 bit format
    Sum0.dw += 32768;                   // +0.5 - Rounding up
    CC.Zero_a = Sum0.w[1];		// 7Q9 bit format

    Sum1.dw >>= 8;			// 10Q22 -> 18Q14 bit format
    Sum1.dw *= HC.Iscaling;		// 18Q14*5Q11=7Q25 bit format
    Sum1.dw += 32768;                   // +0.5 - Rounding up
    CC.Zero_b = Sum1.w[1];		// 7Q9 bit format

    Sum2.dw >>= 8;			// 10Q22 -> 18Q14 bit format
    Sum2.dw *= HC.Iscaling;		// 18Q14*5Q11=7Q25 bit format
    Sum2.dw += 32768;                   // +0.5 - Rounding up
    CC.Zero_c = Sum2.w[1];		// 7Q9 bit format
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
 ****************************************************************************/
void ADC_Zero2(void)
{
uDW Sum0, Sum1, Sum2;
unsigned cntr;

  // Injected channal data offset registers
  ADC_SetInjectedOffset(ADC1,ADC_InjectedChannel_1,0);
  ADC_SetInjectedOffset(ADC2,ADC_InjectedChannel_1,0);
  ADC_SetInjectedOffset(ADC3,ADC_InjectedChannel_1,0);
  
 cntr = 8192;
 Sum0.dw = 0;
 Sum1.dw = 0;
 Sum2.dw = 0;
 ADCInt &= 0xFFFE;		        // Clear BIT1

    while(cntr){ //8192 samples
      cntr--;
      //KickDog();			// Kick the Dog
      while(!(ADCInt & 0x0001)){} 	// Test BIT1; Wait for a new ADC convertion
      ADCInt &= 0xFFFE;		        // Clear BIT1
      
      //Read ADC
      Sum0.dw += ADC_Ia;		// Read ADC in 7Q9 bit format
      Sum1.dw += ADC_Ib;		// Read ADC in 7Q9 bit format
      Sum2.dw += ADC_Ic;		// Read ADC in 7Q9 bit format
    }

    Sum0.dw <<= 3;			// 10Q22 -> 7Q25 bit format
    Sum0.dw += 32768;                   // +0.5 - Rounding up
    ADC_SetInjectedOffset(ADC1,ADC_InjectedChannel_1,Sum0.w[1]);
    
    Sum1.dw <<= 3;			// 10Q22 -> 7Q25 bit format
    Sum1.dw += 32768;                   // +0.5 - Rounding up
    ADC_SetInjectedOffset(ADC2,ADC_InjectedChannel_1,Sum1.w[1]);
    
    Sum2.dw <<= 3;			// 10Q22 -> 7Q25 bit format
    Sum2.dw += 32768;                   // +0.5 - Rounding up
    ADC_SetInjectedOffset(ADC3,ADC_InjectedChannel_1,Sum2.w[1]);
    
    
    CC.Zero_a = 0;		// 7Q9 bit format
    CC.Zero_b = 0;		// 7Q9 bit format
    CC.Zero_c = 0;		// 7Q9 bit format
}
/*****************************************************************************/




/**
* @brief Enables or disables the ADCx Option_2 configuration.
* @param ADCxDC2: The ADCxDC2 bit to be used.
* This parameter can be one of the following values:
* @arg SYSCFG_PMC_ADCxDC2: All ADCxDC2 bits
* @arg SYSCFG_PMC_ADC1DC2: ADC1DC2 bit
* @arg SYSCFG_PMC_ADC2DC2: ADC2DC2 bit
* @arg SYSCFG_PMC_ADC3DC2: ADC3DC2 bit
* @param NewState: new state of the ADCxDC2 bit.
* This parameter can be: ENABLE or DISABLE.
* @retval None
*/
void SET_ADCOption2 (uint32_t ADCxDC2, FunctionalState NewState)
{
/* Enable the SYSCFG clock*/
RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
if (NewState != DISABLE)
{
/* Set the ADCxDC2 */
SYSCFG->PMC |= (uint32_t)ADCxDC2;
}
else
{
/* Reset the ADCxDC2 */
SYSCFG->PMC &=(uint32_t)(~ADCxDC2);
}
}





/**
* @brief Enables or disables the ADC Option_1 configuration.
* @param NewState: new state of the ADCDC1 bit.
* This parameter can be: ENABLE or DISABLE.
* @retval None
*
* WARNING !
*  This bit can only be set at the following conditions:
*   - Prefetch must be OFF.
*   - VDD voltage ranges from 2.4 V to 3.6 V.
*   - This bit must not be set when the ADCxDC2 bit in SYSCFG_PMC register is set
*     ( see function SET_ADCOption2() )
*/
void SET_ADCOption1 (FunctionalState NewState)
{
  /* ENABLE PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // Old HAL !
  
  if (NewState != DISABLE)
  {
    // Disable "AN4073 option2"
    SET_ADCOption2 (SYSCFG_PMC_ADCxDC2, DISABLE);
    
    // Disable prefetch (ART - Adaptive real-time memory accelerator)
    FLASH_PrefetchBufferCmd(DISABLE); // Old HAL !
    
    // ENABLE "AN4073 option1"
    /* Set ADCDC1 bit */
    PWR->CR |= ((uint32_t)PWR_CR_ADCDC1);
  }
  else
  {
    // Disable "AN4073 option1"
    /* Reset ADCDC1 bit */
    PWR->CR &= (uint32_t)(~PWR_CR_ADCDC1);
  }
}




/*****************************************************************************
  * @brief  ADC1 configuration. Injected group - I_U at ADC123_IN10
  *                             Regular group - Temperature sensor at ADC1_IN18
  * @param  None
  * @retval None
 ****************************************************************************/
void ADC1_Config(void)
{
  ADC1->CR1 = Overrun_int_disable + _12_bits_resolution +\
              Scan_mode_enable + InjectedChanelsIntEn;

  ADC1->CR2 = /*RegTrigerDetectRisingEdge + T1CC3StartRegGroup +\*/
              InjTrigerDetectFallingEdge + T8CC4StartInjGroup +\
              DataAlignmentRight + SingleConversion + ADCOFF;

  ADC1->SMPR1 = CH18_SampleTime_84Cycles + CH10_SampleTime_3Cycles;
  ADC1->SMPR2 = 0;

  // Injected channal data offset registers
  ADC_SetInjectedOffset(ADC1,ADC_InjectedChannel_1,0);
  ADC_SetInjectedOffset(ADC1,ADC_InjectedChannel_2,0);
  ADC_SetInjectedOffset(ADC1,ADC_InjectedChannel_3,0);
  ADC_SetInjectedOffset(ADC1,ADC_InjectedChannel_4,0);
  
  /*ADC1->JOFR1 = 0;
  ADC1->JOFR2 = 0;
  ADC1->JOFR3 = 0;
  ADC1->JOFR4 = 0;*/
  
  // Watchdog higher threshold register
  ADC1->HTR = 4095;
  
  // Watchdog lower threshold register
  ADC1->LTR = 0;
  
  // Regular sequence registers
  ADC1->SQR1 = _1_Regular_Conversion;
  ADC1->SQR2 = 0;
  ADC1->SQR3 = CH18;
  
  // Injected sequence register
  // 3 conversions; chanel order - 10
  ADC1->JSQR = Inj_3Conv_CH10;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  ADC2 configuration. Injected group - I_V at ADC123_IN11. 
  *                             Regular group - U_SENS at ADC12_IN13.
  * @param  None
  * @retval None
 ****************************************************************************/
void ADC2_Config(void)
{
  ADC2->CR1 = Overrun_int_disable + _12_bits_resolution +\
              Scan_mode_enable + InjectedChanelsIntDis; //

  ADC2->CR2 = RegTrigerDetectDisable + InjTrigerDetectDisable +\
              DataAlignmentRight + SingleConversion + ADCOFF; //
  
  // Sampling time registers
  ADC2->SMPR1 = CH13_SampleTime_84Cycles + CH11_SampleTime_3Cycles;
  //ADC2->SMPR2 = CH9_SampleTime_84Cycles; // 

  // Injected channal data offset registers
  ADC_SetInjectedOffset(ADC2,ADC_InjectedChannel_1,0);
  ADC_SetInjectedOffset(ADC2,ADC_InjectedChannel_2,0);
  ADC_SetInjectedOffset(ADC2,ADC_InjectedChannel_3,0);
  ADC_SetInjectedOffset(ADC2,ADC_InjectedChannel_4,0);
  
  /*ADC2->JOFR1 = 0;
  ADC2->JOFR2 = 0;
  ADC2->JOFR3 = 0;
  ADC2->JOFR4 = 0;*/
  
  // Watchdog higher threshold register
  ADC2->HTR = 4095;
  
  // Watchdog lower threshold register
  ADC2->LTR = 0;
  
  // Regular sequence registers
  ADC2->SQR1 = _1_Regular_Conversion;
  ADC2->SQR2 = 0;
  ADC2->SQR3 = CH13;
  
  // Injected sequence register
  // 3 conversions; chanels order - CH11
  ADC2->JSQR = Inj_3Conv_CH11;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  ADC3 configuration. Injected group - I_W at ADC123_IN12.
  *                             Regular group - ADC123_IN3(not used)
  * @param  None
  * @retval None
 ****************************************************************************/
void ADC3_Config(void)
{
  ADC3->CR1 = Overrun_int_disable + _12_bits_resolution +\
              Scan_mode_enable + InjectedChanelsIntDis; //

  ADC3->CR2 = RegTrigerDetectDisable + InjTrigerDetectDisable +\
              DataAlignmentRight + SingleConversion + ADCOFF; //

  ADC3->SMPR1 = CH12_SampleTime_3Cycles;
  ADC3->SMPR2 = CH3_SampleTime_84Cycles; //

  // Injected channal data offset registers
  ADC_SetInjectedOffset(ADC3,ADC_InjectedChannel_1,0);
  ADC_SetInjectedOffset(ADC3,ADC_InjectedChannel_2,0);
  ADC_SetInjectedOffset(ADC3,ADC_InjectedChannel_3,0);
  ADC_SetInjectedOffset(ADC3,ADC_InjectedChannel_4,0);
  
  /*ADC3->JOFR1 = 0;
  ADC3->JOFR2 = 0;
  ADC3->JOFR3 = 0;
  ADC3->JOFR4 = 0;*/
  
  // Watchdog higher threshold register
  ADC3->HTR = 4095;
  
  // Watchdog lower threshold register
  ADC3->LTR = 0;
  
  // Regular sequence registers
  ADC3->SQR1 = _1_Regular_Conversion;
  ADC3->SQR2 = 0;
  ADC3->SQR3 = CH3;
  
  // Injected sequence register
  // 3 conversions; chanels order - CH12
  ADC3->JSQR = Inj_3Conv_CH12;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void ADC_Config(void)
{
ADC_CommonInitTypeDef ADC_CommonInitStructure;

  // Enable peripheral clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3, ENABLE);

  // ADC Common Init
  ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult_InjecSimult;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  // Enable both internal channels: ADC1_IN18 (temperature sensor) and ADC1_IN17 (VREFINT).
  ADC_TempSensorVrefintCmd(ENABLE);
  
  // ADC1 Injected channel 10, regular chanel - 18 configuration
  ADC1_Config();
  
  // ADC2 Injected channel 11, regular chanel - 9(not used)configuration
  ADC2_Config();
  
  // ADC3 Injected channels 12, regular chanel - 3(not used) configuration
  ADC3_Config();
  
  //SET_ADCOption1(ENABLE); // ADC noise reduction "option1"
  SET_ADCOption2(0x00070000, ENABLE);
  
  
  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);

  // Enable ADC2
  ADC_Cmd(ADC2, ENABLE);

  // Enable ADC3
  ADC_Cmd(ADC3, ENABLE);
  
  // ADC1 Injected chanels Software Conversion
//  ADC_SoftwareStartInjectedConv(ADC1);
    
  // ADC1 regular chanels Software Conversion
//  ADC_SoftwareStartConv(ADC1);
}
/******************************************************************************/
/******************************** END OF FILE *********************************/
