  /**
  ******************************************************************************
  * @file    main.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2018-04-30
  * @brief   Main program body
  * @compiler IAR EVARM 7.50.2.
  ******************************************************************************
  *
  *   Processor - STM32F446RC
  *
  *   FLASH Memory - 512KB
  *                  Sector 0  0x0800 0000 - 0x0800 3FFF 16 Kbyte
  *                  Sector 1  0x0800 4000 - 0x0800 7FFF 16 Kbyte
  *                  Sector 2  0x0800 8000 - 0x0800 BFFF 16 Kbyte
  *    Bank 1        Sector 3  0x0800 C000 - 0x0800 FFFF 16 Kbyte
  *                  Sector 4  0x0801 0000 - 0x0801 FFFF 64 Kbyte
  *                  Sector 5  0x0802 0000 - 0x0803 FFFF 128 Kbyte
  *                  Sector 6  0x0804 0000 - 0x0805 FFFF 128 Kbyte
  *                  Sector 7  0x0806 0000 - 0x0807 FFFF 128 Kbyte
  *    -----------------------------------------------------------------------
  *
  *   RAM Memory - 128KB
  *   SRAM1          0x2000 0000 - 0x2001 BFFF          112 Kbyte
  *   SRAM2          0x2001 C000 - 0x2001 FFFF          16  Kbyte
  *   
  *                                             
  *                     
  *     DB resides in Sector 1 & 2 !!!       
  *     User program starts from Sector 3 !!!        
  *
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/ 
#include "Main.h"
#include "PWM_Functions.h"
#include "SIN_Tab.h"
#include "TimersFunctions.h"
#include "ADC_functions.h"
#include "Background.h"
#include "SysMonitor.h"
#include "Parameters.h"
#include "HardwareConfiguration.h"
#include "CurrentControl.h"
#include "eeprom.h"
#include "Command_functions.h"
#include "REFGenFunctions.h"
#include "SSD1306.h"
#include "Velocity_Measure.h"
#include "PositionVelocityControl.h"
#include "ADS101x_IIC_functions.h"
#include "PCA9632_IIC_functions.h"

#include "ssd1306_test.h"
#include "Config.h"

#include "USART_functions.h"
#include "USART_functions_2.h"

    #include "arm_itm.h"
    
extern s16 UqRef;
//int32_t Hoang = 10;
//int32_t Hoang2 = 1;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/*****************************************************************************
  * @brief  Main program
  * @param  None
  * @retval None
*****************************************************************************/
int main(void)
{
  // Hoang = 20;
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
  */

RCC_ClocksTypeDef tRCC;

//int16_t Temp;
__IO uint32_t i = 0, Time_mS; 
  
  // Sets the BOR Level = OB_BOR_LEVEL3: Supply voltage ranges from 2.7 to 3.6 V
  if (FLASH_OB_GetBOR() != OB_BOR_LEVEL3) FLASH_OB_BORConfig(OB_BOR_LEVEL3);

  uint32_t cnt=0xFFFFF; while(cnt--); // Protect EEPROM  ~ 0.2s
  
  /*--------------------------- GPIO Configuration --------------------------*/
    GPIO_Config();
  
  //--------------- Delay Init ----------------
    RCC_GetClocksFreq(&tRCC); 
    delay_init(tRCC.HCLK_Frequency);

  /*------------------------- Alarm Initialization --------------------------*/
    AlarmInit();                        // It MUST BE After GPIO configuration
    
  /*----------------- EEPROM Emulation Library Configuration ----------------*/
    EnumToArray();
    FLASH_Unlock();                     // Unlock the Flash Program Erase controller
    if(FLASH_COMPLETE != EE_Init()) AL.Source = 12;                          // EEPROM Init
    FLASH_Lock();                       // Lock the Flash Program Erase controller
    
  /*--------------------------- Parameters Upload ---------------------------*/
    ParameterDefault();
    //ParametersLoad(DriverParam);        // Read Saved Driver Parameters
    //ParametersLoad(MotorParam);         // Read Saved Motor Parameters

  /*====================== User's Modules Configuration =====================*/    
    HardwareConfiguration();		// Calculate Current and Voltage Scaling Coefficients
    CommandFunctions_Init();		// Command Functions Initialization
    CurrentControlInit();		// ReInit of the Current Controller

    VelocityMeasInit();
    PositionVelocityControlInit();
    SinAngleInit(def_HALL);

  /*------------------------ Interrupts Configuration -----------------------*/
    Interrupt_Definitions(); 

  /*--------------------------- ADC Configuration ---------------------------*/
    ADC_Config();
  

  /*------------------------- Timer2 Configuration --------------------------*/
    TIM2_Config();

  /*------------------------- Timer3 Configuration --------------------------*/
    TIM3_Config();

  //------------------------- Timer5 Configuration --------------------------
    TIM5_Config();
    
  /*------------------------- Timer8 Configuration --------------------------*/
    TIM8_Config();
  
  /*------------------------- Timer14 Configuration -------------------------*/
    TIM14_Config();
    
  /*-------------------- Serial interface Configuration ---------------------*/
    USART_Config();
    
    TIM_Cmd(TIM8, ENABLE);          // Enable Timer8, PWMs & Timer8 interrupts
    
    Driver_Enable(); 		    // Driver Enable
    delay_us(20000);            // ADC and PCA start up Delay  ~ 20ms
    ADC_Zero();                 // Zero Level Definition of Phase A, B and C

    // TIM14 enable counter and Interrupts
    TIM_Cmd(TIM14, ENABLE); 
  
    while(1){
     Background();
     //Hoang = 20;
     // Delay(50000); 
     //USART_WaitCharDataSend(USART2, 0x0D);
     //USART_WaitCharDataSend(USART2, 0x0A);
     //USART_WaitCharDataSend(USART2, 2 + 0x30);
     
     //USART_WaitCharDataSend(USART2, '1');
     //USART_WaitCharDataSend(USART2, '1');
     //Delay(500000); 
     //USART_SendData(USART2, 1+ 0x30);  // +0x30 change int to char
      //USART_SendData(USART2, 0x01);
     //while(!(USART2->SR & USART_SR_TXE));
     //USART2->DR = '1'; // Send 'B'
      //Hoang++;
      //ITM_EVENT16_WITH_PC(1, Hoang);
      // Delay(500000); 
      //Hoang2++;
      //ITM_EVENT16_WITH_PC(2, Hoang2);
     
    }
} 
/******************************************************************************/





#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif
/******************************** END OF FILE *********************************/

