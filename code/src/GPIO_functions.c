/**
  ******************************************************************************
  * @file    GPIO_functions.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2015-03-04
  * @brief   
  ******************************************************************************
  * 
  *
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
#include "GPIO_functions.h"
#include "Parameters.h"
#include "Flags.h"
#include "TypesDef.h"

void Delay(__IO uint32_t nCount);


uint32_t IOInputReg, IOInputRegMask = 0, IOOutputReg;
uint32_t IOInputsActiveLevel = 0xFFFFFFFF, IOOutputsActiveLevel = 0xFFFFFFFF;
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
void GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /******** Configure the TIM5 Pins for Reference Clock Interface ********/
  // GPIOA Clocks enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // Configure PA0 & PA1 as timer1 channel A & B inputs
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // Alternate function source set
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
  
  
  /************* Configure the PA4 Pin as GPIO for test purpose **************/
  // GPIOA Periph clock enable
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);

  // Configure PA4 in output pushpull mode
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);*/
  

  /**************** Configure the Driver Enable Control Pin *****************/
  // GPIOA Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // Configure PA7 in output pushpull mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    
  /********************* Configure Voltage sensor ADCs Pin ********************/
  // GPIOB Periph clock enable
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
  /*
     Configure ADC2 Channel 9 as analog input
     ADC Channel 9 -> PB1 - Voltage sensor U_SENS
  */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);*/
    
    
  /****** Configure the TIM2 Pins for Position Hall Sensor Interface ******/  
  // GPIOA and GPIOB Clocks enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  // Set the default values
  GPIO_StructInit(&GPIO_InitStructure);
    
  // Configure PB8, PB9 and PB10 as Position Hall inputs
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // Alternate function set
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
  

  /*********************** Configure the LEDs Pins ***************************/
  // GPIOB Periph clock enable
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);

  // Configure PB1 in output pushpull mode
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);*/
  
  /************************ Configure the Alarm Pin *************************/
  // GPIOA Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // Configure PA6 in output pushpull mode
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);*/
    
    
  /*********** Configure the TIM8 Pins for PWM and Break Function ***********/
  // GPIOC Clocks enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  // GPIOC Configuration: Channel 1, 2, 3 as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
  
    
   /*************** Configure the S-SERVO/Ezi-SERVO2 SELECT Pin ***************/
  /* GPIOA Periph clock enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Configure PA6 in input nopull mode */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);*/

  /****************** Configure the Back EMF control Pin **********************/
  /* GPIOC Periph clock enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Configure PC0, PC1, PC2 and PC3 in output pushpull mode */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);*/
   
      
  /************************** RS485 direction Pin ****************************/
  // GPIOB Clocks enable
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  // Configure PB2 in input pullup mode
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);*/
    

  /******** Configure the TIM3 Pins for Incremental Encoder Interface ********/
  // GPIOB Clocks enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  // Configure PB4 & PB5 as timer3 channel A & B inputs
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // Alternate function source set
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
  
 
  /****************************** LED_RUN Pin *******************************/
  // GPIOB Clocks enable
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  // Configure PB9 in input pullup mode
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);*/
    
  /************************* OLED SSD1306 Pins ****************************/
  /* GPIOB Clocks enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
    
    /* I2C1 SDA and SCL configuration */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;// this defines the output type as open drain
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// this activates the pullup
    //GPIO_Init(GPIOB, &GPIO_InitStructure);*/
    
    // Alternate function source set
    /*GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);*/
    
    
  /********************** Configure the OLED Reset Pin *********************/
  /* GPIOB Clocks enable */
    //RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE);
  
  /* Configure PB5 in output pushpull mode */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);*/


  /******************* I2C1 interface's pins configuration *******************/
  /* Enable GPIOB, clock */
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* Configure PB.07,08 as I2C1 pins */
  /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);*/
  
  /* Alternate function source set */
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
  
  /************* Configure the Reference Resolution Switch Pins ***************/
  /* GPIOB Clocks enable */
    //RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE);

  /* Configure PB12, PB13, PB14 and PB15 in input pullup mode */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);*/
    

/***************** Configure the Motor Current ADCs Pins *********************/
  // GPIOC Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
  /*
     Configure Port C pins 0, 1 , 2 and 3 as analog inputs
     ADC Channel 10 -> PC0 - U_PHASE_SENSOR
     ADC Channel 11 -> PC1 - V_PHASE_SENSOR
     ADC Channel 12 -> PC2 - W_PHASE_SENSOR
     ADC Channel 13 -> PC3 - V_DC
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    
  
  /****************** Configure the RJ45 LEDs control Pins *******************/
  /* GPIOC Periph clock enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure PC8 in output pushpull mode */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);*/
    
  /********************* Configure the IO Input Pins **********************/
  /* GPIOC Periph clock enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure PC8 & PC9 in input mode */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);*/
    
  /* GPIOD Periph clock enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
  /* Configure PD7 in input mode */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);*/
    
  /* Configure PD2, PD4 & PD11 in output mode */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);*/
    
  /* GPIOE Periph clock enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
  /* Configure PE2 to PE6 in input mode */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);*/
    

  /********************** Configure Control Joystick Pins ********************/
  // GPIOB Clocks enable
    //RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE);
  
  // Configure PC11 in input mode
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);*/
    
    // GPIOC Clocks enable
    //RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC , ENABLE);
  
  // Configure PC12, PC13, PC14, PC15 in input mode
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                                              GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);*/

  /******************* Configure the In-Position Signal Pin *******************/
  /* GPIOC Periph clock enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure PC5 in output pushpull mode */
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);*/
    
  /******** Configure the TIM3 Pins for Incremental Encoder Interface ********/  
  /* Enable GPIOC, clock */
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure PC.06, 07 as encoder input */
  /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);*/
  
  /* Alternate function set */
  /*GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);*/

    
  /*********************** Configure GPIO9 as MCO2 Pin ***********************/
  // GPIOC Clocks enable
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);

    /*GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init(GPIOC, &GPIO_InitStructure);*/
    
  /* Alternate function source set */
    //GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_MCO);
    
   
  /****************** Configure the RS485 comunication Pins ******************/
  /* GPIOC Periph clock enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure PC11 in output pushpull mode */
  /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);*/
    
  /********************* Configure the Break control Pin *********************/
  /* GPIOC Periph clock enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure PC11 in output pushpull mode */
  /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);*/
  
  /********************* Configure the Driver Enable Pin *********************/
  // GPIOC Periph clock enable
  /*  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  // Configure PC11 in output pushpull mode
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);*/
  
  /************************* Motor DB Select Pins ****************************/
  /* GPIOC Clocks enable */
  //RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC , ENABLE);
  
  /* Configure PC13, PC14, PC15 in input pullup mode */
  /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);*/
  
  
  /************************** Servo Enable Input Pin *************************/
  // GPIOD Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
  // Configure PD0 in input mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
  
    
  /************************** Alarm Clear Input Pin **************************/
  // GPIOD Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
  // Configure PD1 in input mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

  
  /*********************** Configure the Alarm Out Pin ***********************/
  // GPIOD Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // Configure PG6 in output pushpull mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    
  /******************** Configure the In-Position Out Pin ********************/
  // GPIOD Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // Configure PG7 in output pushpull mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    
  /******************************* LED_ERR Pin *******************************/
  // GPIOG Clocks enable
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

  // Configure PG3 in input pullup mode
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOG, &GPIO_InitStructure);*/
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void TCN75_CommunicationDir(GPIOMode_TypeDef Dir)	
{	
   GPIO_InitTypeDef  GPIO_InitStructure;
   
   // 0 = input	1 = output
   //GpioMuxRegs.GPADIR.bit.GPIOA13 = Dir;
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = Dir;//GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
*****************************************************************************/
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
/*****************************************************************************/
/******************************* END OF FILE **********************************/
