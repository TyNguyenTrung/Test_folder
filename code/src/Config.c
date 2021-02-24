/**
  ******************************************************************************
  * @file    xxx.c 
  * @author  Waveshare Team
  * @version 
  * @date    xx-xx-2014
  * @brief   xxxxx.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm324xg_bldc_hal.h"
#include "LIB_Config.h"
#include "Config.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void device_init(void);
static void driver_init(void);
static void port_init(void);
static void TimerInit(void);

#ifdef INTERFACE_4WIRE_SPI
static void spi2_init(void);
#endif
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  System initialization.
  * @param  None
  * @retval  None
  */
void OLED_Config(void)
{
    RCC_ClocksTypeDef tRCC;
    
    //Enable HSE clock
    RCC_HSEConfig(RCC_HSE_ON);
    //Wait for clock to stabilize
    while (!RCC_WaitForHSEStartUp());
    
    SystemCoreClockUpdate();
    RCC_GetClocksFreq(&tRCC); 
    delay_init(tRCC.HCLK_Frequency);
    
    device_init();
    driver_init();
    
    TimerInit();
}

void TimerInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
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
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
  //---------------------------------
 
  /* TIM14 enable counter */
  //TIM_Cmd(TIM14, ENABLE);
}
/*
void TIM14_IRQHandler(void)
{
  static uint32_t Cnt_mS=0, Cnt_S=0;
  
  if (TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
 
    if (Cnt_mS < 1000) Cnt_mS++;
    else { 
      Cnt_mS = 0; 
      Cnt_S++;
      LEDToggle(LED_G);
    }
  }
}
*/

/**
 * @brief 
 * @param 
 * @retval 
 */
static void device_init(void)
{
	port_init();
#ifdef INTERFACE_4WIRE_SPI
	spi2_init();
#endif
	iic_init();
}

/**
  * @brief  driver initialization.
  * @param  None
  * @retval None
  */
static void driver_init(void)
{
	ssd1306_init();
}


static void port_init(void) 
{
	GPIO_InitTypeDef tGPIO;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	/*----------------------------------------------------------------------------------*/

	//SPI
#ifdef INTERFACE_4WIRE_SPI
	tGPIO.GPIO_Pin = SSD1306_CLK_PIN | SSD1306_DIN_PIN;	 //SCK  MISO	MOSI		 		 
	tGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	tGPIO.GPIO_Mode = GPIO_Mode_AF_PP; 	
	GPIO_Init(GPIOB, &tGPIO);
	
#elif defined(INTERFACE_3WIRE_SPI)
	tGPIO.GPIO_Pin = SSD1306_CLK_PIN;	 //SCK		 		 
	tGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	tGPIO.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_Init(GPIOB, &tGPIO);
	
	tGPIO.GPIO_Pin = SSD1306_DIN_PIN;	 //MOSI		 		 
	tGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	tGPIO.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_Init(GPIOB, &tGPIO);
	
#endif
	
	
	/*----------------------------------------------------------------------------------*/
        
	
	//SSD1306 OLED
/*       
	tGPIO.GPIO_Pin = SSD1306_CS_PIN;     // CS	
        tGPIO.GPIO_Mode = GPIO_Mode_OUT;
	tGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	tGPIO.GPIO_OType = GPIO_OType_PP;
        tGPIO.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOB, &tGPIO);
 	
	tGPIO.GPIO_Pin = SSD1306_RES_PIN | SSD1306_DC_PIN;     // RES D/C 				 
        tGPIO.GPIO_Mode = GPIO_Mode_OUT;
	tGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	tGPIO.GPIO_OType = GPIO_OType_PP;
        tGPIO.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOB, &tGPIO);
*/	
	
	/*----------------------------------------------------------------------------------*/
	//I2C
#ifdef INTERFACE_IIC
        tGPIO.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN;				 
        tGPIO.GPIO_Mode = GPIO_Mode_OUT;
	tGPIO.GPIO_Speed = GPIO_Speed_2MHz;
	tGPIO.GPIO_OType = GPIO_OType_PP;
        tGPIO.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOB, &tGPIO);
#endif
	/*----------------------------------------------------------------------------------*/
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*
void SysTick_Handler(void)
{
	;
}
*/


/**
  * @brief  SPI initialization.
  * @param  None
  * @retval None
  */
#ifdef INTERFACE_4WIRE_SPI
static void spi2_init(void)
{
	SPI_InitTypeDef tSPI;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	tSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	tSPI.SPI_Mode = SPI_Mode_Master;		
	tSPI.SPI_DataSize = SPI_DataSize_8b;		
	tSPI.SPI_CPOL = SPI_CPOL_High;		
	tSPI.SPI_CPHA = SPI_CPHA_2Edge;	
	tSPI.SPI_NSS = SPI_NSS_Soft;		
	tSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		
	tSPI.SPI_FirstBit = SPI_FirstBit_MSB;	
	tSPI.SPI_CRCPolynomial = 7;	
	SPI_Init(SPI2, &tSPI);  
 
	SPI_Cmd(SPI2, ENABLE); 
}
#endif



/*-------------------------------END OF FILE-------------------------------*/
