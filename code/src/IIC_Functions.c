/**
  ******************************************************************************
  * @file    IIC_Functions.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2015-04-06
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
#include "stm32f2xx.h"
#include "GPIO_functions.h"
#include "IIC_Functions.h"
#include "delay.h"
#include "SSD1306.h"
#include "LIB_Config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_TIMEOUT_MAX 2000000
//#define M24512 1
#define SLAVE_ADDRESS 0x78/*slave addresses with write*/
//#define READ_ADDRESS 0x79/*slave addresses with read*/
#define Timed(x) Timeout = 0xFFFF; while (x) { if (Timeout-- == 0) goto errReturn;}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
uint8_t ssd1306_write_byte(uint8_t Data, uint8_t Cmd)
    {
     
    uint32_t timeout = I2C_TIMEOUT_MAX;
    
    // Generate the Start Condition
    I2C_GenerateSTART(I2C1, ENABLE);
     
    // Test on I2C1 EV5, Start transmitted successfully and clear it
    timeout = I2C_TIMEOUT_MAX; // Initialize timeout value
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
      // If the timeout delay is exeeded, exit with error code
      if ((timeout--) == 0) return 0xFF;
    }
     
    // Send Memory device slave Address for write
    I2C_Send7bitAddress(I2C1,SLAVE_ADDRESS,I2C_Direction_Transmitter);
     
    // Test on I2C1 EV6 and clear it
    timeout = I2C_TIMEOUT_MAX; // Initialize timeout value
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
    // If the timeout delay is exeeded, exit with error code
    if ((timeout--) == 0) return 0xFF;
    }
     
    if(Cmd)     I2C_SendData(I2C1, 0x40);
    else        I2C_SendData(I2C1, 0x0);
      
    // Test on I2C1 EV8 and clear it
    timeout = I2C_TIMEOUT_MAX; // Initialize timeout value
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
      // If the timeout delay is exeeded, exit with error code
      if ((timeout--) == 0) return 0xFF;
    }
     
    // Send Data
    I2C_SendData(I2C1, Data);
     
    // Test on I2C1 EV8 and clear it
    timeout = I2C_TIMEOUT_MAX; // Initialize timeout value
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
      // If the timeout delay is exeeded, exit with error code
      if ((timeout--) == 0) return 0xFF;
    }
     
    // Send I2C1 STOP Condition
    I2C_GenerateSTOP(I2C1, ENABLE);
     
    // If operation is OK, return 0
    return 0;
    }
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void I2C_Config(I2C_TypeDef* I2Cx, uint32_t ClockSpeed, uint16_t OwnAddress)
{
GPIO_InitTypeDef GPIO_InitStructure;
I2C_InitTypeDef I2C_InitStructure;

  // GPIOB Periph clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
    
  // Configure I2C clock and GPIO
  GPIO_StructInit(&GPIO_InitStructure);

  if (I2Cx == I2C1){
    /* I2C1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    
    // I2C1 SDA and SCL configuration
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;// this defines the output type as open drain
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// this activates the pullup
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // Alternate function source set
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);*/
  
    /* I2C1 Reset */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
  }
  else {
    /* I2C2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    
    /* I2C1 SDA and SCL configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* I2C2 Reset */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
  }
  
  // Configure I2Cx
  I2C_DeInit(I2Cx);
 
  /* Configure I2Cx */
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = OwnAddress;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
  I2C_Init(I2Cx, &I2C_InitStructure);
  
  /* Enable the I2C peripheral */
  I2C_Cmd(I2Cx, ENABLE);
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
