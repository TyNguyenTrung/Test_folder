/**
  ******************************************************************************
  * @file    Interrupt_Definitions.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2015-02-03
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
#include "Interrupt_Definitions.h"
#include "USART_functions.h"
#include "USART_functions_2.h"

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
void Interrupt_Definitions(void)
{
  /* 4 bits for pre-emption priority, 0 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  ADC_Int_Init();                       //1
  //USART1_Int_Init();
  //USART2_Int_Init();                    //14
  //USART6_Int_Init();
  USART3_Int_Init();                    //14
  //USART_DMA_Tx_NVIC_Init(COM3);
  //TIM1_TIM11_Int_Init();
  //TIM2_Int_Init();                      //3
  TIM6_Int_Init();                      //4
  //TIM7_Int_Init();
  TIM8_UP_TIM13_Int_Init();             //0
  TIM8_TRG_COM_TIM14_Int_Init();        //5
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void ADC_Int_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  

//  NVIC_StructInit(&NVIC_InitStructure);
  
  /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************/



/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void USART1_Int_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  

    /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void USART2_Int_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  

    /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void USART6_Int_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  

    /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void USART3_Int_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  

    /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;//?????
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //????????

  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void USART_DMA_Tx_NVIC_Init(COM_TypeDef COM)
{
  NVIC_InitTypeDef NVIC_InitStructure;
    
  // 4 bits for pre-emption priority, 0 bits for subpriority
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  // Enable the USARTx DMA Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART_DMA_TX_IRQn[COM];    // IRQ channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13; // IRQ channel priority (0..15)
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;     // IRQ channel subpriority (0..15)      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        // IRQ channel ENABLE or DISABLE
  NVIC_Init(&NVIC_InitStructure);                        // Apply changes 
  
  // Enable the USARTx Interrupt
  //NVIC_InitStructure.NVIC_IRQChannel = COM_IRQn[COM];
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);  
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void USART_DMA_Rx_NVIC_Init(COM_TypeDef COM)
{
  NVIC_InitTypeDef NVIC_InitStructure;
    
  // 4 bits for pre-emption priority, 0 bits for subpriority
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  // Enable the USARTx DMA Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART_DMA_RX_IRQn[COM];    // IRQ channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13; // IRQ channel priority (0..15)
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;     // IRQ channel subpriority (0..15)      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        // IRQ channel ENABLE or DISABLE
  NVIC_Init(&NVIC_InitStructure);                        // Apply changes 
 
  // Enable the USARTx Interrupt
  //NVIC_InitStructure.NVIC_IRQChannel = COM_IRQn[COM];
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);  
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void TIM1_TIM11_Int_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void TIM2_Int_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable the TIM2 Chanal 1 Capture and Chanal 2 Compare Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void TIM6_Int_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM7 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
/*void TIM7_Int_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable the TIM7 Update Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}*/
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void TIM8_UP_TIM13_Int_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  // Enable the TIM8 update Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void TIM8_TRG_COM_TIM14_Int_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  // Enable the TIM14 global Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************/
/******************************** END OF FILE *********************************/
