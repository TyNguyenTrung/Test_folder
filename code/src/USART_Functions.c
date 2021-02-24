/**
  ******************************************************************************
  * @file    USART_functions.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2017-02-28
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
#include "USART_functions.h"
#include "USART_functions_2.h"
#include "TimersFunctions.h"
#include <stdio.h>
#include "stm324xg_bldc_hal.h"
#include "ring_buff.h"
    
unsigned long BaudRate;

#define BaudRate_COM6 115200
#define BaudRate_COM1 115200
#define BaudRate_COM2 115200        // 460800       
#define BaudRate_COM3 460800

#define DataBufferSize  4
int16_t ExtVar[DataBufferSize];
int16_t USART_ExtVar[DataBufferSize] = {65,66,67,68};
int16_t USART_ExtVarSh[DataBufferSize] = {0,0,0,0};

int8_t RecDataBuff[PACKET_RECV_SIZE];

volatile uint16_t EZ_COMStatus [COMn];

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


USART_TypeDef* COM_USART[COMn] = {EZ_COM1, EZ_COM2, EZ_COM3, EZ_COM4, EZ_COM5, EZ_COM6}; 
GPIO_TypeDef* COM_TX_PORT[COMn] = {EZ_COM1_TX_GPIO_PORT, EZ_COM2_TX_GPIO_PORT, EZ_COM3_TX_GPIO_PORT, EZ_COM4_TX_GPIO_PORT, EZ_COM5_TX_GPIO_PORT, EZ_COM6_TX_GPIO_PORT};
GPIO_TypeDef* COM_RX_PORT[COMn] = {EZ_COM1_RX_GPIO_PORT, EZ_COM2_RX_GPIO_PORT, EZ_COM3_RX_GPIO_PORT, EZ_COM4_RX_GPIO_PORT, EZ_COM5_RX_GPIO_PORT, EZ_COM6_RX_GPIO_PORT};
const uint32_t COM_USART_CLK[COMn] = {EZ_COM1_CLK, EZ_COM2_CLK, EZ_COM3_CLK, EZ_COM4_CLK, EZ_COM5_CLK, EZ_COM6_CLK};
const uint32_t COM_TX_PORT_CLK[COMn] = {EZ_COM1_TX_GPIO_CLK, EZ_COM2_TX_GPIO_CLK, EZ_COM3_TX_GPIO_CLK, EZ_COM4_TX_GPIO_CLK, EZ_COM5_TX_GPIO_CLK, EZ_COM6_TX_GPIO_CLK};
const uint32_t COM_RX_PORT_CLK[COMn] = {EZ_COM1_RX_GPIO_CLK, EZ_COM2_RX_GPIO_CLK, EZ_COM3_RX_GPIO_CLK, EZ_COM4_RX_GPIO_CLK, EZ_COM5_RX_GPIO_CLK, EZ_COM6_RX_GPIO_CLK};
const uint16_t COM_TX_PIN[COMn] = {EZ_COM1_TX_PIN, EZ_COM2_TX_PIN, EZ_COM3_TX_PIN, EZ_COM4_TX_PIN, EZ_COM5_TX_PIN, EZ_COM6_TX_PIN};
const uint16_t COM_RX_PIN[COMn] = {EZ_COM1_RX_PIN, EZ_COM2_RX_PIN, EZ_COM3_RX_PIN, EZ_COM4_RX_PIN, EZ_COM5_RX_PIN, EZ_COM6_RX_PIN};
const uint8_t COM_TX_PIN_SOURCE[COMn] = {EZ_COM1_TX_SOURCE, EZ_COM2_TX_SOURCE, EZ_COM3_TX_SOURCE, EZ_COM4_TX_SOURCE, EZ_COM5_TX_SOURCE, EZ_COM6_TX_SOURCE};
const uint8_t COM_RX_PIN_SOURCE[COMn] = {EZ_COM1_RX_SOURCE, EZ_COM2_RX_SOURCE, EZ_COM3_RX_SOURCE, EZ_COM4_RX_SOURCE, EZ_COM5_RX_SOURCE, EZ_COM6_RX_SOURCE};
const uint8_t COM_TX_AF[COMn] = {EZ_COM1_TX_AF, EZ_COM2_TX_AF, EZ_COM3_TX_AF, EZ_COM4_TX_AF, EZ_COM5_TX_AF, EZ_COM6_TX_AF};
const uint8_t COM_RX_AF[COMn] = {EZ_COM1_RX_AF, EZ_COM2_RX_AF, EZ_COM3_RX_AF, EZ_COM4_RX_AF, EZ_COM5_RX_AF, EZ_COM6_RX_AF};

USART_InitTypeDef USART_InitStructure;


uint32_t  USART_DMA_CLK[COMn] = {USART1_DMA_CLK, USART2_DMA_CLK, USART3_DMA_CLK, USART4_DMA_CLK, USART5_DMA_CLK, USART6_DMA_CLK};
uint32_t  USART_DR_ADDRESS[COMn] = {USART1_DR_ADDRESS, USART2_DR_ADDRESS, USART3_DR_ADDRESS, USART4_DR_ADDRESS, USART5_DR_ADDRESS, USART6_DR_ADDRESS};

DMA_InitTypeDef DMA_InitStructure[COMn];

uint32_t  USART_TX_DMA_CHANNEL[COMn] = {USART1_TX_DMA_CHANNEL, USART2_TX_DMA_CHANNEL, USART3_TX_DMA_CHANNEL, USART4_TX_DMA_CHANNEL, USART5_TX_DMA_CHANNEL, USART6_TX_DMA_CHANNEL}; 

DMA_TypeDef*    USART_DMA[COMn] = {USART1_DMA, USART2_DMA, USART3_DMA, USART4_DMA, USART5_DMA, USART6_DMA};

DMA_Stream_TypeDef* USART_TX_DMA_STREAM[COMn] = {USART1_TX_DMA_STREAM, USART2_TX_DMA_STREAM, USART3_TX_DMA_STREAM, USART4_TX_DMA_STREAM, USART5_TX_DMA_STREAM, USART6_TX_DMA_STREAM};
uint32_t  USART_TX_DMA_FLAG_FEIF[COMn]  = {USART1_TX_DMA_FLAG_FEIF,  USART2_TX_DMA_FLAG_FEIF, USART3_TX_DMA_FLAG_FEIF, USART4_TX_DMA_FLAG_FEIF, USART5_TX_DMA_FLAG_FEIF, USART6_TX_DMA_FLAG_FEIF};
uint32_t  USART_TX_DMA_FLAG_DMEIF[COMn] = {USART1_TX_DMA_FLAG_DMEIF, USART2_TX_DMA_FLAG_DMEIF, USART3_TX_DMA_FLAG_DMEIF, USART4_TX_DMA_FLAG_DMEIF, USART5_TX_DMA_FLAG_DMEIF, USART6_TX_DMA_FLAG_DMEIF};
uint32_t  USART_TX_DMA_FLAG_TEIF[COMn]  = {USART1_TX_DMA_FLAG_TEIF,  USART2_TX_DMA_FLAG_TEIF,  USART3_TX_DMA_FLAG_TEIF,  USART4_TX_DMA_FLAG_TEIF,  USART5_TX_DMA_FLAG_TEIF, USART6_TX_DMA_FLAG_TEIF};
uint32_t  USART_TX_DMA_FLAG_HTIF[COMn]  = {USART1_TX_DMA_FLAG_HTIF,  USART2_TX_DMA_FLAG_HTIF,  USART3_TX_DMA_FLAG_HTIF,  USART4_TX_DMA_FLAG_HTIF,  USART5_TX_DMA_FLAG_HTIF, USART6_TX_DMA_FLAG_HTIF};
uint32_t  USART_TX_DMA_FLAG_TCIF[COMn]  = {USART1_TX_DMA_FLAG_TCIF,  USART2_TX_DMA_FLAG_TCIF,  USART3_TX_DMA_FLAG_TCIF,  USART4_TX_DMA_FLAG_TCIF,  USART5_TX_DMA_FLAG_TCIF, USART6_TX_DMA_FLAG_TCIF};

uint32_t  USART_RX_DMA_CHANNEL[COMn]    = {USART1_RX_DMA_CHANNEL, USART2_RX_DMA_CHANNEL, USART3_RX_DMA_CHANNEL, USART4_RX_DMA_CHANNEL, USART5_RX_DMA_CHANNEL, USART6_RX_DMA_CHANNEL};
DMA_Stream_TypeDef* USART_RX_DMA_STREAM[COMn] = {USART1_RX_DMA_STREAM, USART2_RX_DMA_STREAM, USART3_RX_DMA_STREAM, USART4_RX_DMA_STREAM, USART5_RX_DMA_STREAM, USART6_RX_DMA_STREAM};
uint32_t  USART_RX_DMA_FLAG_FEIF[COMn]  = {USART1_RX_DMA_FLAG_FEIF,  USART2_RX_DMA_FLAG_FEIF,  USART3_RX_DMA_FLAG_FEIF,  USART4_RX_DMA_FLAG_FEIF,  USART5_RX_DMA_FLAG_FEIF,  USART6_RX_DMA_FLAG_FEIF};
uint32_t  USART_RX_DMA_FLAG_DMEIF[COMn] = {USART1_RX_DMA_FLAG_DMEIF, USART2_RX_DMA_FLAG_DMEIF, USART3_RX_DMA_FLAG_DMEIF, USART4_RX_DMA_FLAG_DMEIF, USART5_RX_DMA_FLAG_DMEIF, USART6_RX_DMA_FLAG_DMEIF};
uint32_t  USART_RX_DMA_FLAG_TEIF[COMn]  = {USART1_RX_DMA_FLAG_TEIF,  USART2_RX_DMA_FLAG_TEIF,  USART3_RX_DMA_FLAG_TEIF,  USART4_RX_DMA_FLAG_TEIF,  USART5_RX_DMA_FLAG_TEIF,  USART6_RX_DMA_FLAG_TEIF};
uint32_t  USART_RX_DMA_FLAG_HTIF[COMn]  = {USART1_RX_DMA_FLAG_HTIF,  USART2_RX_DMA_FLAG_HTIF,  USART3_RX_DMA_FLAG_HTIF,  USART4_RX_DMA_FLAG_HTIF,  USART5_RX_DMA_FLAG_HTIF,  USART6_RX_DMA_FLAG_HTIF};
uint32_t  USART_RX_DMA_FLAG_TCIF[COMn]  = {USART1_RX_DMA_FLAG_TCIF,  USART2_RX_DMA_FLAG_TCIF,  USART3_RX_DMA_FLAG_TCIF,  USART4_RX_DMA_FLAG_TCIF,  USART5_RX_DMA_FLAG_TCIF,  USART6_RX_DMA_FLAG_TCIF};

uint8_t   USART_DMA_TX_IRQn[COMn]       = {USART1_DMA_TX_IRQn,       USART2_DMA_TX_IRQn,       USART3_DMA_TX_IRQn,       USART4_DMA_TX_IRQn,       USART5_DMA_TX_IRQn,       USART6_DMA_TX_IRQn};
uint8_t   USART_DMA_RX_IRQn[COMn]       = {USART1_DMA_RX_IRQn,       USART2_DMA_RX_IRQn,       USART3_DMA_RX_IRQn,       USART4_DMA_RX_IRQn,       USART5_DMA_RX_IRQn,       USART6_DMA_RX_IRQn};

/* Private function prototypes -----------------------------------------------*/
void COM_DMA_Init_Linear(COM_TypeDef COM, DMA_InitTypeDef *DMA_InitStructure);
void USART1_Config(void);
void COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct);

/* Private functions ---------------------------------------------------------*/


/*****************************************************************************
  * @brief  USART_DataRead
  * @param  None
  * @retval None
  ***************************************************************************/
void USART_DataRead(void)
{
uint8_t Data;
  /* Interrupt source check */
 // if(USART_GetITStatus(COM1, USART_IT_RXNE) != RESET){
    /* Read one byte from the receive data register */
      Data = (USART_ReceiveData(USART1) & 0x7F);
      /* return received char */
      USART_SendData(USART1, Data);     
//  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  
  *         
  *         
  * @retval None
  ***************************************************************************/
void DMA_USART_TX_Flags_Clear(COM_TypeDef COMx)
{
    // Clear all DMA Tx Streams flags
    DMA_ClearFlag(USART_TX_DMA_STREAM[COMx], USART_TX_DMA_FLAG_FEIF[COMx] | 
                  USART_TX_DMA_FLAG_DMEIF[COMx] | USART_TX_DMA_FLAG_TEIF[COMx] |
                  USART_TX_DMA_FLAG_HTIF[COMx] | USART_TX_DMA_FLAG_TCIF[COMx]); 
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
  ***************************************************************************/
void COM_DMA_Init_Linear(COM_TypeDef COM, DMA_InitTypeDef *DMA_InitStructure)
{
  // Configure DMA controller to manage USART TX and RX DMA request ------------
  
  // Enable the DMA clock
  RCC_AHB1PeriphClockCmd(USART_DMA_CLK[COM], ENABLE);

  // Here only the unchanged parameters of the DMA initialization structure are
  // configured. During the program operation, the DMA will be configured with
  // different parameters according to the operation phase
  DMA_InitStructure->DMA_PeripheralBaseAddr = USART_DR_ADDRESS[COM];
  DMA_InitStructure->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure->DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure->DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure->DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure->DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure->DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure->DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure->DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  USART x Transmit linear data buffer (array) using DMA
  * @param  COM_TypeDef COMx   : port
  *         uint8_t *TxBuf     : pointer to buffer
  *         uint16_t TxBufSize : data length
  * @retval None
  ***************************************************************************/
void Transmit_USART_dma(COM_TypeDef COMx, const uint8_t *TxBuf, uint16_t TxBufSize)
{  
    // The software must wait until TC=1. The TC flag remains cleared during all data
    // transfers and it is set by hardware at the last frame’s end of transmission
    
    if(USART_GetFlagStatus(COM_USART[COMx], USART_FLAG_TC) == RESET) return;
    
  //TimeOut = USER_TIMEOUT;
    /*while ((USART_GetFlagStatus(COM_USART[COMx], USART_FLAG_TC) == RESET)&&(TimeOut != 0x00));
    if(TimeOut == 0){
      __no_operation(); //Timeout handler
    }*/
  
    // Clear all DMA Tx Streams flags
    DMA_ClearFlag(USART_TX_DMA_STREAM[COMx], USART_TX_DMA_FLAG_FEIF[COMx] |
                  USART_TX_DMA_FLAG_DMEIF[COMx] | USART_TX_DMA_FLAG_TEIF[COMx] | 
                  USART_TX_DMA_FLAG_HTIF[COMx] | USART_TX_DMA_FLAG_TCIF[COMx]);    

    // Prepare the DMA to transfer the transaction command (Xbytes) from the memory to the USART
    DMA_DeInit(USART_TX_DMA_STREAM[COMx]);
    DMA_InitStructure[COMx].DMA_Channel = USART_TX_DMA_CHANNEL[COMx];
    DMA_InitStructure[COMx].DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure[COMx].DMA_Memory0BaseAddr = (uint32_t)TxBuf;
    DMA_InitStructure[COMx].DMA_BufferSize = (uint16_t)TxBufSize;
    DMA_Init(USART_TX_DMA_STREAM[COMx], &DMA_InitStructure[COMx]);

    // Enable the USART DMA Transfer Complete interrupt
    DMA_ITConfig(USART_TX_DMA_STREAM[COMx], DMA_IT_TC, ENABLE);
    
    // Enable the USART DMA requests
    USART_DMACmd(COM_USART[COMx], USART_DMAReq_Tx, ENABLE);

    // Clear the TC bit in the SR register by writing 0 to it
    USART_ClearFlag(COM_USART[COMx], USART_FLAG_TC);

    // Enable the DMA TX Stream, i.e. USART will start sending the data
    DMA_Cmd(USART_TX_DMA_STREAM[COMx], ENABLE);   
} 
/*****************************************************************************/




/*****************************************************************************
  * @brief  DMA in Memory to Periferal mode
  * @param  None
  * @retval None
  ***************************************************************************/
void USART_Tx_DMA_Init(COM_TypeDef COMx)
{
   // Enable COMx DMA Tx interrupts
  USART_DMA_Tx_NVIC_Init(COMx);
  
    // Init USART DMA
  COM_DMA_Init_Linear(COMx, &DMA_InitStructure[COMx]);
  
  //
  Transmit_USART_dma(COMx,(uint8_t*)RecDataBuff,PACKET_RECV_SIZE);

}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Wait if shadow register is full and write data when it gets empty. 
            Do not use DMA.
  * @param  None
  * @retval None
  ***************************************************************************/
void USART_WaitCharDataSend(USART_TypeDef* USARTx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
  
  /* The software must wait if TXE == 0. */
  while(!(USARTx->SR & 0x80));

  USART_SendData(USARTx, Data);     
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
  ***************************************************************************/
void USART_DataSend(COM_TypeDef COMx)
{
int16_t i;

  // The software must wait until TC=1.
  if(USART_GetFlagStatus(COM_USART[COMx], USART_FLAG_TC) == RESET) return;
  
  // Copy data from one buffer to another
  for(i=0; i<4; i++){  USART_ExtVarSh[i] = /*USART_*/ExtVar[i]; }
  
  //GPIOB->BSRRL = GPIO_Pin_2;// Set PC2(RS485 in transmitter mode)
  
  // Initialize and Start DMA
  Transmit_USART_dma(COMx,(uint8_t*)USART_ExtVarSh,8);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  
  * @param  
  *         
  *         
  * @retval None
  ***************************************************************************/
void DMA_USART_RX_Flags_Clear(COM_TypeDef COMx)
{
    // Clear all DMA Rx Streams flags
    DMA_ClearFlag(USART_RX_DMA_STREAM[COMx], USART_RX_DMA_FLAG_FEIF[COMx] | 
                  USART_RX_DMA_FLAG_DMEIF[COMx] | USART_RX_DMA_FLAG_TEIF[COMx] |
                  USART_RX_DMA_FLAG_HTIF[COMx] | USART_RX_DMA_FLAG_TCIF[COMx]); 
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Receive USARTx data using DMA and put them in a linear buffer
  * @param  COM_TypeDef COMx   : port
  *         uint8_t *RxBuf     : pointer to buffer
  *         uint16_t RxBufSize : data length
  * @retval None
  ***************************************************************************/
void Receive_USART_dma(COM_TypeDef COMx, const uint8_t *RxBuf, uint16_t RxBufSize)
{
    // POOLING: USART shift register transfer complete   
    // The software must wait until TC=1. The TC flag remains cleared during all data
    // transfers and it is set by hardware at the last frame’s end of transmission
    
    //while(USART_GetFlagStatus(COM_USART[COMx], USART_FLAG_TC) == RESET){}

    // Clear all DMA Rx Streams flags
    DMA_ClearFlag(USART_RX_DMA_STREAM[COMx], USART_RX_DMA_FLAG_FEIF[COMx] | USART_RX_DMA_FLAG_DMEIF[COMx] |
                                        USART_RX_DMA_FLAG_TEIF[COMx] | USART_RX_DMA_FLAG_HTIF[COMx] |
                                        USART_RX_DMA_FLAG_TCIF[COMx]);    

    // Prepare the DMA to transfer the transaction command (8bytes) from the memory to the USART
    DMA_DeInit(USART_RX_DMA_STREAM[COMx]);
    DMA_InitStructure[COMx].DMA_Channel = USART_RX_DMA_CHANNEL[COMx];
    DMA_InitStructure[COMx].DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure[COMx].DMA_Memory0BaseAddr = (uint32_t)RxBuf;
    DMA_InitStructure[COMx].DMA_BufferSize = (uint16_t)RxBufSize;
    DMA_Init(USART_RX_DMA_STREAM[COMx], &DMA_InitStructure[COMx]);

    // Enable the USART DMA Transfer Complete interrupt
    DMA_ITConfig(USART_RX_DMA_STREAM[COMx], DMA_IT_TC, ENABLE);
    
    // Enable the USART DMA requests
    USART_DMACmd(COM_USART[COMx], USART_DMAReq_Rx, ENABLE);

    // Clear the TC bit in the SR register by writing 0 to it
    //USART_ClearFlag(COM_USART[COMx], USART_FLAG_TC);

    // Enable the DMA TX Stream, i.e. USART will start sending the data
    DMA_Cmd(USART_RX_DMA_STREAM[COMx], ENABLE);   
} 
/*****************************************************************************/




/*****************************************************************************
  * @brief  DMA in Periferal to Memory mode
  * @param  None
  * @retval None
  ***************************************************************************/
void USART_Rx_DMA_Init(COM_TypeDef COMx)
{
      // Enable COMx DMA Rx interrupts
  USART_DMA_Rx_NVIC_Init(COMx);
  
    // Init USART DMA
  COM_DMA_Init_Linear(COMx, &DMA_InitStructure[COMx]);
  
  //
  Receive_USART_dma(COMx,(uint8_t*)RecDataBuff,PACKET_RECV_SIZE);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM2  
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  ***************************************************************************/
void COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM], ENABLE);

  
 if ((COM == COM1) || (COM == COM6))
  {
    /* Enable USART clock */
    RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
  }
  
  if ((COM == COM2) || (COM == COM3) || (COM == COM4) || (COM == COM5))
  {
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
  }
  
  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(COM_TX_PORT[COM], COM_TX_PIN_SOURCE[COM], COM_TX_AF[COM]);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(COM_RX_PORT[COM], COM_RX_PIN_SOURCE[COM], COM_RX_AF[COM]);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // Hoang change 2MHz -> 50MHz
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
  GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
  GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

  /* USART DeInitialization */
  USART_DeInit(COM_USART[COM]);

  /* USART configuration */
  USART_Init(COM_USART[COM], USART_InitStruct);
    
  /* Enable USART */
  USART_Cmd(COM_USART[COM], ENABLE);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Enable/Disable Rx Interrupts
  * @param  None
  * @retval None
  ***************************************************************************/
void USART_Rx_Interrupt_Cfg(COM_TypeDef COMx, FunctionalState STATE)
{
  if (STATE == ENABLE){
    USART_ClearFlag(COM_USART[COMx], USART_FLAG_RXNE); //Clear Flag "Receive data register not empty flag"
    USART_ITConfig(COM_USART[COMx], USART_IT_RXNE, ENABLE);   // Enable USART "Receive Data register not empty" interrupt
    RingBuf_Init ((RingBuffTypeDef *)&USART_Rx_RingBuff[COMx]); //Initialize USART Rx Ring Buffer
  }else{
    USART_ITConfig(COM_USART[COMx], USART_IT_RXNE, DISABLE);   // Disable USART "Receive Data register not empty" interrupt
  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  USART1_Config
  * @param  None
  * @retval None
  ***************************************************************************/
void USART1_Config(void)
{
  /* USARTx configured as follow:
        - BaudRate = 460800 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = BaudRate_COM1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  COMInit(COM1, &USART_InitStructure);
  
  // Init USART DMA
  COM_DMA_Init_Linear(COM1, &DMA_InitStructure[COM1]);
  
  // Enable COM1 DMA Tx interrupts
  //USART_DMA_Tx_NVIC_Init(COM1);
  
  /* Enable the USART1 Receive interrupt: this interrupt is generated when the 
     USART1 receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  USART2_Config
  * @param  None
  * @retval None
  ***************************************************************************/
void USART2_Config(void)
{
  /* USARTx configured as follow:
        - BaudRate = 460800 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = BaudRate_COM2;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  COMInit(COM2, &USART_InitStructure);
  
  // Init USART DMA
  //COM_DMA_Init_Linear(COM2, &DMA_InitStructure[COM2]);
  
  // Enable COM2 DMA Tx interrupts
  //USART_DMA_Tx_NVIC_Init(COM2);
  
  /* Enable the USART2 Receive interrupt: this interrupt is generated when the 
     USART1 receive data register is not empty */
  //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  USART3_Config
  * @param  None
  * @retval None
  ***************************************************************************/
void USART3_Config(void)
{
  /* USARTx configured as follow:
        - BaudRate = 460800 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = BaudRate_COM3;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  COMInit(COM3, &USART_InitStructure);
  
  // Init USART DMA
  //COM_DMA_Init_Linear(COM3, &DMA_InitStructure[COM3]);
  
  // Enable COM3 DMA Tx interrupts
  //USART_DMA_Tx_NVIC_Init(COM3);
  
  /* Enable the USART3 Receive interrupt: this interrupt is generated when the 
     USART3 receive data register is not empty */
  //USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  USART6_Config
  * @param  None
  * @retval None
  ***************************************************************************/
void USART6_Config(void)
{
  /* USARTx configured as follow:
        - BaudRate = 460800 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = BaudRate_COM6;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  COMInit(COM6, &USART_InitStructure);
  
  // Init USART DMA
  //COM_DMA_Init_Linear(COM6, &DMA_InitStructure[COM6]);
  
  // Enable COM1 DMA Tx interrupts
  //USART_DMA_Tx_NVIC_Init(COM6);
  
  /* Enable the USART1 Receive interrupt: this interrupt is generated when the 
     USART1 receive data register is not empty */
  //USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  USARTx_BaudRate
  * @param  None
  * @retval None
  ***************************************************************************/
void USARTx_BaudRate(COM_TypeDef COM, uint32_t Baudrate)
{
  /* USARTx configured as follow:
        - BaudRate = 460800 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = Baudrate;
  /*USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;*/

    // USART configuration
  USART_Init(COM_USART[COM], &USART_InitStructure);
  
  // Init USART DMA
  //COM_DMA_Init_Linear(COM3, &DMA_InitStructure[COM3]);
  
  // Enable COM3 DMA Tx interrupts
  //USART_DMA_Tx_NVIC_Init(COM3);
  
  /* Enable the USART3 Receive interrupt: this interrupt is generated when the 
     USART3 receive data register is not empty */
  //USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  USART_Config
  * @param  None
  * @retval None
  ***************************************************************************/
void USART_Config(void)
{
  //USART1_Config();
  USART2_Config();
  USART_Tx_DMA_Init(COM2);
  //USART6_Config();
  
  // USART3 configuration
  USART3_Config();
  TIM6_Config();

  //USART_Tx_DMA_Init();

  // Initialize DMA in Periferal to Memory mode
  USART_Rx_DMA_Init(COM3);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
  ***************************************************************************/
void USART3_DMA_TransmitReInit(void)
{
  // USART3 configuration
  COM3ReInit();
  
  // Initialize DMA in Memory to Periferal mode(USART transmit)
  USART_Tx_DMA_Init(COM3);
  
  // Clear all DMA Rx Streams flags
  //DMA_USART_RX_Flags_Clear(COM3);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
  ***************************************************************************/
void USART3_DMA_Reset(void)
{
	uint16_t count = DMA_GetCurrDataCounter(USART3_RX_DMA_STREAM);
	if ((count != 0) && (count != PACKET_RECV_SIZE))
	{
		USART_Rx_DMA_Init(COM3);
	}
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
  ***************************************************************************/
void COM3ReInit(void)
{
  //USART_InitTypeDef USART_InitStructure;
  
  USART_InitStructure.USART_BaudRate = BaudRate_COM3;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  //COMInit(COM3, &USART_InitStructure);
  
  /* USART DeInitialization */
  USART_DeInit(COM_USART[COM3]);

  /* USART configuration */
  USART_Init(COM_USART[COM3], &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(COM_USART[COM3], ENABLE);
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/