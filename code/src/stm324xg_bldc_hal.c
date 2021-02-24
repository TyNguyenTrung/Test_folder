// stm324xg_bldc_hal.c
// This file provides - set of firmware functions to manage Leds, push-button

#include "stm324xg_bldc_hal.h"



    
// LED IO description
const uint16_t         LED_PIN[LEDn] = {LED_R_PIN,       LED_G_PIN,       LED_RJ45Y_PIN,       LED_RJ45G_PIN      }; 
GPIO_TypeDef*         LED_PORT[LEDn] = {LED_R_GPIO_PORT, LED_G_GPIO_PORT, LED_RJ45Y_GPIO_PORT, LED_RJ45G_GPIO_PORT};
const uint32_t         LED_CLK[LEDn] = {LED_R_GPIO_CLK,  LED_G_GPIO_CLK,  LED_RJ45Y_GPIO_CLK,  LED_RJ45G_GPIO_CLK };

// Joystick IO description
const uint16_t         JOY_PIN[JOYn] = {JOY_SEL_PIN,              JOY_DOWN_PIN,              JOY_LEFT_PIN,              JOY_RIGHT_PIN,              JOY_UP_PIN             }; 
GPIO_TypeDef*         JOY_PORT[JOYn] = {JOY_SEL_GPIO_PORT,        JOY_DOWN_GPIO_PORT,        JOY_LEFT_GPIO_PORT,        JOY_RIGHT_GPIO_PORT,        JOY_UP_GPIO_PORT       }; 
const uint32_t         JOY_CLK[JOYn] = {JOY_SEL_GPIO_CLK,         JOY_DOWN_GPIO_CLK,         JOY_LEFT_GPIO_CLK,         JOY_RIGHT_GPIO_CLK,         JOY_UP_GPIO_CLK        };
const uint16_t   JOY_EXTI_LINE[JOYn] = {JOY_SEL_EXTI_LINE,        JOY_DOWN_EXTI_LINE,        JOY_LEFT_EXTI_LINE,        JOY_RIGHT_EXTI_LINE,        JOY_UP_EXTI_LINE       };
const uint16_t JOY_PORT_SOURCE[JOYn] = {JOY_SEL_EXTI_PORT_SOURCE, JOY_DOWN_EXTI_PORT_SOURCE, JOY_LEFT_EXTI_PORT_SOURCE, JOY_RIGHT_EXTI_PORT_SOURCE, JOY_UP_EXTI_PORT_SOURCE};
const uint16_t  JOY_PIN_SOURCE[JOYn] = {JOY_SEL_EXTI_PIN_SOURCE,  JOY_DOWN_EXTI_PIN_SOURCE,  JOY_LEFT_EXTI_PIN_SOURCE,  JOY_RIGHT_EXTI_PIN_SOURCE,  JOY_UP_EXTI_PIN_SOURCE }; 
const uint16_t        JOY_IRQn[JOYn] = {JOY_SEL_EXTI_IRQn,        JOY_DOWN_EXTI_IRQn,        JOY_LEFT_EXTI_IRQn,        JOY_RIGHT_EXTI_IRQn,        JOY_UP_EXTI_IRQn       };

void LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  // Enable the GPIO_LED Clock
  RCC_AHB1PeriphClockCmd(LED_CLK[Led], ENABLE);

  // Configure the GPIO_LED pin
  GPIO_InitStructure.GPIO_Pin = LED_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED_PORT[Led], &GPIO_InitStructure);
}

void LEDOff(Led_TypeDef Led)
{
  LED_PORT[Led]->BSRRL = LED_PIN[Led];
}

void LEDOn(Led_TypeDef Led)
{
  LED_PORT[Led]->BSRRH = LED_PIN[Led];  
}

void LEDToggle(Led_TypeDef Led)
{
  LED_PORT[Led]->ODR ^= LED_PIN[Led];
}


