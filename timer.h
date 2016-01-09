/**
  * @file timer.c
  * @brief  Impliments msTicks and millis() on TIM2 for timing
  * @author Eason Smith
  * @date 12/23/2015
  */
#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
/** @addtogroup BMP085_Peripheral_Driver
  * @{
  */

/** @defgroup TIMER
  * @brief TIMER contains timer functions used by pressure sensor driver modules
  * @{
  */ 

/** @var static volatile uint32_t msTicks
  * @brief milliseconds elapsed since TIM2 was initialized.
  */
static volatile uint32_t msTicks = 0; // counts 1ms timeTicks
/**
  * @fn void TIM2_IRQHandler(void)
  * @brief  incriments msticks 1000 times/second.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
		//clear interrupt flag
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		msTicks++;
   
  }
}

/**
  * @fn uint32_t millis(void)
  * @brief  returns number of milliseconds elapsed since the device was turned on.
  * @param  None
  * @retval None
  */
uint32_t millis(void){ return msTicks; } 

//
/**
  * @fn TIM2_init(void)
  * @brief  initializes TIM2. 
  * @param  None
  * @retval None
  */
void TIM2_init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  //Enable the TIM2 globaly 
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  //TIM2 clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  //72 - 1 for a 72 MHz system, some # - 1 for future systems with different clocks
  //Time base config
  TIM_TimeBaseStructure.TIM_Period = 1000 - 1;  // 1 MHz down to 1 KHz (1 ms)
  TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 72 MHz Clock down to 1 MHz (adjust 72 to the clock of the uC)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  //TIM IT enable
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  //TIM2 enable counter
  TIM_Cmd(TIM2, ENABLE);

}
#endif

/**
* @}
*/ 
/**
* @}
*/ 
