/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * HAL for stm32duino.com based on Libmaple and compatible (STM32F1)
 */

#ifdef __STM32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../HAL.h"

#include "HAL_timers_Stm32f1.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

//#define NUM_HARDWARE_TIMERS 4


/**
 * Timer_clock1: Prescaler   2 ->  36    MHz
 * Timer_clock2: Prescaler   8 ->   9    MHz
 * Timer_clock3: Prescaler  32 ->   2.25 MHz
 * Timer_clock4: Prescaler 128 -> 562.5  kHz
 */

/**
 * TODO: Calculate Timer prescale value, so we get the 32bit to adjust
 */

void HAL_timer_start(TIM_TypeDef* timer_num, const uint32_t frequency) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    if (timer_num==STEP_TIMER_NUM)
    {
        NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

        TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  //~122Hz
        TIM_TimeBaseStructure.TIM_Prescaler = 24-1;   //2Mhz
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

        TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
        TIM_Cmd(TIM1, ENABLE);
    }
    else
    {
        NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

        TIM_TimeBaseStructure.TIM_Period = 10000;//100Hz
        TIM_TimeBaseStructure.TIM_Prescaler = 48-1;//1Mhz
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

        TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
        TIM_Cmd(TIM3, ENABLE);
    }
    
    return;
  /*nvic_irq_num irq_num;
  switch (timer_num) {
    case 1: irq_num = NVIC_TIMER1_CC; break;
    case 2: irq_num = NVIC_TIMER2; break;
    case 3: irq_num = NVIC_TIMER3; break;
    case 4: irq_num = NVIC_TIMER4; break;
    case 5: irq_num = NVIC_TIMER5; break;
    default:
      break;
  }
  nvic_irq_set_priority(irq_num, 0xF); // this is the lowest settable priority, but should still be over USB

  switch (timer_num) {
    case STEP_TIMER_NUM:
      timer_pause(STEP_TIMER_DEV);
      timer_set_count(STEP_TIMER_DEV, 0);
      timer_set_prescaler(STEP_TIMER_DEV, (uint16)(STEPPER_TIMER_PRESCALE - 1));
      timer_set_reload(STEP_TIMER_DEV, 0xFFFF);
      timer_set_compare(STEP_TIMER_DEV, STEP_TIMER_CHAN, min(HAL_TIMER_TYPE_MAX, (HAL_STEPPER_TIMER_RATE / frequency)));
      timer_attach_interrupt(STEP_TIMER_DEV, STEP_TIMER_CHAN, stepTC_Handler);
      timer_generate_update(STEP_TIMER_DEV);
      timer_resume(STEP_TIMER_DEV);
      break;
    case TEMP_TIMER_NUM:
      timer_pause(TEMP_TIMER_DEV);
      timer_set_count(TEMP_TIMER_DEV, 0);
      timer_set_prescaler(TEMP_TIMER_DEV, (uint16)(TEMP_TIMER_PRESCALE - 1));
      timer_set_reload(TEMP_TIMER_DEV, 0xFFFF);
      timer_set_compare(TEMP_TIMER_DEV, TEMP_TIMER_CHAN, min(HAL_TIMER_TYPE_MAX, ((F_CPU / TEMP_TIMER_PRESCALE) / frequency)));
      timer_attach_interrupt(TEMP_TIMER_DEV, TEMP_TIMER_CHAN, tempTC_Handler);
      timer_generate_update(TEMP_TIMER_DEV);
      timer_resume(TEMP_TIMER_DEV);
      break;
  }*/
}

void HAL_timer_enable_interrupt(TIM_TypeDef* timer_num) {
  if (timer_num==STEP_TIMER_NUM) ENABLE_STEPPER_DRIVER_INTERRUPT();
  else if (timer_num==TEMP_TIMER_NUM) ENABLE_TEMPERATURE_INTERRUPT();
  /*
  switch (timer_num) {
    case STEP_TIMER_NUM: ENABLE_STEPPER_DRIVER_INTERRUPT(); break;
    case TEMP_TIMER_NUM: ENABLE_TEMPERATURE_INTERRUPT(); break;
    default: break;
  }*/
}

void HAL_timer_disable_interrupt(TIM_TypeDef* timer_num) {
  if (timer_num==STEP_TIMER_NUM) DISABLE_STEPPER_DRIVER_INTERRUPT();
  else if (timer_num==TEMP_TIMER_NUM) DISABLE_TEMPERATURE_INTERRUPT();
  /*switch (timer_num) {
    case STEP_TIMER_NUM: DISABLE_STEPPER_DRIVER_INTERRUPT(); break;
    case TEMP_TIMER_NUM: DISABLE_TEMPERATURE_INTERRUPT(); break;
    default: break;
  }*/
}
/*
static inline bool timer_irq_enabled(const uint8_t timer_num, const uint8 interrupt) {
  //return false;
  return bool(*bb_perip(&(dev->regs).adv->DIER, interrupt));
}*/

bool HAL_timer_interrupt_enabled(TIM_TypeDef* timer_num) {
  return ((timer_num->DIER & TIM_IT_Update)!=0);
}

extern "C" void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    stepTC_Handler();
}
extern "C" void TIM3_IRQHandler()
{
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    tempTC_Handler();
}

#endif // __STM32F1__
