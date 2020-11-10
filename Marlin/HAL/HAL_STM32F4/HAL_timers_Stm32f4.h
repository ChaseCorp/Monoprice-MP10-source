/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2017 Victor Perez
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

#ifndef _HAL_TIMERS_STM32F1_H
#define _HAL_TIMERS_STM32F1_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
//#include "stm32f0xx_tim.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include <stdint.h>

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

/**
 * TODO: Check and confirm what timer we will use for each Temps and stepper driving.
 * We should probable drive temps with PWM.
 */
#define FORCE_INLINE __attribute__((always_inline)) inline
#define min(a,b)                ((a)<(b)?(a):(b))

typedef uint16_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFF

#define STEP_TIMER_NUM TIM10
#define STEP_TIMER_CHAN 1 // Channel of the timer to use for compare and interrupts
#define TEMP_TIMER_NUM TIM11  // index of timer to use for temperature
#define TEMP_TIMER_CHAN 1 // Channel of the timer to use for compare and interrupts
/*
#define CAT(a, ...) a ## __VA_ARGS__
#define TIMER_DEV(num) CAT (&timer, num)

#define STEP_TIMER_DEV TIMER_DEV(STEP_TIMER_NUM)
#define TEMP_TIMER_DEV TIMER_DEV(TEMP_TIMER_NUM)
*/
//STM32_HAVE_TIMER(n);

#define HAL_TIMER_RATE         (F_CPU)  // frequency of timers peripherals
#define STEPPER_TIMER_PRESCALE 21             // prescaler for setting stepper timer, 4Mhz
#define HAL_STEPPER_TIMER_RATE (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)   // frequency of stepper timer (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)
#define HAL_TICKS_PER_US       ((HAL_STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per us

#define PULSE_TIMER_NUM STEP_TIMER_NUM
#define PULSE_TIMER_PRESCALE STEPPER_TIMER_PRESCALE

#define TEMP_TIMER_PRESCALE     1000 // prescaler for setting Temp timer, 72Khz
#define TEMP_TIMER_FREQUENCY    100 // temperature interrupt frequency

#define TIM_ENABLE_IT(Instance, __INTERRUPT__)    (Instance->DIER |= (__INTERRUPT__))
#define TIM_DISABLE_IT(Instance, __INTERRUPT__)   (Instance->DIER &= ~(__INTERRUPT__))

#define ENABLE_STEPPER_DRIVER_INTERRUPT() TIM_ENABLE_IT(STEP_TIMER_NUM,TIM_IT_UPDATE)//timer_enable_irq(STEP_TIMER_DEV, STEP_TIMER_CHAN)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIM_DISABLE_IT(STEP_TIMER_NUM,TIM_IT_UPDATE)//timer_disable_irq(STEP_TIMER_DEV, STEP_TIMER_CHAN)
#define STEPPER_ISR_ENABLED() ((STEP_TIMER_NUM->DIER & TIM_IT_UPDATE)!=0)//HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT() TIM_ENABLE_IT(TEMP_TIMER_NUM,TIM_IT_UPDATE)//timer_enable_irq(TEMP_TIMER_DEV, TEMP_TIMER_CHAN)
#define DISABLE_TEMPERATURE_INTERRUPT() TIM_DISABLE_IT(TEMP_TIMER_NUM,TIM_IT_UPDATE)//timer_disable_irq(TEMP_TIMER_DEV, TEMP_TIMER_CHAN)

#define HAL_timer_get_count(timer_num) timer_num->CNT//timer_get_count(TIMER_DEV(timer_num))
#define HAL_timer_set_count(timer_num, count) timer_num->CNT=count//timer_set_count(TIMER_DEV(timer_num), (uint16)count)


#define HAL_ENABLE_ISRs() do { if (thermalManager.in_temp_isr) DISABLE_TEMPERATURE_INTERRUPT(); else ENABLE_TEMPERATURE_INTERRUPT(); ENABLE_STEPPER_DRIVER_INTERRUPT(); } while(0)
// TODO change this


#define HAL_TEMP_TIMER_ISR extern "C" void tempTC_Handler(void)
#define HAL_STEP_TIMER_ISR extern "C" void stepTC_Handler(void)

extern "C" void tempTC_Handler(void);
extern "C" void stepTC_Handler(void);

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------
/*
static HardwareTimer StepperTimer(STEP_TIMER_NUM);
static HardwareTimer TempTimer(TEMP_TIMER_NUM);
*/
// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start(TIM_TypeDef* timer_num, const uint32_t frequency);
void HAL_timer_enable_interrupt(TIM_TypeDef* timer_num);
void HAL_timer_disable_interrupt(TIM_TypeDef* timer_num);
bool HAL_timer_interrupt_enabled(TIM_TypeDef* timer_num);

/**
 * NOTE: By default libmaple sets ARPE = 1, which means the Auto reload register is preloaded (will only update with an update event)
 * Thus we have to pause the timer, update the value, refresh, resume the timer.
 * That seems like a big waste of time and may be better to change the timer config to ARPE = 0, so ARR can be updated any time.
 * We are using a Channel in each timer in Capture/Compare mode. We could also instead use the Time Update Event Interrupt, but need to disable ARPE
 * so we can change the ARR value on the fly (without calling refresh), and not get an interrupt right there because we caused an UEV.
 * This mode pretty much makes 2 timers unusable for PWM since they have their counts updated all the time on ISRs.
 * The way Marlin manages timer interrupts doesn't make for an efficient usage in STM32F1
 * Todo: Look at that possibility later.
 */

FORCE_INLINE static void HAL_timer_set_compare(TIM_TypeDef* timer_num, const hal_timer_t compare) {
  timer_num->ARR = min(compare, HAL_TIMER_TYPE_MAX);;
/*  switch (timer_num) {
  case STEP_TIMER_NUM:
    timer_set_compare(STEP_TIMER_DEV, STEP_TIMER_CHAN, compare);
    return;
  case TEMP_TIMER_NUM:
    timer_set_compare(TEMP_TIMER_DEV, TEMP_TIMER_CHAN, compare);
    return;
  default:
    return;
  }*/
}

FORCE_INLINE static hal_timer_t HAL_timer_get_compare(TIM_TypeDef* timer_num) {
    return timer_num->ARR;
/*  switch (timer_num) {
  case STEP_TIMER_NUM:
    return timer_get_compare(STEP_TIMER_DEV, STEP_TIMER_CHAN);
  case TEMP_TIMER_NUM:
    return timer_get_compare(TEMP_TIMER_DEV, TEMP_TIMER_CHAN);
  default:
    return 0;
  }*/
}

FORCE_INLINE static void HAL_timer_isr_prologue(TIM_TypeDef* timer_num) {
/*  switch (timer_num) {
  case STEP_TIMER_NUM:
    timer_set_count(STEP_TIMER_DEV, 0);
    timer_generate_update(STEP_TIMER_DEV);
    return;
  case TEMP_TIMER_NUM:
    timer_set_count(TEMP_TIMER_DEV, 0);
    timer_generate_update(TEMP_TIMER_DEV);
    return;
  default:
    return;
  }*/
}

#endif // _HAL_TIMERS_STM32F1_H
