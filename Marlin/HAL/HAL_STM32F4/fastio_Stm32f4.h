/**
 * Fast I/O interfaces for STM32F1
 * These use GPIO functions instead of Direct Port Manipulation, as on AVR.
 */

#ifndef _FASTIO_STM32F4_H
#define _FASTIO_STM32F4_H

//#include <libmaple/gpio.h>

#define READ(IO)              (PIN_MAP[IO].gpio_port->IDR & (PIN_MAP[IO].gpio_bit) ? HIGH : LOW)
#define SET_LOW(IO)           {PIN_MAP[IO].gpio_port->BSRR = (uint32_t)(PIN_MAP[IO].gpio_bit) << 16U;}
#define SET_HIGH(IO)          {PIN_MAP[IO].gpio_port->BSRR = (uint32_t)(PIN_MAP[IO].gpio_bit);}
#define WRITE(IO, v)          {if (v!=0) PIN_MAP[IO].gpio_port->BSRR=PIN_MAP[IO].gpio_bit;else PIN_MAP[IO].gpio_port->BSRR = (uint32_t)(PIN_MAP[IO].gpio_bit) << 16U;}//(PIN_MAP[IO].gpio_device->regs->BSRR = (1U << PIN_MAP[IO].gpio_bit) << (16 * !(bool)v))
#define TOGGLE(IO)            ((PIN_MAP[IO].gpio_port->ODR) ^= (PIN_MAP[IO].gpio_bit))//(PIN_MAP[IO].gpio_device->regs->ODR = PIN_MAP[IO].gpio_device->regs->ODR ^ (1U << PIN_MAP[IO].gpio_bit))
#define WRITE_VAR(IO, v)      WRITE(io, v)

#define _GET_MODE(IO)         //(gpio_get_mode(PIN_MAP[IO].gpio_device, PIN_MAP[IO].gpio_bit))
#define _SET_MODE(IO,M)       //do{ gpio_set_mode(PIN_MAP[IO].gpio_device, PIN_MAP[IO].gpio_bit, M); } while (0)
#define _SET_OUTPUT(IO)       {GPIO_InitTypeDef   GPIO_InitStructure = {0};\
                               GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;\
                               GPIO_InitStructure.Pull = GPIO_NOPULL;\
                               GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;\
                               GPIO_InitStructure.Pin = PIN_MAP[IO].gpio_bit;\
                               HAL_GPIO_Init(PIN_MAP[IO].gpio_port, &GPIO_InitStructure);}//_SET_MODE(IO, GPIO_OUTPUT_PP)

#define SET_INPUT(IO)         {GPIO_InitTypeDef   GPIO_InitStructure = {0};\
                               GPIO_InitStructure.Mode = GPIO_MODE_INPUT;\
                               GPIO_InitStructure.Pull = GPIO_NOPULL;\
                               GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;\
                               GPIO_InitStructure.Pin = PIN_MAP[IO].gpio_bit;\
                               HAL_GPIO_Init(PIN_MAP[IO].gpio_port, &GPIO_InitStructure);}//_SET_MODE(IO, GPIO_INPUT_FLOATING)
#define SET_INPUT_PULLUP(IO)  {GPIO_InitTypeDef   GPIO_InitStructure = {0};\
                               GPIO_InitStructure.Mode = GPIO_MODE_INPUT;\
                               GPIO_InitStructure.Pull = GPIO_PULLUP;\
                               GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;\
                               GPIO_InitStructure.Pin = PIN_MAP[IO].gpio_bit;\
                               HAL_GPIO_Init(PIN_MAP[IO].gpio_port, &GPIO_InitStructure);}//_SET_MODE(IO, GPIO_INPUT_PU)
#define SET_OUTPUT(IO)        { _SET_OUTPUT(IO); WRITE(IO, LOW); }

#define GET_INPUT(IO)         //(_GET_MODE(IO) == GPIO_INPUT_FLOATING || _GET_MODE(IO) == GPIO_INPUT_ANALOG || _GET_MODE(IO) == GPIO_INPUT_PU || _GET_MODE(IO) == GPIO_INPUT_PD)
#define GET_OUTPUT(IO)        //(_GET_MODE(IO) == GPIO_OUTPUT_PP)
#define GET_TIMER(IO)         //(PIN_MAP[IO].timer_device != NULL)

#define OUT_WRITE(IO, v)      { _SET_OUTPUT(IO); WRITE(IO, v); }
/**
 * TODO: Write a macro to test if PIN is PWM or not.
 */
#define PWM_PIN(p)            true

#endif // _FASTIO_STM32F1_H
