#ifdef __STM32F4__

#include "../HAL.h"
#include "HAL_Stm32f4.h"
#include "board.h"
//#include <STM32ADC.h>
#include "pins_MALYAN_M200.h"
#include "Marlin.h"

void watchdog_reset(){}
void watchdog_init(){}
void pinMode(uint8 pin, WiringPinMode mode){}
void spiBegin() {}
void spiInit(uint8_t spiRate) {}
void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) {}
uint8_t spiRec() {}
void spiRead(uint8_t* buf, uint16_t nbyte) {}
void spiSend(uint8_t data) {}
void spiSendBlock(uint8_t token, const uint8_t* buf) {}
void digitalWrite(uint8 pin, uint8 value)
{
    if (value!=0) HAL_GPIO_WritePin(PIN_MAP[pin].gpio_port,PIN_MAP[pin].gpio_bit,GPIO_PIN_SET);
    else HAL_GPIO_WritePin(PIN_MAP[pin].gpio_port,PIN_MAP[pin].gpio_bit,GPIO_PIN_RESET);
}
uint32 digitalRead(uint8 pin)
{
  return HAL_GPIO_ReadPin(PIN_MAP[pin].gpio_port,PIN_MAP[pin].gpio_bit);
}
void analogWrite(uint8 pin, uint8 value)
{
    switch (pin)
    {
        case PA6:
            TIM3->CCR1=value;
            break;
        case PA7:
            TIM3->CCR2=value;
            break;
        /*case PB8:
            TIM_SetCompare2(TIM15, value);
            break;
        case PB9:
            TIM_SetCompare1(TIM16, value);
            break;
        case PB15:
            TIM_SetCompare1(TIM17, value);
            break;*/
        default:
    }
}
#if 1
//#include "stm32f0xx_adc.h"
//#define ADC1_DR_Address    ((uint32_t)0x40012440)

uint16_t HAL_adc_result;

/*uint8 adc_pins[] = {
  #if HAS_TEMP_0
    TEMP_0_PIN,
  #endif
  #if HAS_TEMP_1
    TEMP_1_PIN
  #endif
  #if HAS_TEMP_2
    TEMP_2_PIN,
  #endif
  #if HAS_TEMP_3
    TEMP_3_PIN,
  #endif
  #if HAS_TEMP_4
    TEMP_4_PIN,
  #endif
  #if HAS_TEMP_BED
    TEMP_BED_PIN,
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    FILWIDTH_PIN,
  #endif
    TEMP_CPU_PIN,
};*/

enum TEMP_PINS {
  #if HAS_TEMP_0
    TEMP_0,
  #endif
  #if HAS_TEMP_1
    TEMP_1,
  #endif
  #if HAS_TEMP_2
    TEMP_2,
  #endif
  #if HAS_TEMP_3
    TEMP_3,
  #endif
  #if HAS_TEMP_4
    TEMP_4,
  #endif
  #if HAS_TEMP_BED
    TEMP_BED,
  #endif
  /*#if ENABLED(FILAMENT_WIDTH_SENSOR)
    FILWIDTH,
  #endif*/
    TEMP_CPU,
    ADC_PIN_COUNT
};

uint16_t HAL_adc_results[ADC_PIN_COUNT];

uint16_t AD_Value[5]={0};
uint32_t AD_Value_avr[5]={0};

void delay(uint32_t ms)
{
    uint32_t start = millis();
#if SIMULATE_DEBUG
    return;
#endif
    while (millis() - start < ms)
    {
        //iwdg_feed();
    }
}

void HAL_clear_reset_source(void) { }

uint8_t HAL_get_reset_source(void) { return 1; }

//void _delay_ms(const int delay_ms) { delay(delay_ms); }

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

/**
 * TODO: Change this to correct it for libmaple
 */

// return free memory between end of heap (or end bss) and whatever is current

/*
#include "wirish/syscalls.c"
//extern caddr_t _sbrk(int incr);
#ifndef CONFIG_HEAP_END
extern char _lm_heap_end;
#define CONFIG_HEAP_END ((caddr_t)&_lm_heap_end)
#endif

extern "C" {
  static int freeMemory() {
    char top = 't';
    return &top - reinterpret_cast<char*>(sbrk(0));
  }
  int freeMemory() {
    int free_memory;
    int heap_end = (int)_sbrk(0);
    free_memory = ((int)&free_memory) - ((int)heap_end);
    return free_memory;
  }
}
*/

// --------------------------------------------------------------------------
// ADC
// --------------------------------------------------------------------------
// Init the AD in continuous capture mode
void HAL_adc_init(void) {
  // configure the ADC
}

void HAL_adc_start_conversion(const uint8_t adc_pin) {
  TEMP_PINS pin_index;
  switch (adc_pin) {
    #if HAS_TEMP_0
      case TEMP_0_PIN: pin_index = TEMP_0; break;
    #endif
    #if HAS_TEMP_1
      case TEMP_1_PIN: pin_index = TEMP_1; break;
    #endif
    #if HAS_TEMP_2
      case TEMP_2_PIN: pin_index = TEMP_2; break;
    #endif
    #if HAS_TEMP_3
      case TEMP_3_PIN: pin_index = TEMP_3; break;
    #endif
    #if HAS_TEMP_4
      case TEMP_4_PIN: pin_index = TEMP_4; break;
    #endif
    #if HAS_TEMP_BED
      case TEMP_BED_PIN: pin_index = TEMP_BED; break;
    #endif
    /*#if ENABLED(FILAMENT_WIDTH_SENSOR)
      case FILWIDTH_PIN: pin_index = FILWIDTH; break;
    #endif*/
  }
  HAL_adc_results[TEMP_0]=AD_Value[2];
  HAL_adc_results[TEMP_BED]=AD_Value[1];
  HAL_adc_results[TEMP_CPU]=AD_Value[0];
  HAL_adc_result = (HAL_adc_results[(int)pin_index] >> 2) & 0x3FF; // shift to get 10 bits only.
}

uint16_t HAL_adc_get_result(void) {
  return HAL_adc_result;
}

int32_t map(int32_t value, int32_t fromStart, int32_t fromEnd,
     int32_t toStart, int32_t toEnd) {
     return ((int64_t)(value - fromStart) * (toEnd - toStart)) / (fromEnd - fromStart) +
         toStart;
 }

static char adc_loop;
unsigned char temp_count;//yongzong

/*extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  uint8_t i;

    AD_Value_avr[0]+=AD_Value[0];
    AD_Value_avr[1]+=AD_Value[1];
    AD_Value_avr[2]+=AD_Value[2];

    //if (adc_loop>=3)
    {
      adc_loop=0;
      HAL_adc_results[TEMP_0]=AD_Value_avr[2];
      HAL_adc_results[TEMP_BED]=AD_Value_avr[1];
      HAL_adc_results[TEMP_CPU]=AD_Value_avr[0];
#if FAKE_TEMPERATURE
      raw_temp_value[0]=2000;
      raw_temp_bed_value=12000;
#endif
      AD_Value_avr[0]=0;
      AD_Value_avr[1]=0;
      AD_Value_avr[2]=0;
    }
    //else adc_loop++;
}
*/
#endif

const cpu_pin_info PIN_MAP[] = {
{GPIOA,1},//0
{GPIOA,2},//1
{GPIOA,4},//2
{GPIOA,8},//3
{GPIOA,16},//4
{GPIOA,32},//5
{GPIOA,64},//6
{GPIOA,128},//7
{GPIOA,256},//8
{GPIOA,512},//9
{GPIOA,1024},//10
{GPIOA,2048},//11
{GPIOA,4096},//12
{GPIOA,8192},//13
{GPIOA,16384},//14
{GPIOA,32768},//15
{GPIOB,1},//16
{GPIOB,2},//17
{GPIOB,4},//18
{GPIOB,8},//19
{GPIOB,16},//20
{GPIOB,32},//21
{GPIOB,64},//22
{GPIOB,128},//23
{GPIOB,256},//24
{GPIOB,512},//25
{GPIOB,1024},//26
{GPIOB,2048},//27
{GPIOB,4096},//28
{GPIOB,8192},//29
{GPIOB,16384},//30
{GPIOB,32768},//31
{GPIOC,1},//32
{GPIOC,2},//33
{GPIOC,4},//34
{GPIOC,8},//35
{GPIOC,16},//36
{GPIOC,32},//37
{GPIOC,64},//38
{GPIOC,128},//39
{GPIOC,256},//40
{GPIOC,512},//41
{GPIOC,1024},//42
{GPIOC,2048},//43
{GPIOC,4096},//44
{GPIOC,8192},//45
{GPIOC,16384},//46
{GPIOC,32768},//47
{GPIOD,1},//48
{GPIOD,2},//49
{GPIOD,4},//50
{GPIOD,8},//51
{GPIOD,16},//52
{GPIOD,32},//53
{GPIOD,64},//54
{GPIOD,128},//55
{GPIOD,256},//56
{GPIOD,512},//57
{GPIOD,1024},//58
{GPIOD,2048},//59
{GPIOD,4096},//60
{GPIOD,8192},//61
{GPIOD,16384},//62
{GPIOD,32768},//63
{GPIOE,1},//48
{GPIOE,2},//49
{GPIOE,4},//50
{GPIOE,8},//51
{GPIOE,16},//52
{GPIOE,32},//53
{GPIOE,64},//54
{GPIOE,128},//55
{GPIOE,256},//56
{GPIOE,512},//57
{GPIOE,1024},//58
{GPIOE,2048},//59
{GPIOE,4096},//60
{GPIOE,8192},//61
{GPIOE,16384},//62
{GPIOE,32768},//63
/*{GPIOF,1},//48
{GPIOF,2},//49
{GPIOF,4},//50
{GPIOF,8},//51
{GPIOF,16},//52
{GPIOF,32},//53
{GPIOF,64},//54
{GPIOF,128},//55
{GPIOF,256},//56
{GPIOF,512},//57
{GPIOF,1024},//58
{GPIOF,2048},//59
{GPIOF,4096},//60
{GPIOF,8192},//61
{GPIOF,16384},//62
{GPIOF,32768},//63*/
};

#endif // __STM32F1__
