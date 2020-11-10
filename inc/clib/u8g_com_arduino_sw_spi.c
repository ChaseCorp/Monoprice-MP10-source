/*
  
  u8g_arduino_sw_spi.c

  Universal 8bit Graphics Library
  
  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
  Update for ATOMIC operation done (01 Jun 2013)
    U8G_ATOMIC_OR(ptr, val)
    U8G_ATOMIC_AND(ptr, val)
    U8G_ATOMIC_START();
    U8G_ATOMIC_END();
 

*/

#include "u8g.h"

#if 1//defined(ARDUINO)

#define HIGH 0x1
#define LOW  0x0

#include "stm32f401xc.h"

typedef struct cpu_pin_info {
    GPIO_TypeDef* gpio_port;
    uint32_t gpio_bit;
} cpu_pin_info;
extern const cpu_pin_info PIN_MAP[];

// Note this needs to match with the PIN_MAP array in board.cpp
enum {
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13,PA14,PA15,
    PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13,PB14,PB15,
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
    PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
    PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
//    PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
};

#include "pins_MALYAN_M200.h"
#include "fastio_Stm32f4.h"
//#define u8g_com_arduino_digital_write(u8g   #define SET_OUTPUT(IO)        { _SET_OUTPUT(IO); WRITE(IO, LOW); }
#define u8g_com_arduino_digital_write(X,Y,Z)    WRITE(Y, Z)
#define u8g_com_arduino_assign_pin_output_high(X)   

#if ARDUINO < 100 
//#include <WProgram.h>    
//#include "wiring_private.h"
//#include "pins_arduino.h"

#else 
#include <Arduino.h> 
#include "wiring_private.h"
#endif

/*=========================================================*/
/* Arduino, AVR */

#if defined(__AVR__)

uint8_t u8g_bitData, u8g_bitNotData;
uint8_t u8g_bitClock, u8g_bitNotClock;
volatile uint8_t *u8g_outData;
volatile uint8_t *u8g_outClock;

static void u8g_com_arduino_init_shift_out(uint8_t dataPin, uint8_t clockPin)
{
  u8g_outData = portOutputRegister(digitalPinToPort(dataPin));
  u8g_outClock = portOutputRegister(digitalPinToPort(clockPin));
  u8g_bitData = digitalPinToBitMask(dataPin);
  u8g_bitClock = digitalPinToBitMask(clockPin);

  u8g_bitNotClock = u8g_bitClock;
  u8g_bitNotClock ^= 0x0ff;

  u8g_bitNotData = u8g_bitData;
  u8g_bitNotData ^= 0x0ff;
}

static void u8g_com_arduino_do_shift_out_msb_first(uint8_t val) U8G_NOINLINE;
static void u8g_com_arduino_do_shift_out_msb_first(uint8_t val)
{
  uint8_t cnt = 8;
  uint8_t bitData = u8g_bitData;
  uint8_t bitNotData = u8g_bitNotData;
  uint8_t bitClock = u8g_bitClock;
  uint8_t bitNotClock = u8g_bitNotClock;
  volatile uint8_t *outData = u8g_outData;
  volatile uint8_t *outClock = u8g_outClock;
  U8G_ATOMIC_START();
  do
  {
    if ( val & 128 )
      *outData |= bitData;
    else
      *outData &= bitNotData;
   
    *outClock |= bitClock;
    val <<= 1;
    cnt--;
    *outClock &= bitNotClock;
  } while( cnt != 0 );
  U8G_ATOMIC_END();
}

/*=========================================================*/
/* Arduino, Chipkit */
#elif defined(__18CXX) || defined(__PIC32MX)

uint16_t dog_bitData, dog_bitNotData;
uint16_t dog_bitClock, dog_bitNotClock;
volatile uint32_t *dog_outData;
volatile uint32_t *dog_outClock;
volatile uint32_t dog_pic32_spi_tmp;

static void u8g_com_arduino_init_shift_out(uint8_t dataPin, uint8_t clockPin)
{
  dog_outData = portOutputRegister(digitalPinToPort(dataPin));
  dog_outClock = portOutputRegister(digitalPinToPort(clockPin));
  dog_bitData = digitalPinToBitMask(dataPin);
  dog_bitClock = digitalPinToBitMask(clockPin);

  dog_bitNotClock = dog_bitClock;
  dog_bitNotClock ^= 0x0ffff;

  dog_bitNotData = dog_bitData;
  dog_bitNotData ^= 0x0ffff;
}

static void u8g_com_arduino_do_shift_out_msb_first(uint8_t val)
{
  uint8_t cnt = 8;
  U8G_ATOMIC_START();
  do
  {
    if ( val & 128 )
	*dog_outData |= dog_bitData;
    else
	*dog_outData &= dog_bitNotData;    
    val <<= 1;
    /*
	There must be some delay here. However
	fetching the adress dog_outClock is enough delay, so
	do not place dog_outClock in a local variable. This will
	break the procedure
    */
    *dog_outClock |= dog_bitClock;
    cnt--;
    *dog_outClock &= dog_bitNotClock;
    /* 
	little additional delay after clk pulse, done by 3x32bit reads 
	from I/O. Optimized for PIC32 with 80 MHz.
    */
    dog_pic32_spi_tmp = *dog_outClock;
    dog_pic32_spi_tmp = *dog_outClock;
    dog_pic32_spi_tmp = *dog_outClock;
  } while( cnt != 0 );
  U8G_ATOMIC_END();
}

/*=========================================================*/
/* Arduino Due */
#elif defined(__SAM3X8E__)

/* Due */

void u8g_digital_write_sam_high(uint8_t pin)
{
    PIO_Set( g_APinDescription[pin].pPort, g_APinDescription[pin].ulPin) ;
}

void u8g_digital_write_sam_low(uint8_t pin)
{
    PIO_Clear( g_APinDescription[pin].pPort, g_APinDescription[pin].ulPin) ;
}

static uint8_t u8g_sam_data_pin;
static uint8_t u8g_sam_clock_pin;

static void u8g_com_arduino_init_shift_out(uint8_t dataPin, uint8_t clockPin)
{
  u8g_sam_data_pin = dataPin;
  u8g_sam_clock_pin = clockPin;
}

static void u8g_com_arduino_do_shift_out_msb_first(uint8_t val)
{
  uint8_t i = 8;
  do
  {
    if ( val & 128 )
      u8g_digital_write_sam_high(u8g_sam_data_pin);
    else
      u8g_digital_write_sam_low(u8g_sam_data_pin);
    val <<= 1;
    //u8g_MicroDelay();	
    u8g_digital_write_sam_high(u8g_sam_clock_pin);
    u8g_MicroDelay();	
    u8g_digital_write_sam_low(u8g_sam_clock_pin);
    u8g_MicroDelay();	
    i--;
  } while( i != 0 );
}


#else
/* empty interface */

static void u8g_com_arduino_init_shift_out(uint8_t dataPin, uint8_t clockPin)
{
}

static void u8g_com_arduino_wr_null_byte()
{
    //WRITE(DOGLCD_MOSI,LOW);
    SET_LOW(DOGLCD_MOSI);
    uint8_t i = 8;
    do
    {
    //WRITE(DOGLCD_SCK, HIGH);
    //WRITE(DOGLCD_SCK, LOW);
    SET_HIGH(DOGLCD_SCK);
    SET_LOW(DOGLCD_SCK);
    i--;
    } while( i != 0 );
}

static void u8g_com_arduino_do_shift_out_msb_first(uint8_t val)
{
  uint8_t i = 8;
  do
  {
    if ( val & 128 )
    {
      SET_HIGH(DOGLCD_MOSI);
      //WRITE(DOGLCD_MOSI,HIGH);
    }
    else
    {
      SET_LOW(DOGLCD_MOSI);
      //WRITE(DOGLCD_MOSI,LOW);
    }
    val <<= 1;
    //u8g_MicroDelay();	
    //WRITE(DOGLCD_SCK, HIGH);
    //u8g_MicroDelay();	
    //WRITE(DOGLCD_SCK, LOW);
    //u8g_MicroDelay();	
    SET_HIGH(DOGLCD_SCK);
    SET_LOW(DOGLCD_SCK);
    i--;
  } while( i != 0 );
}

#endif 

uint8_t u8g_com_arduino_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  switch(msg)
  {
    case U8G_COM_MSG_INIT:
      //u8g_com_arduino_assign_pin_output_high(u8g);
      u8g_com_arduino_digital_write(u8g, DOGLCD_SCK, LOW);
      u8g_com_arduino_digital_write(u8g, DOGLCD_MOSI, LOW);
      //u8g_com_arduino_init_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK]);
      break;
    
    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_RESET:
      WRITE(DOGLCD_CS, HIGH);
      WRITE(DOGLCD_SCK, HIGH);
      WRITE(DOGLCD_MOSI, HIGH);
      WRITE(DOGLCD_A0, HIGH);
      //if ( u8g->pin_list[U8G_PI_RESET] != U8G_PIN_NONE )
      //  u8g_com_arduino_digital_write(u8g, U8G_PI_RESET, arg_val);
      break;
      
    case U8G_COM_MSG_CHIP_SELECT:
      if ( arg_val == 0 )
      {
        /* disable */
        u8g_com_arduino_digital_write(u8g, DOGLCD_CS, HIGH);
      }
      else
      {
        /* enable */
        u8g_com_arduino_digital_write(u8g, DOGLCD_SCK, LOW);
        u8g_com_arduino_digital_write(u8g, DOGLCD_CS, LOW);
	/* issue 227 */
	//u8g_com_arduino_init_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK]);
      }
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      u8g_com_arduino_do_shift_out_msb_first( arg_val );
      //u8g_arduino_sw_spi_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK], arg_val);
      break;
    
    case U8G_COM_MSG_WRITE_SEQ:
      {
        register uint8_t *ptr = arg_ptr;
        while( arg_val > 0 )
        {
          u8g_com_arduino_do_shift_out_msb_first(*ptr++);
          // u8g_arduino_sw_spi_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK], *ptr++);
          arg_val--;
        }
      }
      break;

      case U8G_COM_MSG_WRITE_SEQ_P:
      {
        register uint8_t *ptr = arg_ptr;
        while( arg_val > 0 )
        {
          u8g_com_arduino_do_shift_out_msb_first( u8g_pgm_read(ptr) );
          //u8g_arduino_sw_spi_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK], u8g_pgm_read(ptr));
          ptr++;
          arg_val--;
        }
      }
      break;
      
    case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g_com_arduino_digital_write(u8g, DOGLCD_A0, arg_val);
      break;
  }
  return 1;
}

#else /* ARDUINO */

uint8_t u8g_com_arduino_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  return 1;
}

#endif /* ARDUINO */

