/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2017 Victor Perez
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
 * Software SPI functions originally from Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 */

/**
 * Adapted to the STM32F1 HAL
 */

#ifdef __STM32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../HAL.h"
#include "SPI.h"
#include "pins_arduino.h"
#include "spi_pins.h"
#include "../../core/macros.h"

#include "stm32f0xx.h"

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

//static SPISettings spiConfig;

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

#if ENABLED(SOFTWARE_SPI)
  // --------------------------------------------------------------------------
  // Software SPI
  // --------------------------------------------------------------------------
  //#error "Software SPI not supported for STM32F1. Use hardware SPI."
void spiBegin() {
  GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
        
  #if !PIN_EXISTS(SS)
    #error "SS_PIN not defined!"
  #endif
  WRITE(SS_PIN, HIGH);
  SET_OUTPUT(SS_PIN);
}
#define nop asm volatile ("\tnop\n")
void spiInit(uint8_t spiRate) {
    // nothing to do
    UNUSED(spiRate);
  }

  /** Begin SPI transaction, set clock, bit order, data mode */
  void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) {
    // nothing to do
    UNUSED(spiBeginTransaction);
  }

  //------------------------------------------------------------------------------
  /** Soft SPI receive byte */
  uint8_t spiRec() {
    uint8_t data = 0;
    // no interrupts during byte receive - about 8 us
    cli();
    // output pin high - like sending 0xFF
    WRITE(MOSI_PIN, HIGH);

    for (uint8_t i = 0; i < 8; i++) {
      WRITE(SCK_PIN, HIGH);

      // adjust so SCK is nice
      nop;
      nop;

      data <<= 1;

      if (READ(MISO_PIN)) data |= 1;

      WRITE(SCK_PIN, LOW);
    }
    // enable interrupts
    sei();
    return data;
  }
  //------------------------------------------------------------------------------
  /** Soft SPI read data */
  void spiRead(uint8_t* buf, uint16_t nbyte) {
    for (uint16_t i = 0; i < nbyte; i++)
      buf[i] = spiRec();
  }
  //------------------------------------------------------------------------------
  /** Soft SPI send byte */
  void spiSend(uint8_t data) {
    // no interrupts during byte send - about 8 us
    cli();
    for (uint8_t i = 0; i < 8; i++) {
      WRITE(SCK_PIN, LOW);

      WRITE(MOSI_PIN, data & 0x80);

      data <<= 1;

      WRITE(SCK_PIN, HIGH);
    }
    // hold SCK high for a few ns
    nop;
    nop;
    nop;
    nop;

    WRITE(SCK_PIN, LOW);
    // enable interrupts
    sei();
  }
  //------------------------------------------------------------------------------
  /** Soft SPI send block */
  void spiSendBlock(uint8_t token, const uint8_t* buf) {
    spiSend(token);
    for (uint16_t i = 0; i < 512; i++)
      spiSend(buf[i]);
  }
#else

// --------------------------------------------------------------------------
// Hardware SPI
// --------------------------------------------------------------------------

void spiBegin() {
  GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_PinAFConfig( GPIOB , GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5, GPIO_AF_0 );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_0);
    
    SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
    
    SPI_InitTypeDef SPI_InitStructure = {0};
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    SPI_Init(SPI1, &SPI_InitStructure);
    
    SPI_Cmd(SPI1, ENABLE);
    
  #if !PIN_EXISTS(SS)
    #error "SS_PIN not defined!"
  #endif
  WRITE(SS_PIN, HIGH);
  SET_OUTPUT(SS_PIN);
}

/**
 * @brief  Initializes SPI port to required speed rate and transfer mode (MSB, SPI MODE 0)
 *
 * @param  spiRate Rate as declared in HAL.h (speed do not match AVR)
 * @return Nothing
 *
 * @details
 */
void spiInit(uint8_t spiRate) {
    SPI_InitTypeDef SPI_InitStructure = {0};
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    
    switch (spiRate) {
        case SPI_FULL_SPEED:    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; break;
        case SPI_HALF_SPEED:    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; break;
        case SPI_QUARTER_SPEED: SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; break;
        case SPI_EIGHTH_SPEED:  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; break;
        case SPI_SPEED_5:       SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128; break;
        case SPI_SPEED_6:       SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128; break;
        default:                SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // Default from the SPI library
        }
    SPI_Init(SPI1, &SPI_InitStructure);
}

/**
 * @brief  Receives a single byte from the SPI port.
 *
 * @return Byte received
 *
 * @details
 */
uint8_t spiRec(void) {
  unsigned char timeout=0;
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) != SET)
  {
    if (timeout++>500) break;
  }
  SPI_SendData8(SPI1, 0xFF);
  timeout=0;
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) != SET)
  {
    if (timeout++>500) break;
  }
  return SPI_ReceiveData8(SPI1);
/*  SPI.beginTransaction(spiConfig);
  uint8_t returnByte = SPI.transfer(0xFF);
  SPI.endTransaction();
  return returnByte;*/
}

/**
 * @brief  Receives a number of bytes from the SPI port to a buffer
 *
 * @param  buf   Pointer to starting address of buffer to write to.
 * @param  nbyte Number of bytes to receive.
 * @return Nothing
 *
 * @details Uses DMA
 */
void spiRead(uint8_t* buf, uint16_t nbyte) {
  for (uint16_t i=0;i<nbyte;i++) buf[i]=spiRec();
  /*SPI.beginTransaction(spiConfig);
  SPI.dmaTransfer(0, const_cast<uint8*>(buf), nbyte);
  SPI.endTransaction();*/
}

/**
 * @brief  Sends a single byte on SPI port
 *
 * @param  b Byte to send
 *
 * @details
 */
void spiSend(uint8_t b) {
  unsigned char timeout;

  timeout=0;
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) != SET)
  {
    if (timeout++>500) break;
  }
  SPI_SendData8(SPI1, b);
  /*SPI.beginTransaction(spiConfig);
  SPI.send(b);
  SPI.endTransaction();*/
}

/**
 * @brief  Write token and then write from 512 byte buffer to SPI (for SD card)
 *
 * @param  buf   Pointer with buffer start address
 * @return Nothing
 *
 * @details Use DMA
 */
void spiSendBlock(uint8_t token, const uint8_t* buf) {
  spiSend(token);
  for (uint16_t i=0;i<512;i++) spiSend(buf[i]);
  
  /*SPI.beginTransaction(spiConfig);
  SPI.send(token);
  SPI.dmaSend(const_cast<uint8*>(buf), 512);
  SPI.endTransaction();*/
}

/** Begin SPI transaction, set clock, bit order, data mode */
void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) {
  /*spiConfig = SPISettings(spiClock, bitOrder, dataMode);

  SPI.beginTransaction(spiConfig);*/
}

#endif // SOFTWARE_SPI

#endif // __STM32F1__
