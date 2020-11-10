/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
 * Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 * Updated with backports of the latest SdFat library from the same author
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "../inc/MarlinConfig.h"

extern "C" uint8_t BSP_SD_Init();
extern "C" uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
extern "C" uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
extern "C" uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
extern "C" uint8_t BSP_SD_GetCardState(void);
#define   SD_TRANSFER_OK                ((uint8_t)0x00)
#define   SD_TRANSFER_BUSY              ((uint8_t)0x01)

#include "Sd2Card.h"
#include "../Marlin.h"

/**
 * Initialize an SD flash memory card.
 *
 * \param[in] sckRateID SPI clock rate selector. See setSckRate().
 * \param[in] chipSelectPin SD chip select pin number.
 *
 * \return true for success, false for failure.
 * The reason for failure can be determined by calling errorCode() and errorData().
 */
bool Sd2Card::init(uint8_t sckRateID, pin_t chipSelectPin) {
  errorCode_ = type_ = 0;
  // 16-bit init start time allows over a minute
  // If init takes more than 4s it could trigger
  // watchdog leading to a reboot loop.
  #if ENABLED(USE_WATCHDOG)
    watchdog_reset();
  #endif
  if (BSP_SD_Init()==0) return true;
  return false;
}

/**
 * Read a 512 byte block from an SD card.
 *
 * \param[in] blockNumber Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data.
 * \return true for success, false for failure.
 */
bool Sd2Card::readBlock(uint32_t blockNumber, uint8_t* dst) {
    if (BSP_SD_ReadBlocks((uint32_t*)dst,blockNumber,1,1000)==0) return true;
    return false;
}

/**
 * Writes a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return true for success, false for failure.
 */
bool Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src) {
  /*for (uint32_t t=0;t<1000;t++)
  {
        if (BSP_SD_GetCardState()==SD_TRANSFER_OK) break;
  }*/
  if (BSP_SD_WriteBlocks((uint32_t*)src,blockNumber,1,1000)==0) return true;
    return false;
  return false;
}

