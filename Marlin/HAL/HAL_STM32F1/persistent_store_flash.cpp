/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2016 Victor Perez victor_pv@hotmail.com
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
 * persistent_store_flash.cpp
 * HAL for stm32duino and compatible (STM32F1)
 * Implementation of EEPROM settings in SDCard
 */

#ifdef __STM32F1__

#include "../../inc/MarlinConfig.h"

// This is for EEPROM emulation in flash
#if ENABLED(EEPROM_SETTINGS) && ENABLED(FLASH_EEPROM_EMULATION)

#include "../persistent_store_api.h"

#include <flash_stm32.h>
#include <EEPROM.h>

namespace HAL {
namespace PersistentStore {
//#define HAL_STM32F1_EEPROM_SIZE 1024
//char HAL_STM32F1_eeprom_content[HAL_STM32F1_EEPROM_SIZE];
__no_init uint16_t eeprom[1024]@EEPROM_START_ADDRESS;
// Store settings in the last two pages
// Flash pages must be erased before writing, so keep track.
bool firstWrite = true;

bool access_start() {
  return true;
}

bool access_finish(){
  firstWrite = true;
  FLASH_Lock();
  return true;
}

bool write_data(int &pos, const uint8_t *value, uint16_t size, uint16_t *crc) {
  if (firstWrite)
  {
    RCC_HSICmd(ENABLE);

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    FLASH_ErasePage(EEPROM_PAGE0_BASE);
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    FLASH_ErasePage(EEPROM_PAGE1_BASE);
    //memset(eeprom,0xFFFF,2048);
    
    firstWrite=false;
  }
  
  for (int i = 0; i < size; i++) {
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    //unsigned int p = EEPROM_START_ADDRESS+((pos+i)*2);
    FLASH_ProgramHalfWord(EEPROM_START_ADDRESS+((pos+i)*2),value[i]|0xFF00);
    //eeprom[pos + i] = value[i]|0xFF00;
    if (eeprom[pos + i] != (value[i]|0xFF00))
    {
      //verify error
      SERIAL_ECHOLNPGM("ERR");
    }
  }
  crc16(crc, value, size);
  pos += size;
  return true;
}

bool read_data(int &pos, uint8_t* value, uint16_t size, uint16_t *crc, const bool writing/*=true*/) {
  for (int i = 0; i < size; i++) {
    uint8_t c = eeprom[pos + i]&0xFF;
    if (writing) value[i] = c;
    crc16(crc, &c, 1);
  }
  pos += size;
  return true;
  /*for (uint16_t i = 0; i < size; i++) {
    byte* accessPoint = (byte*)(pageBase + pos + i);
    uint8_t c = *accessPoint;
    if (writing) value[i] = c;
    crc16(crc, &c, 1);
  }
  pos += ((size + 1) & ~1);*/
}

} // PersistentStore
} // HAL

#endif // EEPROM_SETTINGS && EEPROM FLASH
#endif // __STM32F1__
