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

#ifdef __STM32F4__

#include "../../inc/MarlinConfig.h"

// This is for EEPROM emulation in flash
#if ENABLED(EEPROM_SETTINGS) && ENABLED(FLASH_EEPROM_EMULATION)

#include "../persistent_store_api.h"

#define EEPROM_START_ADDRESS	0x08004000//((uint32)(0x8000000 + 128 * 1024 - 2 * EEPROM_PAGE_SIZE))
#define EEPROM_PAGE0_BASE		0x08004000//((uint32)(EEPROM_START_ADDRESS + 0x000))
#define EEPROM_SECTOR           FLASH_SECTOR_1

//#define HAL_STM32F1_EEPROM_SIZE 1024
//char HAL_STM32F1_eeprom_content[HAL_STM32F1_EEPROM_SIZE];
__no_init uint8_t eeprom[1024*16]@EEPROM_START_ADDRESS;

namespace HAL {
namespace PersistentStore {

// Store settings in the last two pages
// Flash pages must be erased before writing, so keep track.
bool firstWrite = true;

bool access_start() {
  return true;
}

bool access_finish(){
  firstWrite = true;
  __HAL_FLASH_DATA_CACHE_ENABLE();
  return true;
}

bool write_data(int &pos, const uint8_t *value, uint16_t size, uint16_t *crc) {
  if (firstWrite)
  {
    __HAL_FLASH_DATA_CACHE_DISABLE();

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    FLASH_Erase_Sector(EEPROM_SECTOR,FLASH_VOLTAGE_RANGE_3);

    firstWrite=false;
  }

  for (int i = 0; i < size; i++) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,EEPROM_START_ADDRESS+(pos+i),value[i]);
    if (eeprom[pos + i] != (value[i]))
    {
      SERIAL_ECHOLNPGM("ERR");
    }
  }
  crc16(crc, value, size);
  pos += size;
  return true;
}

bool read_data(int &pos, uint8_t* value, uint16_t size, uint16_t *crc, const bool writing/*=true*/) {
  for (int i = 0; i < size; i++) {
    uint8_t c = eeprom[pos + i];
    if (writing) value[i] = c;
    crc16(crc, &c, 1);
  }
  pos += size;
  return true;
}

} // PersistentStore
} // HAL

void eeprom_read_block (void *__dst, const void *__src, size_t __n)
{
    uint32_t S = (uint32_t)__src;
    uint8_t *D = (uint8_t *)__dst;
    while (__n--) {
        *D = eeprom[S];
        D++; S++;
    }
}

#endif // EEPROM_SETTINGS && EEPROM FLASH
#endif // __STM32F1__
