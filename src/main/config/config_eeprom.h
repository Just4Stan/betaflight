/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define EEPROM_CONF_VERSION 178
// PR-D adds OSD_SYSID_PID at the END of osd_items_e (no shift of existing
// values) and a new sysidConfig group with its own PG_REGISTER. PG-load
// gracefully zero-pads new fields when reading a shorter saved EEPROM, so
// no version bump is required to land USE_SYSID as an opt-in feature.

bool isEEPROMVersionValid(void);
bool isEEPROMStructureValid(void);
bool loadEEPROM(void);
void writeConfigToEEPROM(void);

uint16_t getEEPROMConfigSize(void);
size_t getEEPROMStorageSize(void);

bool saveEEPROMToSDCard(void);
void saveEEPROMToMemoryMappedFlash(void);
