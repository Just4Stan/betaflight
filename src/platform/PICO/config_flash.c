/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>

#include "platform.h"
#include "drivers/system.h"
#include "config/config_streamer.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#ifdef USE_MULTICORE
#include "pico/flash.h"
#endif

#if defined(CONFIG_IN_FLASH)

void configUnlock(void)
{
    // NOOP
}

void configLock(void)
{
    // NOOP
}

void configClearFlags(void)
{
    // NOOP
}

typedef struct {
    uint32_t flash_offs;
    const void *buffer;
} flashWriteParam_t;

static void doFlashWrite(void *p)
{
    const flashWriteParam_t *param = (const flashWriteParam_t *)p;

    if ((param->flash_offs % FLASH_SECTOR_SIZE) == 0) {
        // Erase the flash sector before writing
        flash_range_erase(param->flash_offs, FLASH_SECTOR_SIZE);
    }

    flash_range_program(param->flash_offs, param->buffer, CONFIG_STREAMER_BUFFER_SIZE);
}

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == sizeof(config_streamer_buffer_type_t) * CONFIG_STREAMER_BUFFER_SIZE,  "CONFIG_STREAMER_BUFFER_SIZE does not match written size");

    // pico-sdk flash_range functions use the offset from start of FLASH
    const flashWriteParam_t param = {
        .flash_offs = address - XIP_BASE,
        .buffer     = buffer,
    };

#ifdef USE_MULTICORE
    // Park core1 (which must have called flash_safe_execute_core_init() at startup)
    // and disable interrupts on the calling core before erasing/programming flash.
    // Without this, the other core executing from XIP during the program/erase
    // window would corrupt or hang the system.
    if (flash_safe_execute(doFlashWrite, (void *)&param, UINT32_MAX) != PICO_OK) {
        return CONFIG_RESULT_FAILURE;
    }
#else
    uint32_t interrupts = save_and_disable_interrupts();
    doFlashWrite((void *)&param);
    restore_interrupts(interrupts);
#endif

    return CONFIG_RESULT_SUCCESS;
}

#endif // CONFIG_IN_FLASH
