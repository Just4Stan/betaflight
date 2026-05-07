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

// Single-include shim for ASSERT_CORE0() / ASSERT_CORE1(). On
// USE_MULTICORE targets this expands to the platform's hard_assert
// against get_core_num(); on every other target it's a no-op. This
// avoids repeating the same #ifdef block in gyro.c, pid.c, scheduler.c,
// and (newly) flight/sysid.c.

#pragma once

#ifdef USE_MULTICORE
#include "platform/multicore.h"
#else
#define ASSERT_CORE0() ((void)0)
#define ASSERT_CORE1() ((void)0)
#endif
