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

#pragma once

// Core-affinity invariant macros. The hard-realtime gyro/PID/scheduler paths
// in BF run on core0; core1 is reserved for best-effort offload work
// registered through multicoreScheduleTask(). These macros let those paths
// state the contract at the call site without leaking platform headers into
// generic code.
//
// On USE_MULTICORE targets (PICO RP2350) they hard-assert that the current
// core matches the expected one. On every other target they expand to
// nothing, so STM32/SITL builds see no overhead and no behavioural change.

#ifdef USE_MULTICORE
#include "platform/multicore.h"
#define ASSERT_CORE0() PLATFORM_ASSERT_CORE0()
#define ASSERT_CORE1() PLATFORM_ASSERT_CORE1()
#else
#define ASSERT_CORE0() ((void)0)
#define ASSERT_CORE1() ((void)0)
#endif
