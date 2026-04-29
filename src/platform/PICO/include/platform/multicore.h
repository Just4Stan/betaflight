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

#include <stdbool.h>

#include "pico/multicore.h"

typedef enum multicoreCommand_e {
    MULTICORE_CMD_NONE = 0,
    MULTICORE_CMD_FUNC,
    MULTICORE_CMD_FUNC_BLOCKING, // Command to execute a function on the second core and wait for completion
    MULTICORE_CMD_STOP, // Command to stop the second core
} multicoreCommand_e;

// Define function types for clarity
typedef void core1_func_t(void);

void multicoreStart(void);
void multicoreStop(void);
void multicoreExecute(core1_func_t *func);
void multicoreExecuteBlocking(core1_func_t *func);

// -----------------------------------------------------------------------------
// Lightweight scheduled-task API for core1 (Phase 1 multicore work).
// -----------------------------------------------------------------------------
//
// multicoreExecute*() above is fire-and-forget / blocking-on-completion: every
// call enqueues a function pointer and the consumer side runs it once. That is
// fine for one-shot work (config-flash writes, init steps) but is the wrong
// shape for a sustained per-loop workload such as moving dyn_notch SDFT or a
// future chirp analyser onto core1.
//
// multicoreScheduleTask() registers a long-lived task whose `update()` is
// invoked from core1_main() in steady-state, alongside the existing fire-and-
// forget queue. Tasks are expected to do their own internal pacing (read from
// a SPSC ring of samples produced by core0, drain whatever is available, then
// return). Tasks remain registered for the lifetime of the firmware run.
//
// API contract:
//   - multicoreScheduleTask() must be called before multicoreStart() (i.e.
//     during BF init, before core1 is launched). Calls after the core has
//     started are ignored to keep the consumer-side data structures lockless.
//   - The total number of registered tasks is bounded (MULTICORE_MAX_TASKS).
//   - Tasks must be reentrancy-safe with their own producer side; the ring
//     primitives in core1_ring.h provide the SPSC pattern.

#define MULTICORE_MAX_TASKS 4

typedef void (*multicore_task_update_fn)(void);

typedef struct multicore_task_s {
    multicore_task_update_fn update;
    const char *name;   // for debug / future telemetry; may be NULL
} multicore_task_t;

bool multicoreScheduleTask(const multicore_task_t *task);

// -----------------------------------------------------------------------------
// Core-affinity invariant assertions
// -----------------------------------------------------------------------------
//
// Hard-realtime gyro/PID/scheduler code paths must always run on core0; core1
// is reserved for best-effort offload (multicoreScheduleTask consumers, the
// fire-and-forget queue). Adding asserts at the entry of those paths makes
// the invariant a build-time-checkable contract and surfaces accidental
// regressions immediately rather than as mysterious flight glitches.
//
// Use the wrapper macros in build/core_affinity.h (which works on every
// target) rather than calling these directly.
#include "pico/platform.h"  // for hard_assert + get_core_num
#define PLATFORM_ASSERT_CORE0() hard_assert(get_core_num() == 0)
#define PLATFORM_ASSERT_CORE1() hard_assert(get_core_num() == 1)
