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

#include "platform.h"
#include "platform/multicore.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "pico/flash.h"

#ifdef USE_MULTICORE

// Define a structure for the message we'll pass through the queue
typedef struct {
    multicoreCommand_e command;
    core1_func_t *func;
} core_message_t;

// Define the queue
static queue_t core0_queue;
static queue_t core1_queue;

// -------------------- scheduled-task table --------------------
//
// Populated by multicoreScheduleTask(). The original PR2 contract was
// "register before multicoreStart()" so the table could stay lockless;
// in practice every BF init path runs *after* multicoreStart() (because
// multicoreExecuteBlocking() is what dispatches initPhase1/2/3 onto
// core1 to begin with).
//
// CONCURRENCY MODEL — single producer, single consumer:
//   - Producer: core0 only. All BF init paths that register tasks run
//     either before multicoreStart() or inside initPhaseN which is
//     itself dispatched serially via multicoreExecuteBlocking. There is
//     never a second producer in BF today.
//   - Consumer: core1 main loop only.
//   - Slots are write-once: a task pointer is stored, then the count
//     is bumped. The count is monotonic. core1 reads the count atomically
//     and then walks slots [0, count); slots in that range are
//     guaranteed published.
//
// If a future caller wants concurrent registrations from core1 or from
// multiple core0 contexts, this needs a CAS on the count plus a write
// barrier — at which point a small spinlock would be cleaner than the
// current relaxed-atomic dance.

static const multicore_task_t *scheduled_tasks[MULTICORE_MAX_TASKS];
static volatile uint8_t scheduled_task_count = 0;
static volatile bool core1_running = false;

bool multicoreScheduleTask(const multicore_task_t *task)
{
    if (task == NULL || task->update == NULL) {
        return false;
    }
    const uint8_t n = scheduled_task_count;
    if (n >= MULTICORE_MAX_TASKS) {
        return false;
    }
    scheduled_tasks[n] = task;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&scheduled_task_count, (uint8_t)(n + 1), __ATOMIC_RELEASE);
    return true;
}

static inline void core1_run_scheduled_tasks(void)
{
    const uint8_t n = __atomic_load_n(&scheduled_task_count, __ATOMIC_ACQUIRE);
    for (uint8_t i = 0; i < n; i++) {
        const multicore_task_t *t = scheduled_tasks[i];
        if (t && t->update) {
            t->update();
        }
    }
}

static void core1_main(void)
{
    // Register this core as a flash_safe_execute lockout victim so that core0
    // can safely erase/program flash (e.g. config writes) without this core
    // executing from XIP during the operation.
    flash_safe_execute_core_init();

    while (true) {

        core_message_t msg;
        if (queue_try_remove(&core1_queue, &msg)) {
            switch (msg.command) {
            case MULTICORE_CMD_FUNC:
                if (msg.func) {
                    msg.func();
                }
                break;
            case MULTICORE_CMD_FUNC_BLOCKING:
                if (msg.func) {
                    msg.func();

                    // Send the result back to core0 (it will be blocking until this is done)
                    bool result = true;
                    queue_add_blocking(&core0_queue, &result);
                }
                break;
            case MULTICORE_CMD_STOP:
                multicore_reset_core1();
                return; // Exit the core1_main function
            default:
                // unknown command or none
                break;
            }
        }

        core1_run_scheduled_tasks();

        tight_loop_contents();
    }
}

void multicoreStart(void)
{
    // Initialize the queue with a size of 4 (to be determined based on expected load)
    queue_init(&core1_queue, sizeof(core_message_t), 4);

    // Initialize the queue with a size of 1 (only needed for blocking results)
    queue_init(&core0_queue, sizeof(bool), 1);

    // Start core 1
    core1_running = true;
    multicore_launch_core1(core1_main);
}

void multicoreStop(void)
{
    core_message_t msg;
    msg.command = MULTICORE_CMD_STOP;
    msg.func = NULL;

    queue_add_blocking(&core1_queue, &msg);
 }
#endif // USE_MULTICORE


void multicoreExecuteBlocking(core1_func_t *func)
{
#ifdef USE_MULTICORE
    core_message_t msg;
    msg.command = MULTICORE_CMD_FUNC_BLOCKING;
    msg.func = func;

    bool result;

    queue_add_blocking(&core1_queue, &msg);
    // Wait for the command to complete
    queue_remove_blocking(&core0_queue, &result);
#else
    // If multicore is not used, execute the command directly
    if (func) {
        func();
    }
#endif // USE_MULTICORE
}

void multicoreExecute(core1_func_t *func)
{
#ifdef USE_MULTICORE
    core_message_t msg;
    msg.command = MULTICORE_CMD_FUNC;
    msg.func = func;

    queue_add_blocking(&core1_queue, &msg);
#else
    // If multicore is not used, execute the command directly
    if (func) {
        func();
    }
#endif // USE_MULTICORE
}

