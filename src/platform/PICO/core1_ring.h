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
#include <stdint.h>
#include <stddef.h>

#ifdef USE_MULTICORE

/*
 * Lockless single-producer / single-consumer ring buffer for the BF PICO
 * platform. Used by multicoreScheduleTask() to dispatch work from core0
 * (the producer side, e.g. the gyro task) onto core1 (the consumer side,
 * running core1_main()). Hand-rolled because the pico-sdk queue_t uses
 * mutex locks internally and is far too heavy for an 8 kHz gyro path.
 *
 * Concurrency model:
 *   - head index is written ONLY by the producer (core0).
 *   - tail index is written ONLY by the consumer (core1).
 *   - Buffer storage lives in normal BSS today. SCRATCH_X placement
 *     (one 4 KB block per core, no cross-core cache contention) is a
 *     future optimisation that needs a linker-script change; the
 *     SPSC fences below are correct on shared SRAM regardless.
 *   - On the M33 a plain aligned uint32_t store/load is atomic; we add
 *     compiler barriers via __atomic_thread_fence() to keep the producer
 *     from publishing the head before the payload is committed and the
 *     consumer from reading the payload before observing the new head.
 *
 * Capacity must be a power of two. Indices wrap on the ring size by
 * masking against (CAPACITY-1); the head/tail integers themselves are
 * free-running uint32_t so an empty/full distinction is cheap:
 *     empty when head == tail
 *     full  when head - tail == CAPACITY
 *
 * The element type is a generic 32-bit slot so the same ring can carry
 * either a downsampled gyro sample or a packed (axis|sample) word.
 * Higher-rate consumers (dyn_notch on core1, future chirp analyser) will
 * use this same primitive.
 */

#define CORE1_RING_CAPACITY 64u  // power of two; ~8 ms of headroom at 8 kHz

typedef uint32_t core1_ring_slot_t;

typedef struct core1_ring_s {
    volatile uint32_t head;     // producer-only writer
    volatile uint32_t tail;     // consumer-only writer
    core1_ring_slot_t buf[CORE1_RING_CAPACITY];
} core1_ring_t;

void core1_ring_init(core1_ring_t *r);

// Producer (core0). Returns false if the ring is full and the slot was
// dropped. Callers in the gyro path should treat this as "noisy log" and
// not block.
bool core1_ring_push(core1_ring_t *r, core1_ring_slot_t v);

// Consumer (core1). Returns false if the ring is empty.
bool core1_ring_pop(core1_ring_t *r, core1_ring_slot_t *out);

// Number of slots currently occupied. Producer-side consistent (the
// consumer may have already popped one by the time the call returns).
uint32_t core1_ring_size(const core1_ring_t *r);

#endif // USE_MULTICORE
