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

#ifdef USE_MULTICORE

#include "core1_ring.h"

void core1_ring_init(core1_ring_t *r)
{
    r->head = 0;
    r->tail = 0;
    __atomic_thread_fence(__ATOMIC_RELEASE);
}

bool core1_ring_push(core1_ring_t *r, core1_ring_slot_t v)
{
    const uint32_t head = r->head;
    const uint32_t tail = __atomic_load_n(&r->tail, __ATOMIC_ACQUIRE);
    if ((head - tail) >= CORE1_RING_CAPACITY) {
        // Full — caller decides how to handle (drop is fine for sample paths).
        return false;
    }
    r->buf[head & (CORE1_RING_CAPACITY - 1u)] = v;
    __atomic_store_n(&r->head, head + 1u, __ATOMIC_RELEASE);
    return true;
}

bool core1_ring_pop(core1_ring_t *r, core1_ring_slot_t *out)
{
    const uint32_t tail = r->tail;
    const uint32_t head = __atomic_load_n(&r->head, __ATOMIC_ACQUIRE);
    if (head == tail) {
        return false;
    }
    *out = r->buf[tail & (CORE1_RING_CAPACITY - 1u)];
    __atomic_store_n(&r->tail, tail + 1u, __ATOMIC_RELEASE);
    return true;
}

uint32_t core1_ring_size(const core1_ring_t *r)
{
    const uint32_t head = __atomic_load_n(&r->head, __ATOMIC_ACQUIRE);
    const uint32_t tail = __atomic_load_n(&r->tail, __ATOMIC_ACQUIRE);
    return head - tail;
}

#endif // USE_MULTICORE
