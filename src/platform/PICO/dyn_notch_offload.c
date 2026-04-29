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

// PICO RP2350 implementation of the dyn_notch offload interface (see
// flight/dyn_notch_offload.h). Producer (gyro task on core0) submits the
// pre-windowed SDFT spectrum + algorithm parameters; the consumer (core1
// scheduled task) runs the peak-detection and center-frequency calculation
// steps and stores the result back via atomics. Core0 polls the result and
// applies it in STEP_UPDATE_FILTERS.

#include "platform.h"

#if defined(USE_MULTICORE) && defined(USE_DYN_NOTCH_FILTER)

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "platform/multicore.h"
#include "core1_ring.h"

#include "flight/dyn_notch_offload.h"
#include "flight/dyn_notch_filter.h"

// Job slots are kept in a small fixed-size pool; the ring carries 1-based
// slot indices (0 reserved for "empty"), avoiding pointer transfers across
// the SPSC primitive.
#define DYN_NOTCH_OFFLOAD_SLOTS 4

typedef struct {
    volatile uint32_t in_use;        // atomic 0/1, producer-claimed
    dynNotchOffloadJob_t job;
} jobSlot_t;

static jobSlot_t s_slots[DYN_NOTCH_OFFLOAD_SLOTS];
static core1_ring_t s_ring;

// Per-axis result, written by core1, consumed by core0.
typedef struct {
    volatile uint32_t ready;          // atomic 0/1
    dynNotchOffloadResult_t result;
} axisResult_t;

static axisResult_t s_results[XYZ_AXIS_COUNT];

static volatile bool s_active = false;

// ---------------- Core1 worker ----------------

static void runJob(const dynNotchOffloadJob_t *j)
{
    const int count = j->peakCount;
    const float *sdftData = j->sdftData;

    int peakBins[DYN_NOTCH_COUNT_MAX] = {0};
    float peakValues[DYN_NOTCH_COUNT_MAX] = {0};

    // -------- STEP_DETECT_PEAKS --------
    for (int bin = j->startBin + 1; bin < j->endBin; bin++) {
        if ((sdftData[bin] > sdftData[bin - 1]) && (sdftData[bin] > sdftData[bin + 1])) {
            for (int p = 0; p < count; p++) {
                if (sdftData[bin] > peakValues[p]) {
                    for (int k = count - 1; k > p; k--) {
                        peakBins[k] = peakBins[k - 1];
                        peakValues[k] = peakValues[k - 1];
                    }
                    peakBins[p] = bin;
                    peakValues[p] = sdftData[bin];
                    break;
                }
            }
            bin++; // skip neighbour - can't be a peak too
        }
    }

    // Sort N peaks by ascending bin (matches inline state-machine behaviour)
    for (int p = count - 1; p > 0; p--) {
        for (int k = 0; k < p; k++) {
            if (peakBins[k] > peakBins[k + 1] && peakBins[k + 1] != 0) {
                int   tb = peakBins[k];   peakBins[k] = peakBins[k + 1];   peakBins[k + 1] = tb;
                float tv = peakValues[k]; peakValues[k] = peakValues[k + 1]; peakValues[k + 1] = tv;
            }
        }
    }

    // -------- STEP_CALC_FREQUENCIES --------
    float noiseThreshold = j->noiseThresholdInitial;
    int peakCountActive = 0;
    for (int p = 0; p < count; p++) {
        if (peakBins[p] != 0) {
            noiseThreshold -= 0.75f * sdftData[peakBins[p] - 1];
            noiseThreshold -= sdftData[peakBins[p]];
            noiseThreshold -= 0.75f * sdftData[peakBins[p] + 1];
            peakCountActive++;
        }
    }
    noiseThreshold /= (j->endBin - j->startBin) - peakCountActive + 1;
    noiseThreshold *= 2.0f;

    float centerFreqOut[DYN_NOTCH_COUNT_MAX];
    int   maxCenterFreq = 0;
    for (int p = 0; p < count; p++) {
        centerFreqOut[p] = j->centerFreqIn[p];

        if (peakBins[p] != 0 && peakValues[p] > noiseThreshold) {
            float meanBin = peakBins[p];
            const float y0 = sdftData[peakBins[p] - 1];
            const float y1 = sdftData[peakBins[p]];
            const float y2 = sdftData[peakBins[p] + 1];
            const float denom = 2.0f * (y0 - 2 * y1 + y2);
            if (denom != 0.0f) {
                meanBin += (y0 - y2) / denom;
            }

            const float centerFreq = constrainf(meanBin * j->resolutionHz, j->minHz, j->maxHz);
            const float cutoffMult = constrainf(peakValues[p] / noiseThreshold, 1.0f, 10.0f);
            const float gain = pt1FilterGain(j->smoothHz * cutoffMult, j->pt1LooptimeS);

            centerFreqOut[p] += gain * (centerFreq - centerFreqOut[p]);
        }
        if ((int)centerFreqOut[p] > maxCenterFreq) {
            maxCenterFreq = (int)centerFreqOut[p];
        }
    }

    // Publish result (single writer = core1; reader = core0).
    axisResult_t *r = &s_results[j->axis];
    for (int p = 0; p < count; p++) {
        r->result.peakBins[p] = peakBins[p];
        r->result.centerFreqOut[p] = centerFreqOut[p];
    }
    r->result.maxCenterFreq = maxCenterFreq;

    // Release fence so core0 sees the payload before the ready flag.
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&r->ready, 1u, __ATOMIC_RELAXED);
}

static void core1Update(void)
{
    core1_ring_slot_t slot;
    while (core1_ring_pop(&s_ring, &slot)) {
        // slot is 1-based index into s_slots; 0 means "ignore".
        if (slot == 0 || slot > DYN_NOTCH_OFFLOAD_SLOTS) {
            continue;
        }
        jobSlot_t *js = &s_slots[slot - 1];
        runJob(&js->job);

        // Release the slot back to the producer pool.
        __atomic_thread_fence(__ATOMIC_RELEASE);
        __atomic_store_n(&js->in_use, 0u, __ATOMIC_RELAXED);
    }
}

static const multicore_task_t s_task = {
    .update = core1Update,
    .name   = "dyn_notch",
};

// ---------------- API ----------------

bool dynNotchOffloadInit(void)
{
    core1_ring_init(&s_ring);
    for (int i = 0; i < DYN_NOTCH_OFFLOAD_SLOTS; i++) {
        s_slots[i].in_use = 0;
    }
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        s_results[i].ready = 0;
    }

    if (!multicoreScheduleTask(&s_task)) {
        return false;
    }
    s_active = true;
    return true;
}

bool dynNotchOffloadActive(void)
{
    return s_active;
}

bool dynNotchOffloadPost(const dynNotchOffloadJob_t *job)
{
    if (!s_active || job == NULL) {
        return false;
    }
    // Claim a free slot in the pool.
    int slotIdx = -1;
    for (int i = 0; i < DYN_NOTCH_OFFLOAD_SLOTS; i++) {
        uint32_t expected = 0;
        if (__atomic_compare_exchange_n(&s_slots[i].in_use, &expected, 1u,
                                        false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
            slotIdx = i;
            break;
        }
    }
    if (slotIdx < 0) {
        return false; // pool exhausted (core1 falling behind)
    }
    s_slots[slotIdx].job = *job;

    __atomic_thread_fence(__ATOMIC_RELEASE);
    if (!core1_ring_push(&s_ring, (core1_ring_slot_t)(slotIdx + 1))) {
        // Roll back the slot reservation.
        __atomic_store_n(&s_slots[slotIdx].in_use, 0u, __ATOMIC_RELAXED);
        return false;
    }
    return true;
}

bool dynNotchOffloadPoll(int axis, dynNotchOffloadResult_t *out)
{
    if (!s_active || axis < 0 || axis >= XYZ_AXIS_COUNT || out == NULL) {
        return false;
    }
    axisResult_t *r = &s_results[axis];
    if (__atomic_load_n(&r->ready, __ATOMIC_ACQUIRE) == 0u) {
        return false;
    }
    *out = r->result;
    __atomic_store_n(&r->ready, 0u, __ATOMIC_RELEASE);
    return true;
}

#endif // USE_MULTICORE && USE_DYN_NOTCH_FILTER
