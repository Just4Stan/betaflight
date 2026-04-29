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

#include "common/sdft.h"
#include "flight/dyn_notch_filter.h"

// Cross-platform interface for offloading the dyn_notch peak-detection and
// center-frequency calculation onto a secondary core. The PICO RP2350B target
// supplies an implementation that uses core1; on every other target the
// functions are inline no-ops so the inline state-machine path stays in use.

typedef struct dynNotchOffloadJob_s {
    int   axis;
    int   peakCount;
    int   startBin;
    int   endBin;
    float resolutionHz;
    float minHz;
    float maxHz;
    float pt1LooptimeS;
    float smoothHz;
    float noiseThresholdInitial; // sum(sdftData[start..end]) from STEP_WINDOW
    float sdftData[SDFT_BIN_COUNT];
    float centerFreqIn[DYN_NOTCH_COUNT_MAX];
} dynNotchOffloadJob_t;

typedef struct dynNotchOffloadResult_s {
    int   peakBins[DYN_NOTCH_COUNT_MAX];   // 0 = void peak
    float centerFreqOut[DYN_NOTCH_COUNT_MAX];
    int   maxCenterFreq;                   // for OSD
} dynNotchOffloadResult_t;

#if defined(USE_MULTICORE) && defined(USE_DYN_NOTCH_FILTER)

// Init the offload subsystem. Must be called before multicoreStart().
// Returns false if registration with the core1 scheduled-task table fails;
// in that case callers should fall back to the inline state-machine path.
bool dynNotchOffloadInit(void);

// Returns true if the offload was registered and is active (i.e. previously
// returned true from dynNotchOffloadInit()).
bool dynNotchOffloadActive(void);

// Producer (core0). Submit a job for processing on core1.
// Returns false if the ring is full; caller should fall back to inline.
bool dynNotchOffloadPost(const dynNotchOffloadJob_t *job);

// Producer (core0). Non-blocking poll for the most recent result for `axis`.
// Returns true and fills `out` if a result is ready; returns false otherwise.
// Once consumed the result is cleared (single-shot semantics per axis).
bool dynNotchOffloadPoll(int axis, dynNotchOffloadResult_t *out);

#else // no multicore / no dyn_notch

static inline bool dynNotchOffloadInit(void) { return false; }
static inline bool dynNotchOffloadActive(void) { return false; }
static inline bool dynNotchOffloadPost(const dynNotchOffloadJob_t *job) { (void)job; return false; }
static inline bool dynNotchOffloadPoll(int axis, dynNotchOffloadResult_t *out) { (void)axis; (void)out; return false; }

#endif
