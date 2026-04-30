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

// On-board chirp-based system identification (PR-D).
//
// Pipeline:
//   1. PID task (core0) calls sysidPushSample(axis, setpoint, gyroUnfilt)
//      every PID tick. Samples are buffered into a per-axis ring while a
//      chirp is active on that axis.
//   2. When the chirp generator transitions out of `active` for an axis,
//      sysidNotifyChirpEnd() posts a job to core1.
//   3. core1 runs Welch H1 + 2nd-order rigid-body plant fit. Result is
//      written to the per-axis result slot via atomic store.
//   4. CLI / OSD / MSP read the latest result via sysidGetResult().
//
// The result is also persisted via a PG_REGISTER'd config group so it
// survives a power cycle.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pg/pg.h"

#define SYSID_AXIS_COUNT  3   // roll, pitch, yaw

typedef struct {
    float K;            // DC gain
    float wn_rad_s;     // natural frequency (rad/s)
    float zeta;         // damping ratio
    float rmse_db;      // log-magnitude RMSE
    uint16_t Nbands;    // bins kept after coherence gate
    uint8_t  iters;     // GN iterations
    uint8_t  flags;     // bit 0: converged; bit 1: result valid
    uint32_t timestamp_ms;  // millis() at capture
} sysid_result_t;

#define SYSID_FLAG_CONVERGED  (1u << 0)
#define SYSID_FLAG_VALID      (1u << 1)

// Persistent storage: last result per axis. Saved to EEPROM via the
// existing config flash path so a fresh boot retains the last fit even
// after a power cycle. Wiped to zero on `defaults`.
typedef struct sysidConfig_s {
    sysid_result_t persisted[SYSID_AXIS_COUNT];
} sysidConfig_t;

PG_DECLARE(sysidConfig_t, sysidConfig);

// Copy current in-RAM results into persisted storage; call before save.
void sysidCommitToConfig(void);
// Restore the last persisted results into RAM at boot.
void sysidLoadFromConfig(void);

void sysidInit(void);

// Producer (core0, gyro/PID task). Cheap; just appends to a ring.
void sysidPushSample(int axis, float setpoint, float gyroUnfilt);

// Producer (core0). Resets the per-axis ring; call when the chirp
// generator starts on a new axis.
void sysidNotifyChirpStart(int axis);

// Producer (core0). Triggers core1 to consume the ring + run the fit.
void sysidNotifyChirpEnd(int axis);

// Consumer (any core, read-only). Returns true if a fresh result is
// available; copies it into *out if so. Result is sticky -- calling
// repeatedly returns the same data until a new chirp completes.
bool sysidGetResult(int axis, sysid_result_t *out);

// Returns true while a chirp capture is in progress on `axis`.
bool sysidIsCapturing(int axis);

// Returns true while core1 is mid-computation for `axis`.
bool sysidIsComputing(int axis);

// Number of (u, y) samples in the shared capture buffer right now.
uint32_t sysidCaptureCount(void);

// Last + max wall-clock compute time for the Welch + deconvolve + plant
// fit pipeline, in microseconds. Reported via CLI/MSP for budget audit.
uint32_t sysidLastComputeUs(void);
uint32_t sysidMaxComputeUs(void);

// Synchronous compute trigger for testing -- runs Welch + plant fit on
// whatever's currently in the shared buffer. Steals time from the calling
// core; do not call from a hard-realtime path.
void sysidComputeNow(int axis);

// Map a 2nd-order rigid-body fit to suggested PID gains. The mapping is a
// simple loop-shaping rule (target crossover at 0.5*wn clamped to 60-200
// rad/s, target phase margin ~50 deg). Returns false if the fit is
// unfit-for-suggestion (failed convergence, low Nbands, or unrealistic
// params).
bool sysidSuggestPid(const sysid_result_t *fit,
                     int *out_p, int *out_i, int *out_d);
