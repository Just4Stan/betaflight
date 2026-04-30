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

#ifdef USE_SYSID

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "build/atomic.h"
#include "common/maths.h"
#include "drivers/time.h"

#include "build/assert_core.h"

#ifdef USE_MULTICORE
#include "platform/multicore.h"
#endif

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "config/config.h"
#include "fc/runtime_config.h"
#include "flight/pid.h"

#include "sysid.h"
#include "sysid_welch.h"
#include "sysid_plant_fit.h"

PG_REGISTER_WITH_RESET_FN(sysidConfig_t, sysidConfig, PG_SYSID_CONFIG, 0);
void pgResetFn_sysidConfig(sysidConfig_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
}

// SHARED single (u, y) capture buffer (chirp processes one axis at a time).
// 4096 samples × 500 Hz = 8.2 s window. Memory cost: 32 KB BSS.
#define SYSID_RING_LEN     4096u
#define SYSID_PUSH_STRIDE  16      // 8 kHz PID → 500 Hz capture

typedef struct {
    sysid_result_t result;
    volatile uint8_t result_valid;
} sysid_axis_t;

// One shared buffer (u, y); one current-axis index marking which axis owns
// the buffer right now.
static FAST_DATA_ZERO_INIT float s_u_buf[SYSID_RING_LEN];
static FAST_DATA_ZERO_INIT float s_y_buf[SYSID_RING_LEN];

// PID + dterm-LPF snapshot taken at chirp-end. Captures the controller
// state that was actually in effect during the chirp so deconvolution
// uses those gains, not whatever the user happens to have set when the
// compute eventually runs.
typedef struct {
    float Kp, Ki, Kd;
    float dterm_lpf1_hz;
} sysid_ctrl_snapshot_t;
static FAST_DATA_ZERO_INIT sysid_ctrl_snapshot_t s_ctrl_snap;
static volatile uint32_t s_buf_count = 0;  // also serves as write index — single producer (core0)
static volatile uint8_t  s_capturing_axis = 0xff;
static volatile uint8_t  s_computing_axis = 0xff;
static volatile uint32_t s_push_skip = 0;
static FAST_DATA_ZERO_INIT sysid_axis_t s_axes[SYSID_AXIS_COUNT];

// Welch + FFT scratch. N=512 keeps the scratch (~16 KB) inside the V0.3
// memory budget while still giving us 1 kHz / 512 ≈ 2 Hz bin density.
#define SYSID_NEST           512
#define SYSID_NFREQ          (SYSID_NEST / 2 + 1)
#define SYSID_NOVERLAP_PCT   75
#define SYSID_NOVERLAP       ((SYSID_NEST * SYSID_NOVERLAP_PCT) / 100)
// Bumped from 1 Hz to 3 Hz: at the lowest 1-Hz bins T→1 (closed loop tracks
// DC perfectly), so 1-T → 0 and deconvolution G = T/(C·(1-T)) is numerically
// hostile even with a Dmag2 epsilon guard. Bin spacing at N=512, fs=500
// is ~1 Hz so 3 Hz keeps ~50 usable bins below f_max=100 Hz.
#define SYSID_FIT_FMIN_HZ    3.0f
#define SYSID_FIT_FMAX_HZ    100.0f
#define SYSID_FIT_COH_MIN    0.5f
#define SYSID_LOG_RATE_HZ    500.0f    // = 8000 / SYSID_PUSH_STRIDE

static FAST_DATA_ZERO_INIT float s_win[SYSID_NEST];
static FAST_DATA_ZERO_INIT float s_seg[SYSID_NEST];
static FAST_DATA_ZERO_INIT float s_Ure[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_Uim[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_Yre[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_Yim[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_Suu[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_Syu_re[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_Syu_im[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_Syy[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_H_re[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_H_im[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_coh[SYSID_NFREQ];
static FAST_DATA_ZERO_INIT float s_freq_axis[SYSID_NFREQ];
static welch_fft_ctx_t *s_fft_ctx = NULL;
static volatile uint32_t s_last_compute_us = 0;
static volatile uint32_t s_max_compute_us = 0;

#ifdef USE_MULTICORE
// Inter-core flag: which axis (0..2) needs computing, or 0xff for idle.
static volatile uint8_t s_pending_axis = 0xff;
#endif

// Compute the fit for `axis` from its captured ring. Single-axis at a time.
// Synchronous; runs on whatever core calls it.
static void sysid_compute_axis(int axis)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return;
    sysid_axis_t *a = &s_axes[axis];
    const uint32_t t0 = micros();
    const uint32_t n = s_buf_count;
    if (n < (uint32_t)(SYSID_NEST + 64)) {
        __atomic_store_n(&s_computing_axis, (uint8_t)0xff, __ATOMIC_RELEASE);
        return;
    }

    if (!s_fft_ctx) {
        // Should have been allocated in sysidInit on core0; bail safely.
        __atomic_store_n(&s_computing_axis, (uint8_t)0xff, __ATOMIC_RELEASE);
        return;
    }

    welch_t w;
    if (!welch_init(&w, SYSID_NEST, SYSID_NOVERLAP, s_fft_ctx,
                    s_win, s_seg, s_Ure, s_Uim, s_Yre, s_Yim,
                    s_Suu, s_Syu_re, s_Syu_im, s_Syy)) {
        __atomic_store_n(&s_computing_axis, (uint8_t)0xff, __ATOMIC_RELEASE);
        return;
    }

    welch_process(&w, s_u_buf, s_y_buf, (int)n);
    welch_finalize(&w);
    welch_estimate_h1(&w, 0.0f, s_H_re, s_H_im, s_coh);

    // Deconvolve the controller out of the closed-loop FRF to recover the
    // open-loop plant. The Welch result is T(jω) = G·C / (1 + G·C). Solve
    //   G(jω) = T / (C · (1 - T))
    // using the current axis's PID gains and D-term LPF as C(jω). After
    // this, s_H_re / s_H_im hold the *plant* spectrum and the plant fit
    // is the rigid-body airframe model — not the closed-loop response.
    {
        // Acquire-pair with the release in sysidNotifyChirpEnd so we read
        // the gains that were active during the chirp.
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        const float Kp = s_ctrl_snap.Kp;
        const float Ki = s_ctrl_snap.Ki;
        const float Kd = s_ctrl_snap.Kd;
        const float fc_d = s_ctrl_snap.dterm_lpf1_hz;
        const float twopi_fc_d = 6.28318530f * fc_d;
        // Skip the DC bin where T → 1 makes deconvolution singular.
        if (s_coh[0] < 1.0f) { s_H_re[0] = 0.0f; s_H_im[0] = 0.0f; }
        s_coh[0] = 0.0f;
        for (int k = 1; k < SYSID_NFREQ; k++) {
            const float w  = 6.28318530f * s_freq_axis[k];
            // D-term PT1 filter H_D(jω) = 1 / (1 + jω/(2π fc_d))
            //                           = (1 - jω/twopi_fc_d) / (1 + (ω/twopi_fc_d)²)
            const float r = (fc_d > 0.0f) ? (w / twopi_fc_d) : 0.0f;
            const float den_d = 1.0f + r * r;
            const float HDre =  1.0f / den_d;
            const float HDim = -r    / den_d;
            // Controller C(jω) = Kp + Ki/(jω) + Kd · jω · H_D(jω)
            //   Ki/(jω) = -j Ki/ω
            //   Kd·jω·(HDre + jHDim) = Kd · (-ω HDim + j ω HDre)
            const float C_re = Kp - Kd * w * HDim;
            const float C_im = Kd * w * HDre - Ki / w;
            // 1 - T
            const float oneMTre = 1.0f - s_H_re[k];
            const float oneMTim =      - s_H_im[k];
            // Denominator = C · (1 - T)
            const float Dre = C_re * oneMTre - C_im * oneMTim;
            const float Dim = C_re * oneMTim + C_im * oneMTre;
            const float Dmag2 = Dre * Dre + Dim * Dim;
            if (Dmag2 < 1e-18f) {
                s_H_re[k] = 0.0f; s_H_im[k] = 0.0f; s_coh[k] = 0.0f;
                continue;
            }
            // G = T / (C·(1-T))  →  multiply numerator by conjugate of denom
            const float Tre = s_H_re[k];
            const float Tim = s_H_im[k];
            s_H_re[k] = (Tre * Dre + Tim * Dim) / Dmag2;
            s_H_im[k] = (Tim * Dre - Tre * Dim) / Dmag2;
        }
    }

    float K0, wn0, zeta0;
    plant_fit_t pf = {0};
    if (plant_fit_initial_guess(s_freq_axis, s_H_re, s_H_im, s_coh,
                                SYSID_NFREQ, 5.0f, &K0, &wn0, &zeta0)) {
        plant_fit_2nd_order(s_freq_axis, s_H_re, s_H_im, s_coh,
                            SYSID_NFREQ, SYSID_FIT_FMIN_HZ, SYSID_FIT_FMAX_HZ,
                            SYSID_FIT_COH_MIN, K0, wn0, zeta0, &pf);
    }

    a->result.K = pf.K;
    a->result.wn_rad_s = pf.wn;
    a->result.zeta = pf.zeta;
    a->result.rmse_db = pf.rmse_db;
    a->result.Nbands = (uint16_t)pf.Nbands;
    a->result.iters = (uint8_t)pf.iters;
    a->result.flags = SYSID_FLAG_VALID | (pf.converged ? SYSID_FLAG_CONVERGED : 0);
    a->result.timestamp_ms = millis();
    const uint32_t dt = micros() - t0;
    s_last_compute_us = dt;
    if (dt > s_max_compute_us) s_max_compute_us = dt;
    // Release the result struct to readers, then clear the in-flight flag
    // (also release-ordered so a polled "is computing?" never sees 0xff
    // before the result_valid=1 it advertises).
    __atomic_store_n(&a->result_valid, (uint8_t)1, __ATOMIC_RELEASE);
    __atomic_store_n(&s_computing_axis, (uint8_t)0xff, __ATOMIC_RELEASE);
}

#ifdef USE_MULTICORE
// core1-side worker: poll the pending axis flag, run compute, clear.
static void sysid_core1_update(void)
{
    ASSERT_CORE1();
    uint8_t ax = __atomic_load_n(&s_pending_axis, __ATOMIC_ACQUIRE);
    if (ax >= SYSID_AXIS_COUNT) return;
    sysid_compute_axis((int)ax);
    __atomic_store_n(&s_pending_axis, 0xff, __ATOMIC_RELEASE);
}

static const multicore_task_t s_sysid_task = {
    .update = sysid_core1_update,
    .name = "sysid",
};
#endif

void sysidComputeNow(int axis)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return;
    s_computing_axis = (uint8_t)axis;
    sysid_compute_axis(axis);
}

uint32_t sysidCaptureCount(void)
{
    return s_buf_count;
}

uint32_t sysidLastComputeUs(void) { return s_last_compute_us; }
uint32_t sysidMaxComputeUs(void)  { return s_max_compute_us; }

void sysidCommitToConfig(void)
{
    sysidConfig_t *c = sysidConfigMutable();
    for (int ax = 0; ax < SYSID_AXIS_COUNT; ax++) {
        if (s_axes[ax].result_valid) {
            c->persisted[ax] = s_axes[ax].result;
        }
    }
}

void sysidLoadFromConfig(void)
{
    const sysidConfig_t *c = sysidConfig();
    for (int ax = 0; ax < SYSID_AXIS_COUNT; ax++) {
        if (c->persisted[ax].flags & SYSID_FLAG_VALID) {
            s_axes[ax].result = c->persisted[ax];
            // Stale: the persisted timestamp is millis() from a previous
            // boot. Zero it so the CLI age column shows "0" until the
            // user runs a fresh chirp; otherwise (now - very_old_ts) is
            // meaningless and may even underflow uint32 on cold boot.
            s_axes[ax].result.timestamp_ms = 0;
            s_axes[ax].result_valid = 1;
        }
    }
}

// Map a 2nd-order plant fit  G(s) = K_g · ωn² / (s² + 2ζ ωn s + ωn²)
// (recovered from closed-loop FRF via deconvolution in sysid_compute_axis)
// to BF P/I/D gain integers via classical loop shaping.
//
// Crossover target wc is HARDCODED to 20 Hz (= 125.66 rad/s). A quad rate
// plant after deconvolution has its dominant pole near 1 Hz and a second
// pole near 10 Hz; we want crossover comfortably above both, where the
// PID's D-zero contributes phase lift. 20 Hz is the sweet spot for 3-5"
// quads — high enough to track stick inputs, low enough to stay below
// motor / frame resonances.
//
// Phase margin target φm = 50° → ∠C(jωc) needed = -130° - ∠G(jωc).
// PID controller C(s) = Kp + Ki/s + Kd s. With:
//   I-zero at ωi = wc/5    (≤ 12° of lag at crossover)
//   D-zero at ωd = wc/3    (~+50° of phase lift at crossover)
// |C(jωc)| = Kp · sqrt(1 + (ωc/ωd)²) · sqrt(1 + (ωi/ωc)²) = Kp · 3.225
//
// |G(jωc)| evaluated from the 2nd-order LP fit at the FIXED wc:
//   |G(jωc)|² = K_g² · ωn⁴ / ((ωn² - ωc²)² + (2ζ·ωn·ωc)²)
//
// Unity loop gain: Kp · 3.225 · |G(jωc)| = 1 →
//   Kp_c = 1 / (3.225 · |G(jωc)|)
//   Ki_c = Kp_c · ωc / 5
//   Kd_c = Kp_c · 3 / ωc
//
// Convert to BF integers via PTERM_SCALE = 0.032029, ITERM_SCALE = 0.244381,
// DTERM_SCALE = 0.000529 from pid.h.
bool sysidSuggestPid(const sysid_result_t *fit, int *out_p, int *out_i, int *out_d)
{
    if (!fit || !out_p || !out_i || !out_d) return false;
    if (!(fit->flags & SYSID_FLAG_VALID)) return false;
    if (fit->Nbands < 6) return false;
    if (fit->K < 0.05f || fit->K > 50.0f) return false;
    if (fit->wn_rad_s < 1.0f || fit->wn_rad_s > 600.0f) return false;
    if (fit->zeta < 0.05f || fit->zeta > 5.0f) return false;

    static const float WC_TARGET_RAD_S = 125.66f;  // 20 Hz, sensible for 3-5" quads

    const float wc  = WC_TARGET_RAD_S;
    const float wn  = fit->wn_rad_s;
    const float K_g = fit->K;
    const float zeta = fit->zeta;

    // |G(jωc)| from the 2nd-order LP fit, evaluated at the HARDCODED wc.
    const float real = wn * wn - wc * wc;
    const float imag = 2.0f * zeta * wn * wc;
    const float den2 = real * real + imag * imag;
    if (den2 < 1e-30f) return false;
    const float Gmag = K_g * wn * wn / sqrtf(den2);
    if (Gmag < 1e-9f) return false;

    const float Kp_c = 1.0f / (3.225f * Gmag);
    const float Ki_c = Kp_c * wc / 5.0f;
    const float Kd_c = Kp_c * 3.0f / wc;

    const float Pf = Kp_c / 0.032029f;
    const float If = Ki_c / 0.244381f;
    const float Df = Kd_c / 0.000529f;

    int p = (int)(Pf + 0.5f);
    int i = (int)(If + 0.5f);
    int d = (int)(Df + 0.5f);
    if (p < 5)   { p = 5; }   if (p > 200) { p = 200; }
    if (i < 10)  { i = 10; }  if (i > 250) { i = 250; }
    if (d < 0)   { d = 0; }   if (d > 150) { d = 150; }
    *out_p = p; *out_i = i; *out_d = d;
    return true;
}

void sysidInit(void)
{
    for (int ax = 0; ax < SYSID_AXIS_COUNT; ax++) {
        s_axes[ax].result_valid = 0;
        memset(&s_axes[ax].result, 0, sizeof(s_axes[ax].result));
    }
    s_capturing_axis = 0xff;
    s_computing_axis = 0xff;
    s_buf_count = 0;
    s_push_skip = 0;
    // Pre-allocate the FFT context on core0 so core1 never touches the
    // allocator. The pool inside sysid_fft.c is static; this just
    // initialises twiddles + bitrev tables for SYSID_NEST.
    s_fft_ctx = welch_fft_create(SYSID_NEST);
    for (int k = 0; k < SYSID_NFREQ; k++) {
        s_freq_axis[k] = (float)k * SYSID_LOG_RATE_HZ / (float)SYSID_NEST;
    }
    sysidLoadFromConfig();
#ifdef USE_MULTICORE
    multicoreScheduleTask(&s_sysid_task);
#endif
}

FAST_CODE void sysidPushSample(int axis, float setpoint, float gyroUnfilt)
{
    ASSERT_CORE0();
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return;
    if (s_capturing_axis != (uint8_t)axis) return;
    // Stride-decimate from PID rate (8 kHz) down to ~1 kHz capture rate.
    if (++s_push_skip < SYSID_PUSH_STRIDE) return;
    s_push_skip = 0;
    if (s_buf_count >= SYSID_RING_LEN) return;
    s_u_buf[s_buf_count] = setpoint;
    s_y_buf[s_buf_count] = gyroUnfilt;
    s_buf_count++;
}

void sysidNotifyChirpStart(int axis)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return;
    // If the previous chirp's compute hasn't finished yet, refuse the new
    // capture. The shared (u, y) buffer is single-producer / single-consumer
    // and can't be safely reset while core1 is still reading it. The
    // dropped-axis case is logged via result_valid staying false for that
    // axis until the next successful capture.
    if (__atomic_load_n(&s_computing_axis, __ATOMIC_ACQUIRE) != 0xff) {
        return;
    }
    // Snapshot the controller AT CHIRP START — these are the gains that
    // were active during the chirp and that the deconvolution must use.
    // Taking the snapshot at chirp END would let any user-driven gain
    // change during the chirp leak into the deconvolution.
    s_ctrl_snap.Kp           = pidRuntime.pidCoefficient[axis].Kp;
    s_ctrl_snap.Ki           = pidRuntime.pidCoefficient[axis].Ki;
    s_ctrl_snap.Kd           = pidRuntime.pidCoefficient[axis].Kd;
    s_ctrl_snap.dterm_lpf1_hz = (float)currentPidProfile->dterm_lpf1_static_hz;
    s_buf_count = 0;
    s_push_skip = 0;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&s_capturing_axis, (uint8_t)axis, __ATOMIC_RELEASE);
}

void sysidNotifyChirpEnd(int axis)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return;
    if (s_capturing_axis == (uint8_t)axis) {
        __atomic_store_n(&s_capturing_axis, (uint8_t)0xff, __ATOMIC_RELEASE);
    }
    __atomic_store_n(&s_computing_axis, (uint8_t)axis, __ATOMIC_RELEASE);
#ifdef USE_MULTICORE
    __atomic_store_n(&s_pending_axis, (uint8_t)axis, __ATOMIC_RELEASE);
#else
    sysid_compute_axis(axis);
#endif
}

bool sysidGetResult(int axis, sysid_result_t *out)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT || !out) return false;
    // Acquire-pair with the release in sysid_compute_axis so we never see
    // a result_valid=1 ahead of the struct payload it advertises.
    if (!__atomic_load_n(&s_axes[axis].result_valid, __ATOMIC_ACQUIRE)) return false;
    *out = s_axes[axis].result;
    return true;
}

bool sysidIsCapturing(int axis)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return false;
    return s_capturing_axis == (uint8_t)axis;
}

bool sysidIsComputing(int axis)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return false;
    return s_computing_axis == (uint8_t)axis;
}

#endif // USE_SYSID
