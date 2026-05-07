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
#include "sensors/gyro.h"

#include "sysid.h"
#include "sysid_welch.h"
#include "sysid_plant_fit.h"

PG_REGISTER_WITH_RESET_FN(sysidConfig_t, sysidConfig, PG_SYSID_CONFIG, 0);
void pgResetFn_sysidConfig(sysidConfig_t *cfg)
{
    // `defaults` resets the knobs but preserves persisted fits — those are
    // diagnostic data, not configuration.
    cfg->target_wc_dHz     = 200;   // 20 Hz crossover (3-5" sweet spot)
    cfg->fit_fmin_dHz      = 30;    // 3 Hz
    cfg->fit_fmax_dHz      = 1000;  // 100 Hz
    cfg->fit_min_coherence = 50;    // 0.50
}

// Dirty flag — sysidCommitToConfig only writes when there's actually new
// data, avoiding unnecessary flash wear on every saveConfig().
static volatile bool s_persist_dirty = false;

// STREAMING capture: O(N) RAM regardless of chirp length. Producer
// (core0) maintains a small N-deep circular ring of decimated samples;
// every `step = N - noverlap` samples it copies the latest N to a
// segment buffer and signals core1 via s_seg_pending. core1 windows,
// FFTs, and accumulates Suu/Syu/Syy without keeping the time-domain
// data. At chirp end core0 sets s_pending_axis; core1 drains any
// trailing segment, then finalises + plant-fits + publishes.
//
// Memory cost: 2 × N samples (ring) + 2 × N samples (segment buffer) =
// 4 KB at N=512. ~30× less than the old batch capture buffer, and
// covers any chirp length.
#define SYSID_PUSH_STRIDE  16      // 8 kHz PID → 500 Hz capture

// Welch + FFT scratch. N=512 keeps the scratch (~16 KB) inside the V0.3
// memory budget while still giving us 1 kHz / 512 ≈ 2 Hz bin density.
#define SYSID_NEST           512
#define SYSID_NFREQ          (SYSID_NEST / 2 + 1)
#define SYSID_NOVERLAP_PCT   75
#define SYSID_NOVERLAP       ((SYSID_NEST * SYSID_NOVERLAP_PCT) / 100)

// PT1 anti-alias on producer side: capture rate 500 Hz → Nyquist 250 Hz.
// Cut at 200 Hz so chirp content above 250 Hz (the BF chirp generator's
// f1 default is 400 Hz) folds out before stride-decimation.
#define SYSID_AA_CUTOFF_HZ   200.0f
static FAST_DATA_ZERO_INIT pt1Filter_t s_aa_u, s_aa_y;

typedef struct {
    sysid_result_t result;
    volatile uint8_t result_valid;
} sysid_axis_t;

// Streaming ring: just N samples deep, indexed circularly. Producer
// (core0) writes; consumer (core1) reads via the segment buffer below.
#define SYSID_RING_LEN  SYSID_NEST    // = N, sliding window
static FAST_DATA_ZERO_INIT float s_ring_u[SYSID_RING_LEN];
static FAST_DATA_ZERO_INIT float s_ring_y[SYSID_RING_LEN];
static volatile uint32_t s_ring_pos = 0;            // write index, mod N
static volatile uint32_t s_ring_filled = 0;         // saturates at N
static volatile uint32_t s_samples_since_seg = 0;   // counter, resets at every step

// Segment hand-off buffer (one slot, drop-on-not-consumed).
static FAST_DATA_ZERO_INIT float s_seg_u_out[SYSID_NEST];
static FAST_DATA_ZERO_INIT float s_seg_y_out[SYSID_NEST];
static volatile uint8_t s_seg_pending = 0;          // 1 = filled, awaiting core1
static volatile uint32_t s_seg_axis = 0;            // axis the pending segment is for
static volatile uint32_t s_seg_dropped = 0;         // diagnostic count
static volatile uint8_t  s_chirp_ending = 0;        // set in NotifyEnd
static volatile uint8_t  s_chirp_axis = 0;          // axis to finalise on next compute

// Controller + gyro-filter-chain snapshot taken at chirp START. The
// deconvolution model is:
//     T_meas = F·G·C / (1 + G·C·F)
// where F is the gyro filter chain (LPF1 + LPF2) and C is the rate
// controller (PID + dterm LPF). Recovered plant: G = T / (C·F·(1 - T)).
// Snapshot at start (not end) so user-side gain tweaks during the chirp
// don't leak into the deconvolution. dyn_notch contributions are not
// modelled — its centers track motor RPM (100–300 Hz), well above the
// 1–100 Hz fit band, so its in-band magnitude error is < 1 dB.
typedef struct {
    float Kp, Ki, Kd;
    float dterm_lpf1_hz;
    float gyro_lpf1_hz;
    float gyro_lpf2_hz;
} sysid_ctrl_snapshot_t;
static FAST_DATA_ZERO_INIT sysid_ctrl_snapshot_t s_ctrl_snap;
static volatile uint32_t s_total_samples_captured = 0;  // diagnostic counter — running total
static volatile uint8_t  s_capturing_axis = 0xff;
static volatile uint8_t  s_computing_axis = 0xff;
static volatile uint32_t s_push_skip = 0;
static FAST_DATA_ZERO_INIT sysid_axis_t s_axes[SYSID_AXIS_COUNT];

// (SYSID_NEST/SYSID_NFREQ/SYSID_NOVERLAP defined above for ring sizing.)
// Defaults for the band fit live in sysidConfig (fmin/fmax/coh_min knobs).
// Capture rate is derived at sysidInit() from gyro.targetLooptime — a
// 4 kHz PID build (pid_process_denom=2) gets 250 Hz capture, an 8 kHz
// build gets 500 Hz, etc. The frequency axis (s_freq_axis) and the
// deconvolution C(jω) evaluation both use this runtime value, so they
// stay consistent across pid_process_denom changes.
static FAST_DATA_ZERO_INIT float s_log_rate_hz;

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
// Pending axis dispatch — set by sysidNotifyChirpEnd, cleared by core1
// when finalize is queued. Distinct from s_seg_pending which signals
// per-segment work.
static volatile uint8_t s_pending_axis = 0xff;
#endif

// Persistent Welch accumulator state — lives across an entire chirp.
// Initialised at sysidNotifyChirpStart, fed segment-by-segment during
// the chirp, finalised at chirp end. Stored as a file-static (single
// in-flight chirp at a time).
static welch_t s_welch;
static FAST_DATA_ZERO_INIT bool s_welch_inited = false;
static FAST_DATA_ZERO_INIT uint8_t s_welch_axis = 0xff;

// Process one Welch segment into the persistent accumulator. Called from
// core1 (or the sync fallback). Idempotent + thread-safe relative to
// producer (single segment slot, drop-on-not-consumed).
static void sysid_process_segment_core1(void)
{
    if (__atomic_load_n(&s_seg_pending, __ATOMIC_ACQUIRE) == 0u) return;
    if (!s_welch_inited) {
        // Producer pushed a segment before init? Shouldn't happen — drop.
        __atomic_store_n(&s_seg_pending, (uint8_t)0, __ATOMIC_RELEASE);
        return;
    }
    // Producer has signalled "filled"; consumer copies + processes; clears.
    welch_process_segment(&s_welch, s_seg_u_out, s_seg_y_out);
    __atomic_store_n(&s_seg_pending, (uint8_t)0, __ATOMIC_RELEASE);
}

// Finalise + plant-fit + publish for `axis`. Called at chirp end after
// any trailing segment has been processed.
static void sysid_finalize_axis(int axis)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return;
    sysid_axis_t *a = &s_axes[axis];
    __atomic_store_n(&a->result_valid, (uint8_t)0, __ATOMIC_RELEASE);
    const uint32_t t0 = micros();

    if (!s_welch_inited || s_welch.Navg < 5) {
        // Aborted chirp / too few segments for a meaningful coherence
        // estimate. Bail.
        s_welch_inited = false;
        return;
    }

    welch_finalize(&s_welch);
    welch_estimate_h1(&s_welch, 0.0f, s_H_re, s_H_im, s_coh);

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
        const float fc_g1 = s_ctrl_snap.gyro_lpf1_hz;
        const float fc_g2 = s_ctrl_snap.gyro_lpf2_hz;
        // Skip the DC bin where T → 1 makes deconvolution singular.
        if (s_coh[0] < 1.0f) { s_H_re[0] = 0.0f; s_H_im[0] = 0.0f; }
        s_coh[0] = 0.0f;
        // Helper: PT1 filter response 1/(1 + jω/(2π fc)) → returns (re, im).
        #define PT1_RESPONSE(omega_, fc_, out_re, out_im) do {        \
            const float _r = ((fc_) > 0.0f) ? ((omega_) / (6.28318530f * (fc_))) : 0.0f; \
            const float _d = 1.0f + _r * _r;                           \
            (out_re) =  1.0f / _d;                                     \
            (out_im) = -_r   / _d;                                     \
        } while (0)
        for (int k = 1; k < SYSID_NFREQ; k++) {
            const float omega = 6.28318530f * s_freq_axis[k];
            // D-term PT1 H_D(jω)
            float HDre, HDim;
            PT1_RESPONSE(omega, fc_d, HDre, HDim);
            // Controller C(jω) = Kp + Ki/(jω) + Kd · jω · H_D(jω)
            const float C_re = Kp - Kd * omega * HDim;
            const float C_im = Kd * omega * HDre - Ki / omega;
            // Gyro filter chain F(jω) = H_LPF1(jω) · H_LPF2(jω) — two PT1s
            // in series. (gyro_hardware_lpf is sub-Nyquist on the IMU's
            // own AAF and rolls off above our band; ignored.)
            float F1re, F1im, F2re, F2im;
            PT1_RESPONSE(omega, fc_g1, F1re, F1im);
            PT1_RESPONSE(omega, fc_g2, F2re, F2im);
            const float Fre = F1re * F2re - F1im * F2im;
            const float Fim = F1re * F2im + F1im * F2re;
            // 1 - T
            const float oneMTre = 1.0f - s_H_re[k];
            const float oneMTim =      - s_H_im[k];
            // Denominator = C · F · (1 - T)
            const float CFre = C_re * Fre - C_im * Fim;
            const float CFim = C_re * Fim + C_im * Fre;
            const float Dre = CFre * oneMTre - CFim * oneMTim;
            const float Dim = CFre * oneMTim + CFim * oneMTre;
            const float Dmag2 = Dre * Dre + Dim * Dim;
            if (Dmag2 < 1e-18f) {
                s_H_re[k] = 0.0f; s_H_im[k] = 0.0f; s_coh[k] = 0.0f;
                continue;
            }
            // G = T / (C·F·(1-T))  →  numerator × conj(denom) / |denom|²
            const float Tre = s_H_re[k];
            const float Tim = s_H_im[k];
            s_H_re[k] = (Tre * Dre + Tim * Dim) / Dmag2;
            s_H_im[k] = (Tim * Dre - Tre * Dim) / Dmag2;
        }
        #undef PT1_RESPONSE
    }

    const sysidConfig_t *cfg = sysidConfig();
    const float fit_fmin = (float)cfg->fit_fmin_dHz * 0.1f;
    const float fit_fmax = (float)cfg->fit_fmax_dHz * 0.1f;
    const float coh_min  = (float)cfg->fit_min_coherence * 0.01f;
    float K0, wn0, zeta0;
    plant_fit_t pf = {0};
    if (plant_fit_initial_guess(s_freq_axis, s_H_re, s_H_im, s_coh,
                                SYSID_NFREQ, 5.0f, &K0, &wn0, &zeta0)) {
        plant_fit_2nd_order(s_freq_axis, s_H_re, s_H_im, s_coh,
                            SYSID_NFREQ, fit_fmin, fit_fmax,
                            coh_min, K0, wn0, zeta0, &pf);
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
    s_persist_dirty = true;
}

#ifdef USE_MULTICORE
// core1-side worker. Two distinct units of work:
//   1. Drain a pending segment (per ~256 ms during a chirp).
//   2. Finalise + plant-fit at chirp end (one-shot per chirp).
// Always drain segments first; only finalise once segments are empty.
static void sysid_core1_update(void)
{
    ASSERT_CORE1();
    sysid_process_segment_core1();
    if (__atomic_load_n(&s_seg_pending, __ATOMIC_ACQUIRE) != 0u) return;
    uint8_t ax = __atomic_exchange_n(&s_pending_axis, (uint8_t)0xff, __ATOMIC_ACQUIRE);
    if (ax >= SYSID_AXIS_COUNT) return;
    sysid_finalize_axis((int)ax);
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
    // Drain any pending segment, then finalise. In streaming mode all
    // the segment-by-segment work must already have been done as the
    // chirp progressed; this entry is mainly for testing.
    sysid_process_segment_core1();
    sysid_finalize_axis(axis);
}

uint32_t sysidCaptureCount(void)
{
    return s_total_samples_captured;
}

uint32_t sysidLastComputeUs(void) { return s_last_compute_us; }
uint32_t sysidMaxComputeUs(void)  { return s_max_compute_us; }

void sysidWipeHistory(void)
{
    for (int ax = 0; ax < SYSID_AXIS_COUNT; ax++) {
        __atomic_store_n(&s_axes[ax].result_valid, (uint8_t)0, __ATOMIC_RELEASE);
        memset(&s_axes[ax].result, 0, sizeof(s_axes[ax].result));
    }
    sysidConfig_t *c = sysidConfigMutable();
    memset(c->persisted, 0, sizeof(c->persisted));
    s_persist_dirty = true;
}

bool sysidApplySuggestion(int axis)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return false;
    sysid_result_t r;
    if (!sysidGetResult(axis, &r)) return false;
    int p = 0, i = 0, d = 0;
    if (!sysidSuggestPid(&r, &p, &i, &d)) return false;
    currentPidProfile->pid[axis].P = (uint8_t)p;
    currentPidProfile->pid[axis].I = (uint8_t)i;
    currentPidProfile->pid[axis].D = (uint8_t)d;
    return true;
}

void sysidCommitToConfig(void)
{
    // Skip when nothing has changed since the last save: avoids burning a
    // flash sector on every writeEEPROM() call when the user is just
    // saving unrelated config tweaks.
    if (!s_persist_dirty) return;
    // Use sysidGetResult so the read is acquire-ordered against core1's
    // release-store; otherwise a save issued mid-compute can persist a
    // half-written struct.
    sysidConfig_t *c = sysidConfigMutable();
    for (int ax = 0; ax < SYSID_AXIS_COUNT; ax++) {
        sysid_result_t r;
        if (sysidGetResult(ax, &r)) {
            c->persisted[ax] = r;
        }
    }
    s_persist_dirty = false;
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

    // Target crossover: user-tunable via sysidConfig.target_wc_dHz (×10),
    // default 20 Hz (3–5" sweet spot). Pilots can dial it down for
    // cinelifters or up for tinywhoops via CLI `set sysid_target_wc_dhz`.
    const float wn  = fit->wn_rad_s;
    const float wc_hz = (float)sysidConfig()->target_wc_dHz * 0.1f;
    float wc = 6.28318530f * wc_hz;
    if (wc < 30.0f)  wc = 30.0f;     // sanity floor 5 Hz
    if (wc > 600.0f) wc = 600.0f;    // sanity cap 95 Hz
    (void)wn;
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
    s_total_samples_captured = 0;
    s_push_skip = 0;
    s_ring_pos = 0;
    s_ring_filled = 0;
    s_samples_since_seg = 0;
    s_seg_pending = 0;
    s_welch_inited = false;
    // Pre-allocate the FFT context on core0 so core1 never touches the
    // allocator. The pool inside sysid_fft.c is static; this just
    // initialises twiddles + bitrev tables for SYSID_NEST.
    s_fft_ctx = welch_fft_create(SYSID_NEST);
    // Derive capture rate from the actual PID looptime so 4 kHz / 8 kHz
    // builds both produce the right frequency axis.
    const float pid_rate_hz = (gyro.targetLooptime > 0)
        ? (1.0e6f / (float)gyro.targetLooptime) : 8000.0f;
    s_log_rate_hz = pid_rate_hz / (float)SYSID_PUSH_STRIDE;
    // PT1 AA filter coefficients at PID rate.
    const float dT_pid = 1.0f / pid_rate_hz;
    const float aa_k = pt1FilterGain(SYSID_AA_CUTOFF_HZ, dT_pid);
    pt1FilterInit(&s_aa_u, aa_k);
    pt1FilterInit(&s_aa_y, aa_k);
    for (int k = 0; k < SYSID_NFREQ; k++) {
        s_freq_axis[k] = (float)k * s_log_rate_hz / (float)SYSID_NEST;
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
    // PT1 anti-alias at PID rate. Even when we're skipping samples for
    // stride decimation, we still feed both filters so the state stays
    // consistent and the filter's group delay equilibrates.
    const float u_aa = pt1FilterApply(&s_aa_u, setpoint);
    const float y_aa = pt1FilterApply(&s_aa_y, gyroUnfilt);
    if (++s_push_skip < SYSID_PUSH_STRIDE) return;
    s_push_skip = 0;

    // Append to circular ring.
    const uint32_t pos = s_ring_pos;
    s_ring_u[pos] = u_aa;
    s_ring_y[pos] = y_aa;
    s_ring_pos = (pos + 1) % SYSID_RING_LEN;
    if (s_ring_filled < SYSID_RING_LEN) s_ring_filled++;
    s_total_samples_captured++;

    // Hand off a fresh segment every `step` samples once the ring is full.
    if (++s_samples_since_seg < (uint32_t)(SYSID_NEST - SYSID_NOVERLAP)) return;
    if (s_ring_filled < SYSID_RING_LEN) return;
    s_samples_since_seg = 0;
    if (__atomic_load_n(&s_seg_pending, __ATOMIC_ACQUIRE) != 0u) {
        // core1 hasn't drained the previous segment — drop this one.
        // Producer is faster than consumer only if FFT is slow; with our
        // ~5 ms compute and ~256 ms producer interval this never fires
        // in practice. The counter exposes any pathological case.
        s_seg_dropped++;
        return;
    }
    // Linear copy of the latest N samples in time order: oldest at
    // s_ring_u[s_ring_pos], newest at s_ring_u[(s_ring_pos - 1) mod N].
    const uint32_t start = s_ring_pos;
    for (int i = 0; i < SYSID_NEST; i++) {
        const uint32_t idx_ring = (start + i) % SYSID_RING_LEN;
        s_seg_u_out[i] = s_ring_u[idx_ring];
        s_seg_y_out[i] = s_ring_y[idx_ring];
    }
    s_seg_axis = (uint32_t)axis;
    __atomic_store_n(&s_seg_pending, (uint8_t)1, __ATOMIC_RELEASE);
}

bool sysidNotifyChirpStart(int axis)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return false;
    // Refuse if chirp amplitude on this axis is too small to identify a
    // plant — Welch on near-zero excitation produces nonsense fits.
    // 50 dps amplitude is the lowest value that produced reliable
    // numbers across our LOG00069 / LOG00072 / LOG00079 datasets.
    if (pidRuntime.chirpAmplitude[axis] < 50.0f) {
        return false;
    }
    // If the previous chirp's compute hasn't finished yet, refuse the new
    // capture. The shared (u, y) buffer is single-producer / single-consumer
    // and can't be safely reset while core1 is still reading it. The
    // caller must NOT issue a matching ChirpEnd in this case — otherwise
    // core1 would run the fit on stale data tagged for the new axis.
    if (__atomic_load_n(&s_computing_axis, __ATOMIC_ACQUIRE) != 0xff) {
        return false;
    }
    // Snapshot the controller AT CHIRP START — these are the gains that
    // were active during the chirp and that the deconvolution must use.
    // Taking the snapshot at chirp END would let any user-driven gain
    // change during the chirp leak into the deconvolution.
    s_ctrl_snap.Kp           = pidRuntime.pidCoefficient[axis].Kp;
    s_ctrl_snap.Ki           = pidRuntime.pidCoefficient[axis].Ki;
    s_ctrl_snap.Kd           = pidRuntime.pidCoefficient[axis].Kd;
    // BF stock profile uses dynamic dterm LPF1 (cutoff slides between
    // dyn_min and dyn_max with throttle). The chirp typically runs near
    // hover throttle, so use the midpoint as a representative cutoff.
    // Static-only profiles set dyn_min=0; fall back to dterm_lpf1_static_hz.
    if (pidRuntime.dynLpfMin > 0) {
        s_ctrl_snap.dterm_lpf1_hz =
            0.5f * ((float)pidRuntime.dynLpfMin + (float)pidRuntime.dynLpfMax);
    } else {
        s_ctrl_snap.dterm_lpf1_hz = (float)currentPidProfile->dterm_lpf1_static_hz;
    }
    s_ctrl_snap.gyro_lpf1_hz = (float)gyroConfig()->gyro_lpf1_static_hz;
    s_ctrl_snap.gyro_lpf2_hz = (float)gyroConfig()->gyro_lpf2_static_hz;

    // Reset streaming state and re-arm the persistent Welch accumulator
    // for this chirp. Done on core0 BEFORE any segment can land (capturing
    // flag is set last, with a release).
    s_ring_pos = 0;
    s_ring_filled = 0;
    s_samples_since_seg = 0;
    s_total_samples_captured = 0;
    s_seg_dropped = 0;
    __atomic_store_n(&s_seg_pending, (uint8_t)0, __ATOMIC_RELEASE);
    if (welch_init(&s_welch, SYSID_NEST, SYSID_NOVERLAP, s_fft_ctx,
                   s_win, s_seg, s_Ure, s_Uim, s_Yre, s_Yim,
                   s_Suu, s_Syu_re, s_Syu_im, s_Syy)) {
        s_welch_inited = true;
        s_welch_axis = (uint8_t)axis;
    } else {
        return false;
    }
    s_push_skip = 0;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&s_capturing_axis, (uint8_t)axis, __ATOMIC_RELEASE);
    return true;
}

void sysidNotifyChirpEnd(int axis)
{
    if (axis < 0 || axis >= SYSID_AXIS_COUNT) return;
    if (s_capturing_axis == (uint8_t)axis) {
        __atomic_store_n(&s_capturing_axis, (uint8_t)0xff, __ATOMIC_RELEASE);
    }
    __atomic_store_n(&s_computing_axis, (uint8_t)axis, __ATOMIC_RELEASE);
#ifdef USE_MULTICORE
    // core1 will: drain any final pending segment, then finalise + fit.
    __atomic_store_n(&s_pending_axis, (uint8_t)axis, __ATOMIC_RELEASE);
#else
    sysid_process_segment_core1();
    sysid_finalize_axis(axis);
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
