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

// Welch H1 estimator + MSC coherence. Pure C port of pichim's MATLAB
// `estimate_frequency_response.m` from `pichim/bf_controller_tuning`
// (GPL-3.0). Algorithm body; see sysid_welch.h for the public API.

#include "platform.h"

#ifdef USE_SYSID

#include "sysid_welch.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static void hann_periodic(float *w, int N) {
    for (int k = 0; k < N; k++) {
        w[k] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)k / (float)N));
    }
}

bool welch_init(welch_t *w, int N, int noverlap, welch_fft_ctx_t *fft,
                float *buf_win, float *buf_seg,
                float *buf_Ure, float *buf_Uim,
                float *buf_Yre, float *buf_Yim,
                float *buf_Suu, float *buf_Syu_re, float *buf_Syu_im,
                float *buf_Syy)
{
    if (N <= 0 || (N & (N - 1)) != 0 || (N & 1)) return false;
    if (noverlap < 0 || noverlap >= N) return false;
    if (!fft) return false;

    w->N = N;
    w->Nfreq = N / 2 + 1;
    w->noverlap = noverlap;
    w->fft = fft;
    w->win = buf_win;
    w->seg = buf_seg;
    w->Ure = buf_Ure; w->Uim = buf_Uim;
    w->Yre = buf_Yre; w->Yim = buf_Yim;
    w->Suu = buf_Suu;
    w->Syu_re = buf_Syu_re; w->Syu_im = buf_Syu_im;
    w->Syy = buf_Syy;

    hann_periodic(w->win, N);

    double s = 0.0;
    for (int i = 0; i < N; i++) s += (double)w->win[i];
    w->W = (float)(s / (double)N / 2.0);

    welch_reset(w);
    return true;
}

void welch_reset(welch_t *w) {
    memset(w->Suu,    0, sizeof(float) * w->Nfreq);
    memset(w->Syu_re, 0, sizeof(float) * w->Nfreq);
    memset(w->Syu_im, 0, sizeof(float) * w->Nfreq);
    memset(w->Syy,    0, sizeof(float) * w->Nfreq);
    w->Navg = 0;
}

// (load_demean / mul_window were used by the old batch welch_process. The
// streaming + per-segment paths inline these operations now.)

void welch_process_segment(welch_t *w, float *seg_u, float *seg_y) {
    const int N = w->N;
    const float invNW = 1.0f / ((float)N * w->W);

    // Per-segment DC removal (matches pichim's algorithm). The pichim
    // batch-mode also subtracts a global mean first; that's a tiny
    // correction over per-segment demean and we drop it for streaming
    // (we'd need a running global mean over the whole chirp, which adds
    // state for negligible benefit — the per-segment demean already
    // removes the dominant low-frequency drift).
    double su = 0.0, sy = 0.0;
    for (int i = 0; i < N; i++) { su += (double)seg_u[i]; sy += (double)seg_y[i]; }
    const float mu_u = (float)(su / (double)N);
    const float mu_y = (float)(sy / (double)N);
    for (int i = 0; i < N; i++) {
        seg_u[i] = (seg_u[i] - mu_u) * w->win[i];
        seg_y[i] = (seg_y[i] - mu_y) * w->win[i];
    }
    welch_fft_rfft(w->fft, seg_u, w->Ure, w->Uim);
    welch_fft_rfft(w->fft, seg_y, w->Yre, w->Yim);

    // Apply 1/(N*W) once per segment, accumulate spectra.
    for (int k = 0; k < w->Nfreq; k++) {
        const float ur = w->Ure[k] * invNW, ui = w->Uim[k] * invNW;
        const float yr = w->Yre[k] * invNW, yi = w->Yim[k] * invNW;

        float duu = ur*ur + ui*ui;
        float dyy = yr*yr + yi*yi;
        float dyu_re = yr*ur + yi*ui;
        float dyu_im = yi*ur - yr*ui;

        if (k == 0 || k == N / 2) {
            duu *= 0.25f;
            dyy *= 0.25f;
            dyu_re *= 0.25f;
            dyu_im *= 0.25f;
        }

        w->Suu[k]    += duu;
        w->Syy[k]    += dyy;
        w->Syu_re[k] += dyu_re;
        w->Syu_im[k] += dyu_im;
    }
    w->Navg++;
}

void welch_process(welch_t *w, const float *inp, const float *out, int Ndata) {
    // Batch convenience: walk the input in N-overlap segments, copying each
    // into the welch_t's seg buffer and feeding to welch_process_segment.
    const int N = w->N;
    const int step = N - w->noverlap;
    int s = 0;
    while (s + N <= Ndata) {
        // The streaming entry takes a separate seg_y buffer; the welch_t
        // only owns one seg scratch. For batch we re-use a stack-resident
        // pointer dance: load u into seg, save aside a local y copy.
        // Simpler: process u via seg, and use seg for y too — but we need
        // both spectra per segment. Use seg for u, copy y into Yre/Yim
        // staging (dirty but works, since welch_process_segment overwrites
        // them anyway). To keep things clean, fall back to a stack array.
        float seg_y_local[1024];   // safe upper bound for N up to 1024
        for (int i = 0; i < N; i++) { w->seg[i] = inp[s + i]; seg_y_local[i] = out[s + i]; }
        welch_process_segment(w, w->seg, seg_y_local);
        s += step;
    }
}

void welch_finalize(welch_t *w) {
    if (w->Navg <= 0) return;
    const float n = 1.0f / (float)w->Navg;
    for (int k = 0; k < w->Nfreq; k++) {
        w->Suu[k]    *= n;
        w->Syy[k]    *= n;
        w->Syu_re[k] *= n;
        w->Syu_im[k] *= n;
    }
}

void welch_estimate_h1(const welch_t *w, float delta,
                       float *H_re, float *H_im, float *coh)
{
    for (int k = 0; k < w->Nfreq; k++) {
        const float Suu = w->Suu[k] + delta;
        const float Syu_r = w->Syu_re[k];
        const float Syu_i = w->Syu_im[k];
        const float Syy = w->Syy[k];
        const float invSuu = (fabsf(Suu) > 1e-30f) ? 1.0f / Suu : 0.0f;
        if (H_re) H_re[k] = Syu_r * invSuu;
        if (H_im) H_im[k] = Syu_i * invSuu;
        if (coh) {
            const float num   = Syu_r * Syu_r + Syu_i * Syu_i;
            const float denom = Suu * Syy;
            coh[k] = (denom > 1e-30f) ? (num / denom) : 0.0f;
        }
    }
}

#endif // USE_SYSID
