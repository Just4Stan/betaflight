/*
 * Welch H1 estimator + MSC coherence — minimal C port of pichim's
 * `estimate_frequency_response()` from bf_controller_tuning.
 *
 * Targets:
 *   - Host (validation):       link with welch_fft_host.c (radix-2 CFFT).
 *   - On-board (BF, RP2350):   link with welch_fft_arm.c (CMSIS-DSP
 *                              arm_rfft_fast_f32 + unpack).
 *
 * Algorithm (matches pichim's MATLAB and InsaneBroccoli's Python port,
 * cross-verified 2026-04-29 against LOG00069.BFL):
 *
 *   global demean of (inp, out)
 *   for each segment k of length N (Hann periodic, overlap = noverlap):
 *     1. segment-local demean
 *     2. window with Hann
 *     3. U = FFT(seg_u) / (N * W)   where W = sum(win) / N / 2
 *        Y = FFT(seg_y) / (N * W)
 *     4. accumulate Suu_k = U U*
 *                   Syu_k = Y U*
 *                   Syy_k = Y Y*
 *        DC and Nyquist bins get an extra 1/4 (one-sidedness + no negative-frequency mate)
 *
 *   After all segments:
 *     Suu /= Navg, Syu /= Navg, Syy /= Navg
 *     H1  = Syu / (Suu + delta)
 *     MSC = |Syu|^2 / (Suu * Syy)
 *
 * All buffers are caller-allocated; no malloc in the hot path.
 * N must be a positive even power of two.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// FFT backend handle. Backends define the struct.
typedef struct welch_fft_ctx_s welch_fft_ctx_t;

// Backend-provided. Allocates internal twiddles/scratch sized for N.
welch_fft_ctx_t *welch_fft_create(int N);
void             welch_fft_destroy(welch_fft_ctx_t *ctx);

// Runs the forward real-to-complex FFT of `in[N]` and writes the first
// Nfreq=N/2+1 complex bins to (out_re[Nfreq], out_im[Nfreq]).
// `in` may be modified by the call.
void welch_fft_rfft(welch_fft_ctx_t *ctx, float *in,
                    float *out_re, float *out_im);

// Algorithm state.
typedef struct {
    int N;
    int Nfreq;             // N/2 + 1
    int noverlap;
    welch_fft_ctx_t *fft;
    float *win;            // [N]
    float *seg;            // [N] scratch (FFT input)
    float *Ure, *Uim;      // [Nfreq] per-segment U spectrum
    float *Yre, *Yim;      // [Nfreq] per-segment Y spectrum
    float *Suu;            // [Nfreq] auto-spectra are real; Suu_im is identically zero
    float *Syu_re, *Syu_im;
    float *Syy;
    float W;
    int   Navg;
} welch_t;

// Returns false on bad N or backend init failure. All buffer pointers are
// caller-owned. Hann window is filled at init.
bool welch_init(welch_t *w, int N, int noverlap, welch_fft_ctx_t *fft,
                float *buf_win, float *buf_seg,
                float *buf_Ure, float *buf_Uim,
                float *buf_Yre, float *buf_Yim,
                float *buf_Suu, float *buf_Syu_re, float *buf_Syu_im,
                float *buf_Syy);

void welch_reset(welch_t *w);

// Process the (input, output) record start-to-end, accumulating spectra.
void welch_process(welch_t *w, const float *inp, const float *out, int Ndata);

// Average accumulated spectra over Navg segments.
void welch_finalize(welch_t *w);

// Compute H1[k] = Syu[k] / (Suu[k] + delta) and MSC[k] = |Syu|^2 / (Suu*Syy).
// Any output may be NULL to skip.
void welch_estimate_h1(const welch_t *w, float delta,
                       float *H_re, float *H_im, float *coh);

static inline float welch_bin_hz(int k, int N, float Ts) {
    return (float)k / ((float)N * Ts);
}
