/*
 * Welch H1 estimator + MSC coherence — algorithm body.
 * See sysid_welch.h.
 */

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

static void load_demean(float *dst, const float *src, int n, float global_mean) {
    double seg_sum = 0.0;
    for (int i = 0; i < n; i++) {
        float v = src[i] - global_mean;
        dst[i] = v;
        seg_sum += (double)v;
    }
    const float seg_mean = (float)(seg_sum / (double)n);
    for (int i = 0; i < n; i++) dst[i] -= seg_mean;
}

static void mul_window(float *x, const float *w, int n) {
    for (int i = 0; i < n; i++) x[i] *= w[i];
}

void welch_process(welch_t *w, const float *inp, const float *out, int Ndata) {
    const int N = w->N;
    const int step = N - w->noverlap;
    const float invNW = 1.0f / ((float)N * w->W);

    // Global demean of inp / out (matches pichim's algorithm).
    double mu_i = 0.0, mu_o = 0.0;
    for (int i = 0; i < Ndata; i++) { mu_i += (double)inp[i]; mu_o += (double)out[i]; }
    mu_i /= (double)Ndata; mu_o /= (double)Ndata;
    const float gmi = (float)mu_i, gmo = (float)mu_o;

    int s = 0;
    while (s + N <= Ndata) {
        // U spectrum
        load_demean(w->seg, &inp[s], N, gmi);
        mul_window(w->seg, w->win, N);
        welch_fft_rfft(w->fft, w->seg, w->Ure, w->Uim);

        // Y spectrum
        load_demean(w->seg, &out[s], N, gmo);
        mul_window(w->seg, w->win, N);
        welch_fft_rfft(w->fft, w->seg, w->Yre, w->Yim);

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
