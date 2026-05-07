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

// 2nd-order rigid-body plant fit via Gauss-Newton on coherence-weighted
// log-magnitude. Adapted from pichim's MATLAB
// `calculate_step_response_from_frd.m` and `lib/get_fcut_from_*.m`
// (GPL-3.0). See sysid_plant_fit.h for the public API.

#include "platform.h"

#ifdef USE_SYSID

#include "sysid_plant_fit.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// log10(|H_model(f; K, wn, zeta)|), where wn is in rad/s and f in Hz.
static inline float log10_mag_model(float f, float K, float wn, float zeta) {
    const float w = 2.0f * (float)M_PI * f;
    const float real = wn * wn - w * w;
    const float imag = 2.0f * zeta * wn * w;
    const float denom2 = real * real + imag * imag;
    const float num2   = K * K * wn * wn * wn * wn;
    return 0.5f * log10f(num2 / denom2 + 1e-30f);
}

// Partials of log10|H| w.r.t. (logK, wn, zeta). We parameterise with logK
// (instead of K) because the residual is in log-magnitude — equal step in
// logK == equal step in dB, gives a much better-conditioned Jacobian.
//
// d(log10|H|)/d(logK) = ln(10) factor cancels: d(log10 K)/d(logK) = 1/ln(10) ... actually:
// We write log10|H| = log10(K) + log10(wn^2) - 0.5 * log10(D), with D = real^2+imag^2.
// log10(K) = logK / ln(10), so d(log10|H|)/d(logK) = 1 / ln(10).
//
// Easier: just work in natural-log internally, convert to dB at the end.
//
// We choose to keep the fit in log10 so the rmse_db output is straightforward.
// Below: dlog10|H|/dlogK = 1/ln(10). dlog10|H|/dwn and dlog10|H|/dzeta computed analytically.

static inline void log10_mag_grad(float f, float K, float wn, float zeta,
                                  float *dlogK, float *dwn, float *dzeta) {
    const float w = 2.0f * (float)M_PI * f;
    const float w2 = w * w;
    const float wn2 = wn * wn;
    const float real = wn2 - w2;
    const float imag = 2.0f * zeta * wn * w;
    const float D = real * real + imag * imag + 1e-30f;
    const float ln10 = logf(10.0f);

    // We parameterise by logK := log10(K), so K = 10^logK and:
    //   log10|H| = logK + 2 log10(wn) - 0.5 log10(D)
    //
    // d(log10|H|)/d(logK)  = 1
    // d(log10|H|)/d(wn)    = (2/wn)/ln10 - 0.5 / (D ln10) * dD/dwn
    // d(log10|H|)/d(zeta)  =            - 0.5 / (D ln10) * dD/dzeta
    //
    //   D        = (wn^2 - w^2)^2 + (2 zeta wn w)^2
    //   dD/dwn   = 4 wn (wn^2 - w^2) + 4 zeta w * (2 zeta wn w)
    //            = 4 wn real + 4 zeta w * imag
    //   dD/dzeta = 2 (2 zeta wn w) * (2 wn w) = 4 wn w * imag
    const float dD_dwn   = 4.0f * wn * real + 4.0f * zeta * w * imag;
    const float dD_dzeta = 4.0f * wn * w * imag;

    *dlogK = 1.0f;
    *dwn   = (2.0f / wn) / ln10 - 0.5f * dD_dwn   / (D * ln10);
    *dzeta =                    - 0.5f * dD_dzeta / (D * ln10);
    (void)K;
}

bool plant_fit_initial_guess(const float *freq_hz,
                             const float *H_re, const float *H_im,
                             const float *coh,
                             int Nfreq, float f_low,
                             float *K0, float *wn0, float *zeta0)
{
    // K0 = geometric mean of |H| over low-frequency bins with coh > 0.5,
    // computed as mean(log10|H|) → 10^mean. Matches the log-domain
    // residual that the GN fit minimises later, so the initial guess
    // and the optimum live in the same space (linear-mean-of-magnitudes
    // would bias K0 high in noise-dominated bins).
    double logK_acc = 0.0;
    int    K_n = 0;
    for (int k = 0; k < Nfreq; k++) {
        if (freq_hz[k] > f_low) break;
        if (coh[k] > 0.5f) {
            const float m = sqrtf(H_re[k]*H_re[k] + H_im[k]*H_im[k]);
            if (m > 0.0f) { logK_acc += (double)log10f(m); K_n++; }
        }
    }
    if (K_n < 2) return false;
    *K0 = (float)pow(10.0, logK_acc / (double)K_n);

    // wn0 = freq where |H| first drops 3 dB below K0. Scan from HIGH to
    // LOW so that for a 2nd-order LP the rolloff is found at the corner
    // (low-to-high scan would lock onto the first dip at any noise floor).
    const float thresh = (*K0) * 0.70794578f;  // -3 dB linear
    *wn0 = 2.0f * (float)M_PI * freq_hz[Nfreq - 1] * 0.1f;  // fallback
    for (int k = Nfreq - 1; k >= 0; k--) {
        if (coh[k] < 0.3f) continue;
        const float m = sqrtf(H_re[k]*H_re[k] + H_im[k]*H_im[k]);
        if (m >= thresh) {
            *wn0 = 2.0f * (float)M_PI * freq_hz[k];
            break;
        }
    }
    *zeta0 = 0.7f;
    return true;
}

// File-local scratch for the band-of-interest index list. plant_fit_2nd_order
// is called only from sysid_compute_axis on core1 and is not reentrant — a
// 3 KB stack allocation here would risk overflowing the 4 KB core1 stack.
enum { PLANT_FIT_MAX_BANDS = 128 };
static int   s_pf_core1_idx[PLANT_FIT_MAX_BANDS];
static float s_pf_core1_w[PLANT_FIT_MAX_BANDS];
static float s_pf_core1_y[PLANT_FIT_MAX_BANDS];

bool plant_fit_2nd_order(const float *freq_hz,
                         const float *H_re, const float *H_im,
                         const float *coh,
                         int Nfreq,
                         float f_min, float f_max, float coh_min,
                         float K0, float wn0, float zeta0,
                         plant_fit_t *out)
{
    int   *idx = s_pf_core1_idx;
    float *w_w = s_pf_core1_w;
    float *y   = s_pf_core1_y;
    int Nb = 0;
    enum { MAX_BANDS = PLANT_FIT_MAX_BANDS };
    for (int k = 0; k < Nfreq && Nb < MAX_BANDS; k++) {
        const float f = freq_hz[k];
        if (f < f_min || f > f_max) continue;
        if (coh[k] < coh_min) continue;
        const float m = sqrtf(H_re[k]*H_re[k] + H_im[k]*H_im[k]);
        if (m <= 0.0f) continue;
        idx[Nb] = k;
        w_w[Nb] = coh[k] * coh[k];
        y[Nb]   = log10f(m);
        Nb++;
    }
    if (Nb < 6) {
        out->Nbands = Nb;
        out->iters = 0;
        out->converged = false;
        return false;
    }

    float logK = log10f(fabsf(K0) + 1e-12f);
    float wn   = wn0;
    float zeta = zeta0;
    // Adaptive Levenberg damping. lam grows on uphill rejection, shrinks
    // on accepted improvement. Standard trust-region semantics with the
    // Marquardt correction (multiplicative on the diagonal).
    float lam_lm = 1.0e-3f;

    // Clean data converges in 8–12 iters; pathological data wastes compute
    // without converging on a meaningful answer. 30 is a generous budget.
    const int   max_iter   = 30;
    // Per-parameter convergence tolerances. logK and zeta are unitless so
    // an absolute tolerance is meaningful; wn is a frequency in rad/s so
    // we use a relative tolerance against wn itself. Earlier all-in-one
    // 5e-5 mixed dimensions and was unreachable on real data — GN hit
    // max_iter every time even on textbook inputs.
    const float tol_logK   = 1e-3f;   // ~0.02 dB step
    const float tol_zeta   = 5e-3f;
    const float tol_wn_rel = 5e-3f;   // 0.5 % of current wn
    int it = 0;
    bool converged = false;

    // Compute the weighted SSE at a (logK,wn,zeta) point.
    #define COMPUTE_SSE(_logK, _wn, _zeta) ({                                  \
        const float _K = powf(10.0f, (_logK));                                 \
        double _sse = 0.0;                                                     \
        for (int _n = 0; _n < Nb; _n++) {                                      \
            const int _k = idx[_n];                                            \
            const float _r = y[_n] - log10_mag_model(freq_hz[_k], _K, (_wn), (_zeta)); \
            _sse += (double)w_w[_n] * (double)(_r * _r);                       \
        }                                                                      \
        _sse; })

    double sse_curr = COMPUTE_SSE(logK, wn, zeta);

    for (it = 0; it < max_iter; it++) {
        // J^T W J  is 3x3, J^T W r is 3x1.
        float JtWJ[3][3] = {{0}};
        float JtWr[3]    = {0,0,0};
        const float K = powf(10.0f, logK);

        for (int n = 0; n < Nb; n++) {
            const int k = idx[n];
            const float f = freq_hz[k];
            float dK_, dwn_, dzeta_;
            log10_mag_grad(f, K, wn, zeta, &dK_, &dwn_, &dzeta_);
            const float ymodel = log10_mag_model(f, K, wn, zeta);
            const float r = y[n] - ymodel;
            const float ww = w_w[n];

            JtWJ[0][0] += ww * dK_   * dK_;
            JtWJ[0][1] += ww * dK_   * dwn_;
            JtWJ[0][2] += ww * dK_   * dzeta_;
            JtWJ[1][1] += ww * dwn_  * dwn_;
            JtWJ[1][2] += ww * dwn_  * dzeta_;
            JtWJ[2][2] += ww * dzeta_* dzeta_;

            JtWr[0] += ww * dK_   * r;
            JtWr[1] += ww * dwn_  * r;
            JtWr[2] += ww * dzeta_* r;
        }
        // symmetrise
        JtWJ[1][0] = JtWJ[0][1];
        JtWJ[2][0] = JtWJ[0][2];
        JtWJ[2][1] = JtWJ[1][2];

        // Marquardt-style multiplicative damping: λ scales the diagonal
        // proportionally to its current magnitude. Larger λ → step
        // closer to gradient descent (small but always downhill);
        // smaller λ → closer to Gauss-Newton (fast near the optimum).
        const float diag0 = JtWJ[0][0] * (1.0f + lam_lm);
        const float diag1 = JtWJ[1][1] * (1.0f + lam_lm);
        const float diag2 = JtWJ[2][2] * (1.0f + lam_lm);
        JtWJ[0][0] = diag0 + 1e-12f;
        JtWJ[1][1] = diag1 + 1e-12f;
        JtWJ[2][2] = diag2 + 1e-12f;

        // 3x3 solve via cofactor expansion (small matrix, no need for LU).
        const float a = JtWJ[0][0], b = JtWJ[0][1], c = JtWJ[0][2];
        const float d = JtWJ[1][0], e = JtWJ[1][1], fe = JtWJ[1][2];
        const float g = JtWJ[2][0], h = JtWJ[2][1], i = JtWJ[2][2];
        const float det = a*(e*i - fe*h) - b*(d*i - fe*g) + c*(d*h - e*g);
        if (fabsf(det) < 1e-20f) break;
        const float inv_det = 1.0f / det;
        const float A = (e*i - fe*h) * inv_det;
        const float B = (c*h - b*i) * inv_det;
        const float C = (b*fe - c*e) * inv_det;
        const float D = (fe*g - d*i) * inv_det;
        const float E = (a*i - c*g) * inv_det;
        const float F = (c*d - a*fe) * inv_det;
        const float G = (d*h - e*g) * inv_det;
        const float Hh = (b*g - a*h) * inv_det;
        const float I = (a*e - b*d) * inv_det;

        const float dlogK = A*JtWr[0] + B*JtWr[1] + C*JtWr[2];
        const float dwn   = D*JtWr[0] + E*JtWr[1] + F*JtWr[2];
        const float dzeta = G*JtWr[0] + Hh*JtWr[1] + I*JtWr[2];

        // Trial step with bound check.
        const float logK_n = logK + dlogK;
        const float wn_n   = wn   + dwn;
        const float zeta_n = zeta + dzeta;
        const bool in_bounds = (wn_n > 1e-3f && zeta_n > 0.02f && zeta_n < 4.0f);

        // Reject the step if SSE went up OR we left bounds, and grow λ.
        // Accept and shrink λ otherwise. Real LM trust-region update.
        if (in_bounds) {
            const double sse_new = COMPUTE_SSE(logK_n, wn_n, zeta_n);
            if (sse_new < sse_curr) {
                logK = logK_n; wn = wn_n; zeta = zeta_n;
                sse_curr = sse_new;
                lam_lm *= 0.5f;     // step worked → trust GN more
                if (lam_lm < 1e-7f) lam_lm = 1e-7f;
                if (fabsf(dlogK) < tol_logK &&
                    fabsf(dzeta) < tol_zeta &&
                    fabsf(dwn) / (wn + 1e-6f) < tol_wn_rel) {
                    converged = true; it++; break;
                }
                continue;
            }
        }
        // Uphill or out-of-bounds: bump damping and re-try next iter.
        lam_lm *= 4.0f;
        if (lam_lm > 1e6f) break;   // diverged
    }
    #undef COMPUTE_SSE

    // RMSE in dB at convergence
    const float K = powf(10.0f, logK);
    double sse = 0.0; double wsum = 0.0;
    for (int n = 0; n < Nb; n++) {
        const int k = idx[n];
        const float f = freq_hz[k];
        const float r_log10 = y[n] - log10_mag_model(f, K, wn, zeta);
        const float r_db = 20.0f * r_log10;
        sse += (double)w_w[n] * (double)(r_db * r_db);
        wsum += (double)w_w[n];
    }
    out->K = K;
    out->wn = wn;
    out->zeta = zeta;
    out->rmse_db = (wsum > 0.0) ? (float)sqrt(sse / wsum) : 0.0f;
    out->Nbands = Nb;
    out->iters = it;
    out->converged = converged;
    return true;
}

#endif // USE_SYSID
