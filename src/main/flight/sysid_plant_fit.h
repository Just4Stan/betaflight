/*
 * 2nd-order rigid-body plant fit on a Welch FRF.
 *
 *   H(s) = K * wn^2 / (s^2 + 2 zeta wn s + wn^2)
 *
 * Method: Gauss-Newton on log-magnitude, weighted by coherence. Phase is
 * not used in the fit (could be added later as a second residual weighted
 * by 1 / variance(phase)). Coherence-weighted magnitude is what pichim's
 * MATLAB pipeline does for the rigid-body baseline; phase fitting only
 * pays off once you bring in delay terms.
 *
 * Convergence: 8-12 iterations for a clean rigid-body sweep, ~1000 ops per
 * iteration if Nbands ~= 50. Trivial on M33+FPU. ~150 LOC.
 *
 * The fit is over [f_min, f_max], typically [1 Hz, 100 Hz]. Bins with
 * coherence below `coh_min` are dropped.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float K;       // DC gain
    float wn;      // natural frequency [rad/s]
    float zeta;    // damping ratio
    float rmse_db; // residual RMS in dB on the bins used
    int   Nbands;  // bins kept for fit (after coherence gate)
    int   iters;   // GN iterations executed
    bool  converged;
} plant_fit_t;

// Run the fit. Inputs:
//   freq_hz[Nfreq]:     frequency axis from Welch (Hz)
//   H_re[Nfreq], H_im:  H1 estimate at each bin
//   coh[Nfreq]:         MSC at each bin
//   f_min, f_max:       inclusive band to fit (Hz)
//   coh_min:            drop bins with coh < coh_min
//   K0, wn0, zeta0:     starting guess
// Outputs:
//   *out:               final params + diagnostics
// Returns true if at least 6 bins were kept after gating.
bool plant_fit_2nd_order(const float *freq_hz,
                         const float *H_re, const float *H_im,
                         const float *coh,
                         int Nfreq,
                         float f_min, float f_max, float coh_min,
                         float K0, float wn0, float zeta0,
                         plant_fit_t *out);

// Suggest a starting guess from the FRF magnitude:
//   K0 = mean |H| in [0, f_min*2]
//   wn0 = freq of -3dB rolloff (closed-loop bandwidth proxy)
//   zeta0 = 0.7 (default critically-damped-ish)
// Returns false if no bin in low-frequency band has coh > 0.5.
bool plant_fit_initial_guess(const float *freq_hz,
                             const float *H_re, const float *H_im,
                             const float *coh,
                             int Nfreq, float f_low,
                             float *K0, float *wn0, float *zeta0);
