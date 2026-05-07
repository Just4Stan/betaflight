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

/*
 * FFT backend for sysid_welch — radix-2 in-place complex FFT, then
 * extracts the first N/2+1 bins of the real-input spectrum.
 *
 * For real input x[0..N-1], we run a full N-point CFFT (with the imaginary
 * part of input set to zero) and read bins 0..N/2. Slightly wasteful (2x
 * memory + 2x compute vs a packed half-size CFFT trick) but trivially
 * correct.
 *
 * USED BY BOTH host validation AND the on-board PICO build. CMSIS-DSP
 * arm_rfft_fast_f32 would roughly halve the compute on M33+FPU but
 * needs ~50 KB of CMSIS source folded into the PICO build; deferred
 * until the rest of the pipeline is flight-validated. See PLAN section
 * 9.5 for the trade study.
 */

#include "platform.h"

#ifdef USE_SYSID

#include "sysid_welch.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Compile-time max FFT size for on-board use. The orchestrator picks N
// at runtime but stays inside this bound. Static allocation here means no
// malloc on core1 (pico-sdk's allocator isn't core1-safe) and no
// per-instance heap fragmentation.
#define SYSID_FFT_NMAX  512

struct welch_fft_ctx_s {
    int N;
    float tw[2 * SYSID_FFT_NMAX];
    int   bitrev[SYSID_FFT_NMAX];
    float cdata[2 * SYSID_FFT_NMAX];
};

static welch_fft_ctx_t s_ctx_pool;
static bool s_ctx_initialised = false;

static int log2_int(int N) {
    int log2 = 0;
    while ((1 << log2) < N) log2++;
    return log2;
}

welch_fft_ctx_t *welch_fft_create(int N) {
    if (N <= 0 || (N & (N - 1)) != 0) return NULL;
    if (N > SYSID_FFT_NMAX) return NULL;
    welch_fft_ctx_t *ctx = &s_ctx_pool;
    if (s_ctx_initialised && ctx->N != N) {
        // Pool is single-instance: a second caller asking for a different N
        // would silently clobber the first caller's twiddle tables. Fail
        // loud rather than corrupt.
        return NULL;
    }
    if (s_ctx_initialised && ctx->N == N) {
        return ctx;
    }
    ctx->N = N;
    for (int k = 0; k < N; k++) {
        const float a = -2.0f * (float)M_PI * (float)k / (float)N;
        ctx->tw[2*k]     = cosf(a);
        ctx->tw[2*k + 1] = sinf(a);
    }
    const int log2N = log2_int(N);
    for (int i = 0; i < N; i++) {
        int x = i, r = 0;
        for (int b = 0; b < log2N; b++) { r = (r << 1) | (x & 1); x >>= 1; }
        ctx->bitrev[i] = r;
    }
    s_ctx_initialised = true;
    return ctx;
}

void welch_fft_destroy(welch_fft_ctx_t *ctx) {
    // Static pool — nothing to free. Keep the symbol so non-PICO code paths
    // (offline host validation) that pair create/destroy still link.
    (void)ctx;
}

static void cfft_radix2(float *data, int N, const float *tw, const int *br) {
    // bit-reverse permute
    for (int i = 0; i < N; i++) {
        int j = br[i];
        if (j > i) {
            float tr = data[2*i],     ti = data[2*i + 1];
            data[2*i]     = data[2*j];     data[2*i + 1] = data[2*j + 1];
            data[2*j]     = tr;            data[2*j + 1] = ti;
        }
    }
    // butterflies
    for (int s = 1; s < N; s <<= 1) {
        const int span = s << 1;
        const int twstep = N / span;
        for (int k = 0; k < N; k += span) {
            for (int j = 0; j < s; j++) {
                const int twi = j * twstep;
                const float wr = tw[2*twi];
                const float wi = tw[2*twi + 1];
                const int   ai = 2 * (k + j);
                const int   bi = 2 * (k + j + s);
                const float ar = data[ai],     aim = data[ai + 1];
                const float br_ = data[bi],    bim = data[bi + 1];
                const float tr = wr * br_  - wi * bim;
                const float ti = wr * bim  + wi * br_;
                data[ai]     = ar  + tr;
                data[ai + 1] = aim + ti;
                data[bi]     = ar  - tr;
                data[bi + 1] = aim - ti;
            }
        }
    }
}

void welch_fft_rfft(welch_fft_ctx_t *ctx, float *in,
                    float *out_re, float *out_im)
{
    const int N = ctx->N;
    // Pack real -> complex
    for (int i = 0; i < N; i++) {
        ctx->cdata[2*i]     = in[i];
        ctx->cdata[2*i + 1] = 0.0f;
    }
    cfft_radix2(ctx->cdata, N, ctx->tw, ctx->bitrev);
    // Extract bins 0..N/2
    for (int k = 0; k <= N / 2; k++) {
        out_re[k] = ctx->cdata[2*k];
        out_im[k] = ctx->cdata[2*k + 1];
    }
}

#endif // USE_SYSID
