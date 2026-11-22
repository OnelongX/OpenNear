/*
 * OpenNear v3.0 Route B (SDR / FPGA L1 Physical Layer Reference Benchmark)
 *
 * Benchmarks three core L1 PHY innovations:
 *   1. 8 us Short Dedicated Preamble (32-chip @ 4 Msps) under AWGN + Carrier Frequency
 *      Offset (CFO +/- 40 ppm @ 2.44 GHz = 97.6 kHz) + 2-Tap Rayleigh Multipath Fading
 *      with differential phase CFO coarse estimation & compensation.
 *   2. Float vs. Fixed-Point (Q4.4 int16_t, zero-float) CA-Polar (N=128, K=48, CA-SCL L=8)
 *      BLER sweep across Es/N0 = -3 dB .. +2 dB.
 *   3. Adaptive Modulation & Coding (AMC) dual-waveform (LE_MSK_2M <-> LE_QPSK_4M <-> HS_OFDM_20M)
 *      hysteresis tracking under fading SNR sweep.
 */
#include "on_polar.h"
#include "on_amc.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint64_t s_rng = 0x9E3779B97F4A7C15ULL;
static double urand(void) {
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 7;
    s_rng ^= s_rng << 17;
    return ((double)(s_rng & 0xFFFFFFFFULL) + 1.0) / 4294967297.0;
}

static double grand(void) {
    return sqrt(-2.0 * log(urand())) * cos(6.283185307179586 * urand());
}

/* 32-chip periodic 2x16 Golay-structured short preamble (8.0 us @ 4 Msps)
 * Identical two 16-chip halves allow Schmidl-Cox differential CFO coarse estimation! */
static const int8_t kShortHalf16[16] = {
    +1,+1,+1,-1,+1,+1,-1,+1,+1,+1,+1,-1,-1,-1,+1,-1
};

static void bench_preamble_cfo_multipath(void) {
    printf("=== 1. L1 8 us Short Preamble under CFO (+40 ppm = 97.6 kHz) + 2-Tap Rayleigh Fading ===\n");
    const double fs_hz = 4.0e6;
    const double cfo_hz = 97600.0; /* +40 ppm @ 2.44 GHz */
    const double two_pi = 6.283185307179586;
    const int rx_len = 160;
    const int true_idx = 64;
    const double snr_db_list[3] = { -3.0, 0.0, +6.0 };

    for (int s = 0; s < 3; ++s) {
        double snr_db = snr_db_list[s];
        double sigma = sqrt(1.0 / (2.0 * pow(10.0, snr_db / 10.0)));
        double sum_time_err_ns = 0.0;
        double sum_cfo_err_hz  = 0.0;
        int trials = 200;
        int exact_hits = 0;

        for (int t = 0; t < trials; ++t) {
            /* 2-tap multipath channel: dominant LOS + delayed scatterer (0.25 amplitude, random phase) */
            double mp_amp = 0.25;
            double mp_ang = urand() * two_pi;
            double h1_re = mp_amp * cos(mp_ang);
            double h1_im = mp_amp * sin(mp_ang);

            double rx_re[160], rx_im[160];
            for (int i = 0; i < rx_len; ++i) {
                double s0 = 0.0, s1 = 0.0;
                if (i >= true_idx && i < true_idx + 32) {
                    s0 = (double)kShortHalf16[(i - true_idx) & 15];
                }
                if (i - 1 >= true_idx && i - 1 < true_idx + 32) {
                    s1 = (double)kShortHalf16[(i - 1 - true_idx) & 15];
                }
                double ch_re = s0 + h1_re * s1;
                double ch_im = h1_im * s1;
                double ph = two_pi * cfo_hz * ((double)i / fs_hz);
                double c = cos(ph), sn = sin(ph);
                rx_re[i] = ch_re * c - ch_im * sn + sigma * grand();
                rx_im[i] = ch_re * sn + ch_im * c + sigma * grand();
            }

            /* Non-coherent split-half matched filter (robust to +/- 100 kHz CFO!) */
            double best_mag = -1.0;
            int best_idx = 0;
            double best_p_re = 0.0, best_p_im = 0.0;
            for (int i = 0; i <= rx_len - 32; ++i) {
                double a_re = 0.0, a_im = 0.0, b_re = 0.0, b_im = 0.0;
                for (int k = 0; k < 16; ++k) {
                    double ref = (double)kShortHalf16[k];
                    a_re += rx_re[i + k]      * ref;
                    a_im += rx_im[i + k]      * ref;
                    b_re += rx_re[i + 16 + k] * ref;
                    b_im += rx_im[i + 16 + k] * ref;
                }
                /* Combined half-correlator energy (invariant to carrier rotation across 16 samples) */
                double mag = (a_re * a_re + a_im * a_im) + (b_re * b_re + b_im * b_im);
                if (mag > best_mag) {
                    best_mag = mag;
                    best_idx = i;
                    /* Cross-correlation P = conj(A) * B -> angle(P) = 2*pi*cfo*(16/fs) */
                    best_p_re = a_re * b_re + a_im * b_im;
                    best_p_im = a_re * b_im - a_im * b_re;
                }
            }

            if (best_idx == true_idx) exact_hits++;
            sum_time_err_ns += fabs((double)(best_idx - true_idx) * (1e9 / fs_hz));
            double cfo_est_hz = atan2(best_p_im, best_p_re) * (fs_hz / (two_pi * 16.0));
            sum_cfo_err_hz += fabs(cfo_est_hz - cfo_hz);
        }
        printf("  SNR=%+4.1f dB | Peak Hit=%5.1f%% | Timing Err=%5.1f ns | Mean CFO Est Err=%5.1f kHz\n",
               snr_db, 100.0 * exact_hits / trials, sum_time_err_ns / trials,
               (sum_cfo_err_hz / trials) * 1e-3);
    }
}

static void bench_polar_float_vs_fixed(void) {
    printf("\n=== 2. L1 CA-Polar (N=128, K=48, L=8): Float vs. Embedded Fixed-Point Q4.4 ===\n");
    const double esn0_db_list[5] = { -3.0, -2.0, -1.0, 0.0, +2.0 };
    const int trials = 200;

    for (int s = 0; s < 5; ++s) {
        double esn0_db = esn0_db_list[s];
        double snr_lin = pow(10.0, esn0_db / 10.0);
        double sigma   = sqrt(1.0 / (2.0 * snr_lin));
        int ok_flt = 0, ok_fix = 0;

        for (int t = 0; t < trials; ++t) {
            uint8_t msg[ON_POLAR_MSG_BYTES];
            uint8_t coded[ON_POLAR_N];
            uint8_t rec_flt[ON_POLAR_MSG_BYTES], rec_fix[ON_POLAR_MSG_BYTES];
            float llr_f[ON_POLAR_N];
            int16_t llr_q4[ON_POLAR_N];

            for (int b = 0; b < 5; ++b) msg[b] = (uint8_t)((t * 53 + b * 29) & 0xFF);
            on_polar_encode_block(msg, coded);
            for (uint32_t i = 0; i < ON_POLAR_N; ++i) {
                double sym = coded[i] ? -1.0 : +1.0;
                double rx = sym + sigma * grand();
                double l = 2.0 * rx / (sigma * sigma);
                llr_f[i] = (float)l;
                int32_t q = (int32_t)llround(l * 16.0);
                if (q >  511) q =  511;
                if (q < -512) q = -512;
                llr_q4[i] = (int16_t)q;
            }
            if (on_polar_decode_block(llr_f, rec_flt) == ON_OK && memcmp(msg, rec_flt, 5) == 0) {
                ok_flt++;
            }
            if (on_polar_decode_block_fixed(llr_q4, rec_fix) == ON_OK && memcmp(msg, rec_fix, 5) == 0) {
                ok_fix++;
            }
        }
        printf("  Es/N0=%+4.1f dB | Float BLER=%.4f (%3d/%3d) | Q4.4 Fixed BLER=%.4f (%3d/%3d)\n",
               esn0_db,
               1.0 - (double)ok_flt / trials, ok_flt, trials,
               1.0 - (double)ok_fix / trials, ok_fix, trials);
    }
}

static void bench_amc_dual_waveform(void) {
    printf("\n=== 3. L1 Dual-Waveform AMC Hysteresis (LE_MSK_2M <-> LE_QPSK_4M <-> HS_OFDM_20M) ===\n");
    on_amc_ctx_t amc;
    on_amc_init(&amc, ON_PHY_LE_MSK_2M);
    for (int i = 0; i < 12; ++i) on_amc_update(&amc, 12.0f, true);
    on_phy_mode_t m1 = amc.current_mode;
    for (int i = 0; i < 12; ++i) on_amc_update(&amc, 22.0f, true);
    on_phy_mode_t m2 = amc.current_mode;
    for (int i = 0; i < 6; ++i)  on_amc_update(&amc, 3.0f, false);
    on_phy_mode_t m3 = amc.current_mode;
    printf("  12dB->mode=%d (4M QPSK) | 22dB->mode=%d (20M OFDM) | 3dB Fade->mode=%d (2M MSK) | up=%u down=%u\n",
           (int)m1, (int)m2, (int)m3, amc.upgrades, amc.downgrades);
}

int main(void) {
    bench_preamble_cfo_multipath();
    bench_polar_float_vs_fixed();
    bench_amc_dual_waveform();
    return 0;
}
