#include "on_ranging.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double wrap_pi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void on_ranging_init(on_ranging_ctx_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

void on_ranging_clear_tones(on_ranging_ctx_t *ctx) {
    if (!ctx) return;
    memset(ctx->has_tone, 0, sizeof(ctx->has_tone));
    memset(ctx->phase_rad, 0, sizeof(ctx->phase_rad));
    ctx->tone_count = 0u;
}

void on_ranging_add_tone(on_ranging_ctx_t *ctx, uint8_t ch_idx, double two_way_phase_rad) {
    if (!ctx || ch_idx >= ON_AFH_NUM_CHANNELS) return;
    if (!ctx->has_tone[ch_idx]) {
        ctx->has_tone[ch_idx] = true;
        ctx->tone_count++;
    }
    ctx->phase_rad[ch_idx] = wrap_pi(two_way_phase_rad);
}

double on_ranging_synth_tone_phase(uint8_t ch_idx, double true_dist_m, double phase_noise_rad) {
    double f_hz = (2400.0 + (double)on_afh_ch_to_rf_offset_mhz(ch_idx)) * 1e6;
    double phi = -4.0 * M_PI * f_hz * (true_dist_m / ON_RANGING_SPEED_OF_LIGHT_MPS) + phase_noise_rad;
    return wrap_pi(phi);
}

on_err_t on_ranging_solve(on_ranging_ctx_t *ctx,
                          double rtt_ns,
                          double geofence_max_m,
                          double dt_sec,
                          on_ranging_result_t *out_res) {
    if (!ctx || !out_res || ctx->tone_count < 2u) {
        return ON_ERR_INVALID_ARG;
    }

    double f_mhz[ON_AFH_NUM_CHANNELS];
    double phi[ON_AFH_NUM_CHANNELS];
    uint32_t n = 0u;

    for (uint8_t ch = 0u; ch < ON_AFH_NUM_CHANNELS; ++ch) {
        if (ctx->has_tone[ch]) {
            f_mhz[n] = 2400.0 + (double)on_afh_ch_to_rf_offset_mhz(ch);
            phi[n]   = ctx->phase_rad[ch];
            n++;
        }
    }

    /* Stage 1: Coarse unambiguous slope from minimum-step (4..6 MHz) adjacent pairs */
    double sum_slope = 0.0;
    double sum_w     = 0.0;
    for (uint32_t i = 1u; i < n; ++i) {
        double df = f_mhz[i] - f_mhz[i - 1u];
        if (df > 0.5 && df <= 6.5) {
            double dphi = wrap_pi(phi[i] - phi[i - 1u]);
            sum_slope += (dphi / df);
            sum_w     += 1.0;
        }
    }
    double coarse_slope = (sum_w > 0.0)
        ? (sum_slope / sum_w)
        : (wrap_pi(phi[1] - phi[0]) / (f_mhz[1] - f_mhz[0]));

    /* Stage 2: Unwrap all tones across the full 76 MHz aperture using coarse_slope */
    double unwrapped[ON_AFH_NUM_CHANNELS];
    unwrapped[0] = phi[0];
    for (uint32_t i = 1u; i < n; ++i) {
        double pred = unwrapped[0] + coarse_slope * (f_mhz[i] - f_mhz[0]);
        double cycles = round((pred - phi[i]) / (2.0 * M_PI));
        unwrapped[i] = phi[i] + cycles * 2.0 * M_PI;
    }

    /* Stage 3: Full-aperture Ordinary Least Squares (OLS) phase-slope fit */
    double mean_f = 0.0, mean_p = 0.0;
    for (uint32_t i = 0u; i < n; ++i) {
        mean_f += f_mhz[i];
        mean_p += unwrapped[i];
    }
    mean_f /= (double)n;
    mean_p /= (double)n;

    double num = 0.0, den = 0.0;
    for (uint32_t i = 0u; i < n; ++i) {
        double df = f_mhz[i] - mean_f;
        double dp = unwrapped[i] - mean_p;
        num += df * dp;
        den += df * df;
    }
    double fine_slope_rad_per_hz = (den > 1e-9) ? ((num / den) * 1e-6) : (coarse_slope * 1e-6);

    double d_mcpd = -(ON_RANGING_SPEED_OF_LIGHT_MPS / (4.0 * M_PI)) * fine_slope_rad_per_hz;
    if (d_mcpd < 0.0) d_mcpd = 0.0;

    /* 4-Timestamp ToF distance upper bound: d_ToF = c * (RTT / 2) */
    double d_tof = (rtt_ns > 0.0)
        ? (0.5 * (rtt_ns * 1e-9) * ON_RANGING_SPEED_OF_LIGHT_MPS)
        : d_mcpd;

    /* Anti-Relay Attack Detection: ToF vs MCPD discrepancy or ToF stretch */
    bool relay_attack = (fabs(d_tof - d_mcpd) > ON_RANGING_RELAY_SLACK_M);

    /* 1D Alpha-Beta Kinematic Filter update */
    const double alpha = 0.65;
    const double beta  = 0.20;
    if (!ctx->filter_inited || dt_sec <= 1e-6) {
        ctx->filtered_dist_m = d_mcpd;
        ctx->velocity_mps    = 0.0;
        ctx->filter_inited   = true;
    } else {
        double pred_d = ctx->filtered_dist_m + ctx->velocity_mps * dt_sec;
        double resid  = d_mcpd - pred_d;
        ctx->filtered_dist_m = pred_d + alpha * resid;
        ctx->velocity_mps    = ctx->velocity_mps + (beta / dt_sec) * resid;
    }

    out_res->d_mcpd_m              = d_mcpd;
    out_res->d_tof_m               = d_tof;
    out_res->filtered_dist_m       = ctx->filtered_dist_m;
    out_res->velocity_mps          = ctx->velocity_mps;
    out_res->relay_attack_detected = relay_attack;
    out_res->geofence_pass         = (!relay_attack) && (ctx->filtered_dist_m <= geofence_max_m) &&
                                     (d_tof <= geofence_max_m + ON_RANGING_RELAY_SLACK_M);
    return ON_OK;
}
