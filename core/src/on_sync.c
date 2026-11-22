#include "on_sync.h"
#include <math.h>
#include <stdlib.h>

static int64_t ll_abs64(int64_t x) {
    return x < 0 ? -x : x;
}

void on_sync_init(on_sync_ctx_t *ctx) {
    if (!ctx) return;
    ctx->locked           = false;
    ctx->lock_count       = 0u;
    ctx->ref_local_ns     = 0;
    ctx->ref_net_ns       = 0;
    ctx->rate_ppb         = 0.0;
    ctx->has_prev         = false;
    ctx->prev_rx_local_ns = 0;
    ctx->prev_tx_net_ns   = 0;
    ctx->last_dt_loc_ns   = 0;
    ctx->rtt_calibrated   = false;
    ctx->link_delay_ns    = 0;
    ctx->ant_delay_ns     = 0;
    ctx->kp                = 0.65;
    ctx->ki                = 0.05;
    ctx->ka                = 0.0;
    ctx->drift_accel_ppb_s = 0.0;
    ctx->accel_en          = false;
    ctx->max_rate_ppb      = 200000.0; /* +/- 200 ppm crystal bound */
    ctx->last_err_ns       = 0;
    ctx->max_abs_err_ns    = 0;
    ctx->holdover_frames   = 0u;
}

void on_sync_enable_accel_tracker(on_sync_ctx_t *ctx, bool enable, double ka) {
    if (!ctx) return;
    ctx->accel_en = enable;
    ctx->ka       = (ka > 0.0) ? ka : 0.015;
    if (!enable) {
        ctx->drift_accel_ppb_s = 0.0;
    }
}

void on_sync_reset(on_sync_ctx_t *ctx) {
    on_sync_init(ctx);
}

int64_t on_sync_local_to_net(const on_sync_ctx_t *ctx, int64_t local_ns) {
    if (!ctx || !ctx->locked) {
        return local_ns;
    }
    double dt_loc = (double)(local_ns - ctx->ref_local_ns);
    double dt_sec = dt_loc * 1e-9;
    double eff_ppb = ctx->rate_ppb + (ctx->accel_en ? (0.5 * ctx->drift_accel_ppb_s * dt_sec) : 0.0);
    double scaled = dt_loc * (1.0 + eff_ppb * 1e-9);
    int64_t delta_net = (int64_t)llround(scaled);
    return ctx->ref_net_ns + delta_net;
}

int64_t on_sync_net_to_local(const on_sync_ctx_t *ctx, int64_t target_net_ns) {
    if (!ctx || !ctx->locked) {
        return target_net_ns;
    }
    double dt_net = (double)(target_net_ns - ctx->ref_net_ns);
    double dt_sec = dt_net * 1e-9;
    double eff_ppb = ctx->rate_ppb + (ctx->accel_en ? (0.5 * ctx->drift_accel_ppb_s * dt_sec) : 0.0);
    double inv_scale = 1.0 / (1.0 + eff_ppb * 1e-9);
    int64_t delta_loc = (int64_t)llround(dt_net * inv_scale);
    return ctx->ref_local_ns + delta_loc;
}

int64_t on_sync_on_beacon(on_sync_ctx_t *ctx, int64_t t_tx_net_ns, int64_t t_rx_local_ns) {
    if (!ctx) return 0;

    int64_t t_expected_net_ns = t_tx_net_ns + ctx->link_delay_ns + ctx->ant_delay_ns;
    ctx->holdover_frames = 0u;

    /* Acquisition step 1: First beacon -> snap phase immediately */
    if (!ctx->locked) {
        ctx->ref_local_ns     = t_rx_local_ns;
        ctx->ref_net_ns       = t_expected_net_ns;
        ctx->prev_rx_local_ns = t_rx_local_ns;
        ctx->prev_tx_net_ns   = t_expected_net_ns;
        ctx->has_prev         = true;
        ctx->locked           = true;
        ctx->lock_count       = 1u;
        ctx->last_err_ns      = 0;
        return 0;
    }

    int64_t t_pred_net_ns = on_sync_local_to_net(ctx, t_rx_local_ns);
    int64_t err_ns = t_expected_net_ns - t_pred_net_ns;

    /* Acquisition step 2: Second beacon -> direct two-point NCO frequency seed */
    if (ctx->lock_count == 1u && ctx->has_prev) {
        int64_t dt_loc = t_rx_local_ns - ctx->prev_rx_local_ns;
        int64_t dt_net = t_expected_net_ns - ctx->prev_tx_net_ns;
        ctx->last_dt_loc_ns = dt_loc;
        if (dt_loc > 100000) { /* > 100 us sanity check */
            double est_ppb = ((double)(dt_net - dt_loc) / (double)dt_loc) * 1e9;
            if (est_ppb >  ctx->max_rate_ppb) est_ppb =  ctx->max_rate_ppb;
            if (est_ppb < -ctx->max_rate_ppb) est_ppb = -ctx->max_rate_ppb;
            ctx->rate_ppb = est_ppb;
        }
        ctx->ref_local_ns     = t_rx_local_ns;
        ctx->ref_net_ns       = t_expected_net_ns;
        ctx->prev_rx_local_ns = t_rx_local_ns;
        ctx->prev_tx_net_ns   = t_expected_net_ns;
        ctx->lock_count++;
        ctx->last_err_ns = err_ns;
        return err_ns;
    }

    /* Re-anchor large step (> 50 us, e.g. after major topology jump) */
    if (ll_abs64(err_ns) > 50000) {
        ctx->ref_local_ns     = t_rx_local_ns;
        ctx->ref_net_ns       = t_expected_net_ns;
        ctx->prev_rx_local_ns = t_rx_local_ns;
        ctx->prev_tx_net_ns   = t_expected_net_ns;
        ctx->last_dt_loc_ns   = 0;
        ctx->lock_count++;
        ctx->last_err_ns = err_ns;
        return err_ns;
    }

    /* Steady-state digital NCO TC-PLL PI (+ v4.0 Second-Order Alpha-Beta-Gamma) tracking */
    int64_t dt_loc = t_rx_local_ns - ctx->prev_rx_local_ns;
    ctx->last_dt_loc_ns = dt_loc;
    if (dt_loc > 100000) {
        double dt_sec = (double)dt_loc * 1e-9;

        /* Propagate frequency by tracked thermal acceleration before rebasing reference */
        if (ctx->accel_en) {
            ctx->rate_ppb += ctx->drift_accel_ppb_s * dt_sec;
        }

        /* Re-base reference point to prevent double precision drift over long runs */
        ctx->ref_net_ns   = t_pred_net_ns + (int64_t)llround(ctx->kp * (double)err_ns);
        ctx->ref_local_ns = t_rx_local_ns;

        /* Integral NCO fractional rate update (ppb) */
        double delta_ppb = ctx->ki * ((double)err_ns / dt_sec);
        ctx->rate_ppb += delta_ppb;
        if (ctx->rate_ppb >  ctx->max_rate_ppb) ctx->rate_ppb =  ctx->max_rate_ppb;
        if (ctx->rate_ppb < -ctx->max_rate_ppb) ctx->rate_ppb = -ctx->max_rate_ppb;

        /* Second-order thermal drift-acceleration update (ppb/s) */
        if (ctx->accel_en && dt_sec > 1e-5) {
            double delta_accel = ctx->ka * ((double)err_ns / (0.5 * dt_sec * dt_sec));
            ctx->drift_accel_ppb_s += delta_accel;
            if (ctx->drift_accel_ppb_s >  500000.0) ctx->drift_accel_ppb_s =  500000.0;
            if (ctx->drift_accel_ppb_s < -500000.0) ctx->drift_accel_ppb_s = -500000.0;
        }
    }

    ctx->prev_rx_local_ns = t_rx_local_ns;
    ctx->prev_tx_net_ns   = t_expected_net_ns;
    ctx->lock_count++;
    ctx->last_err_ns = err_ns;
    if (ctx->lock_count > 4u && ll_abs64(err_ns) > ctx->max_abs_err_ns) {
        ctx->max_abs_err_ns = ll_abs64(err_ns);
    }
    return err_ns;
}

on_err_t on_sync_on_rtt(on_sync_ctx_t *ctx,
                        int64_t t1_local_ns,
                        int64_t t2_g_rx_ns,
                        int64_t t3_g_tx_ns,
                        int64_t t4_local_ns,
                        int64_t *out_tau_ns) {
    if (!ctx) return ON_ERR_INVALID_ARG;
    if (t4_local_ns <= t1_local_ns || t3_g_tx_ns < t2_g_rx_ns) {
        return ON_ERR_INVALID_ARG;
    }

    /* Convert local elapsed (T4 - T1) into G-clock domain using locked NCO slope */
    double dt_loc_in_g = (double)(t4_local_ns - t1_local_ns) * (1.0 + ctx->rate_ppb * 1e-9);
    double g_turnaround = (double)(t3_g_tx_ns - t2_g_rx_ns);
    double rtt_ns = dt_loc_in_g - g_turnaround;
    if (rtt_ns < 0.0) {
        rtt_ns = 0.0;
    }
    int64_t tau_sample = (int64_t)llround(rtt_ns * 0.5);

    int64_t old_delay = ctx->link_delay_ns;
    if (!ctx->rtt_calibrated) {
        ctx->link_delay_ns  = tau_sample;
        ctx->rtt_calibrated = true;
    } else {
        /* Smooth EWMA (alpha = 0.25) */
        ctx->link_delay_ns = (ctx->link_delay_ns * 3 + tau_sample) / 4;
    }

    /* Compensate phase reference by delta_delay so RTT step does not kick NCO frequency */
    int64_t delta_delay = ctx->link_delay_ns - old_delay;
    if (ctx->locked && delta_delay != 0) {
        ctx->ref_net_ns     += delta_delay;
        ctx->prev_tx_net_ns += delta_delay;
    }

    if (out_tau_ns) {
        *out_tau_ns = ctx->link_delay_ns;
    }
    return ON_OK;
}

void on_sync_on_missed_beacon(on_sync_ctx_t *ctx) {
    if (!ctx) return;
    ctx->holdover_frames++;
}

void on_sync_apply_followup(on_sync_ctx_t *ctx, int16_t prev_tx_err_ns) {
    if (!ctx || !ctx->locked || prev_tx_err_ns == 0) return;
    /* Exact algebraic correction for the previous beacon's true antenna departure (+prev_tx_err_ns) */
    double e = (double)prev_tx_err_ns;
    ctx->prev_tx_net_ns += (int64_t)prev_tx_err_ns;

    if (ctx->lock_count <= 1u || ctx->last_dt_loc_ns <= 100000) {
        ctx->ref_net_ns += (int64_t)prev_tx_err_ns;
    } else if (ctx->lock_count == 2u) {
        ctx->ref_net_ns += (int64_t)prev_tx_err_ns;
        ctx->rate_ppb   += (e / (double)ctx->last_dt_loc_ns) * 1e9;
    } else {
        /* Undo false PI proportional & integral kick caused by uncompensated TX queue delay */
        ctx->ref_net_ns += (int64_t)llround(ctx->kp * e);
        double dt_sec = (double)ctx->last_dt_loc_ns * 1e-9;
        ctx->rate_ppb += ctx->ki * (e / dt_sec);
    }
    if (ctx->rate_ppb >  ctx->max_rate_ppb) ctx->rate_ppb =  ctx->max_rate_ppb;
    if (ctx->rate_ppb < -ctx->max_rate_ppb) ctx->rate_ppb = -ctx->max_rate_ppb;
}

void on_sync_set_ant_delay_ns(on_sync_ctx_t *ctx, int64_t ant_delay_ns) {
    if (!ctx) return;
    int64_t delta = ant_delay_ns - ctx->ant_delay_ns;
    ctx->ant_delay_ns = ant_delay_ns;
    if (ctx->locked && delta != 0) {
        ctx->ref_net_ns     += delta;
        ctx->prev_tx_net_ns += delta;
    }
}
