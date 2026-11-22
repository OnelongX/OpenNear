#ifndef OPENNEAR_SYNC_H
#define OPENNEAR_SYNC_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear L3 3-Step Sub-Microsecond Hardware Sync Engine
 *
 * Step 1: Hardware preamble correlator capture (t_rx_local_ns)
 * Step 2: 4-timestamp bidirectional RTT link-delay calibration (tau_link_ns)
 * Step 3: Digital NCO / TC-PLL PI loop (offset + fractional ppb frequency lock)
 */
typedef struct {
    bool     locked;
    uint32_t lock_count;

    /* Linear affine NCO model: net_ns = ref_net_ns + (loc - ref_loc) * (1 + rate_ppb / 1e9) */
    int64_t  ref_local_ns;
    int64_t  ref_net_ns;
    double   rate_ppb;        /* Fractional frequency correction in parts-per-billion (1 ppm = 1000 ppb) */

    /* Previous sample for 2-point fast frequency acquisition & PTP follow-up exact inverse */
    bool     has_prev;
    int64_t  prev_rx_local_ns;
    int64_t  prev_tx_net_ns;
    int64_t  last_dt_loc_ns;

    /* Step 2: Calibrated one-way propagation + analog frontend / antenna group delay */
    bool     rtt_calibrated;
    int64_t  link_delay_ns;
    int64_t  ant_delay_ns;    /* v3.1: Calibrated board-level RF balun/antenna group delay */

    /* PI Controller tuning */
    double   kp;              /* Proportional phase gain (0..1) */
    double   ki;              /* Integral frequency gain (ppb per ns error per second) */
    double   max_rate_ppb;    /* Clamp for crystal tolerance (default +/- 200,000 ppb = +/- 200 ppm) */

    /* Telemetry */
    int64_t  last_err_ns;
    int64_t  max_abs_err_ns;
    uint32_t holdover_frames; /* Consecutive missed SYNC frames during coasting */
} on_sync_ctx_t;

void    on_sync_init(on_sync_ctx_t *ctx);
void    on_sync_reset(on_sync_ctx_t *ctx);

/*
 * Step 1 + Step 3: Feed a hardware-captured SYNC/SUB_SYNC reception.
 * t_tx_net_ns: exact network TX timestamp carried in SYNC payload
 * t_rx_local_ns: exact local hardware capture timestamp at preamble correlation instant
 * Returns residual phase error in nanoseconds (before correction).
 */
int64_t on_sync_on_beacon(on_sync_ctx_t *ctx, int64_t t_tx_net_ns, int64_t t_rx_local_ns);

/*
 * Step 2: Feed a 4-timestamp bidirectional RTT measurement (T1, T2, T3, T4) in ns.
 * Computes one-way link + analog frontend delay tau_link and updates EWMA estimate.
 */
on_err_t on_sync_on_rtt(on_sync_ctx_t *ctx,
                        int64_t t1_local_ns,
                        int64_t t2_g_rx_ns,
                        int64_t t3_g_tx_ns,
                        int64_t t4_local_ns,
                        int64_t *out_tau_ns);

/* Convert local hardware monotonic timestamp (ns) -> synchronized global network time (ns) */
int64_t on_sync_local_to_net(const on_sync_ctx_t *ctx, int64_t local_ns);

/* Convert desired global network time (ns) -> exact local hardware timer trigger timestamp (ns) */
int64_t on_sync_net_to_local(const on_sync_ctx_t *ctx, int64_t target_net_ns);

/* Notify sync engine of a missed SYNC frame (increments coasting/holdover counter) */
void    on_sync_on_missed_beacon(on_sync_ctx_t *ctx);

/*
 * v3.1 IEEE 1588 PTP Two-Step Follow-Up & Board Antenna Delay Calibration:
 * Retroactively corrects the previous SYNC beacon's actual TX antenna departure offset
 * (prev_tx_err_ns) before locking to the current beacon, eliminating software TX queue jitter!
 */
void    on_sync_apply_followup(on_sync_ctx_t *ctx, int16_t prev_tx_err_ns);
void    on_sync_set_ant_delay_ns(on_sync_ctx_t *ctx, int64_t ant_delay_ns);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_SYNC_H */
