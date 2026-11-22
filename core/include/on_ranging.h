#ifndef OPENNEAR_RANGING_H
#define OPENNEAR_RANGING_H

#include "on_types.h"
#include "on_afh.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v4.0 (NearLink 2.0 ISAC) High-Precision Ranging & Anti-Relay Geofence Engine
 *
 * Combines:
 *   1. 16-Channel Multi-Carrier Phase Difference (MCPD) across 2402..2478 MHz (76 MHz span)
 *      via unwrapped least-squares phase-slope regression:
 *        Phi(f) = -4 * pi * f * d / c  =>  d_MCPD = -(c / (4 * pi)) * (dPhi / df)
 *      achieving centimetre/decimetre (+/- 0.05 m) ranging resolution.
 *   2. 4-Timestamp Bidirectional RTT Time-of-Flight (ToF) speed-of-light upper bound:
 *        d_ToF = (c * RTT) / 2
 *   3. Anti-Relay Attack Detector (Digital Car Key / Smart Lock security):
 *      Detects RF repeater / coaxial relay / phase-spoofing MitM attacks whenever
 *      ToF propagation delay diverges from MCPD phase-slope distance beyond physical bounds.
 *   4. 1D Alpha-Beta Kinematic Distance & Approach-Velocity Tracker.
 */

#define ON_RANGING_SPEED_OF_LIGHT_MPS  299792458.0
#define ON_RANGING_RELAY_SLACK_M       1.80   /* Max physical ToF vs MCPD discrepancy before Relay alarm */

typedef struct {
    bool     has_tone[ON_AFH_NUM_CHANNELS];
    double   phase_rad[ON_AFH_NUM_CHANNELS];
    uint8_t  tone_count;

    /* 1D Alpha-Beta Kinematic Filter state */
    bool     filter_inited;
    double   filtered_dist_m;
    double   velocity_mps;
} on_ranging_ctx_t;

typedef struct {
    double   d_mcpd_m;              /* Multi-carrier phase-slope distance (m) */
    double   d_tof_m;               /* 4-timestamp RTT speed-of-light distance (m) */
    double   filtered_dist_m;       /* Alpha-Beta smoothed distance (m) */
    double   velocity_mps;          /* Radial approach/recede speed (m/s) */
    bool     relay_attack_detected; /* True if ToF vs MCPD violates speed-of-light causality */
    bool     geofence_pass;         /* True if inside geofence_max_m AND no relay attack */
} on_ranging_result_t;

void     on_ranging_init(on_ranging_ctx_t *ctx);
void     on_ranging_clear_tones(on_ranging_ctx_t *ctx);

/* Record a two-way carrier phase measurement (radians in [-pi, pi]) on logical channel ch_idx */
void     on_ranging_add_tone(on_ranging_ctx_t *ctx, uint8_t ch_idx, double two_way_phase_rad);

/* Simulate ideal + noisy two-way carrier phase on channel ch_idx at true distance dist_m */
double   on_ranging_synth_tone_phase(uint8_t ch_idx, double true_dist_m, double phase_noise_rad);

/*
 * Solve MCPD + ToF distance, check for Relay Attack, and update Alpha-Beta filter (dt_sec step).
 */
on_err_t on_ranging_solve(on_ranging_ctx_t *ctx,
                          double rtt_ns,
                          double geofence_max_m,
                          double dt_sec,
                          on_ranging_result_t *out_res);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_RANGING_H */
