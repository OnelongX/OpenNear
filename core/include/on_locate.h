#ifndef OPENNEAR_LOCATE_H
#define OPENNEAR_LOCATE_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v7.0 Zero-Preconfig Multi-Anchor RSSI Indoor Positioning Engine
 *
 * Pure C11, zero heap allocations, zero external dependencies.
 * Works directly on ESP32-WROOM-32 using hardware pkt->rx_ctrl.rssi overheard by
 * fixed Anchor nodes (Primary G + 1-hop / Sub-G anchors) and relayed zero-collision
 * over OpenNear's private D-TDMA slots.
 *
 * Capabilities:
 *   1. Dynamic Anchor registry (add/update/remove anchors at runtime via Web/Serial/MQTT).
 *   2. Per-(target, anchor) 3-sample median + EMA RSSI denoiser (rejects multipath spikes).
 *   3. Room-level classifier with +4 dB hysteresis anti-ping-pong.
 *   4. 2D (x, y) solver: log-distance path-loss + weighted centroid + 4-step Gauss-Newton
 *      trilateration refinement + 2D EMA kinematic smoothing.
 */

#define ON_LOC_MAX_ANCHORS         12u
#define ON_LOC_MAX_TARGETS         24u
#define ON_LOC_MAX_OBS_PER_TARGET  8u
#define ON_LOC_ROOM_MAX_LEN        20u
#define ON_LOC_OBS_STALE_MS        15000u
#define ON_LOC_HYSTERESIS_DB       4

typedef struct {
    bool     active;
    uint8_t  mac48[6];
    uint16_t short_id;
    char     room[ON_LOC_ROOM_MAX_LEN];
    int32_t  x_cm;             /* Anchor X coordinate in centimeters (e.g. 250 = 2.50 m) */
    int32_t  y_cm;             /* Anchor Y coordinate in centimeters */
    int8_t   tx_power_1m_dbm;  /* Reference RSSI at 1 m (default -45 dBm) */
    float    path_loss_n;      /* Indoor path-loss exponent (default 2.5) */
} on_loc_anchor_t;

typedef struct {
    bool     used;
    uint8_t  anchor_idx;
    int8_t   raw_hist[3];
    uint8_t  hist_cnt;
    uint8_t  hist_pos;
    float    smooth_rssi;
    uint32_t last_ms;
} on_loc_obs_t;

typedef struct {
    bool         active;
    uint8_t      mac48[6];
    uint16_t     short_id;
    on_loc_obs_t obs[ON_LOC_MAX_OBS_PER_TARGET];
    char         cur_room[ON_LOC_ROOM_MAX_LEN];
    int8_t       cur_room_anchor_idx;
    bool         pos_inited;
    float        smooth_x_cm;
    float        smooth_y_cm;
    uint32_t     last_seen_ms;
} on_loc_target_t;

typedef struct {
    bool     valid;
    uint8_t  mac48[6];
    uint16_t short_id;
    char     room[ON_LOC_ROOM_MAX_LEN];
    int32_t  x_cm;
    int32_t  y_cm;
    int8_t   best_rssi_dbm;
    uint8_t  anchor_count;
    uint16_t est_err_cm;
} on_loc_estimate_t;

typedef struct {
    on_loc_anchor_t anchors[ON_LOC_MAX_ANCHORS];
    on_loc_target_t targets[ON_LOC_MAX_TARGETS];
} on_loc_ctx_t;

void     on_loc_init(on_loc_ctx_t *ctx);

/* Register, update, or remove (room == NULL || room[0] == '\0') a fixed positioning Anchor */
on_err_t on_loc_set_anchor(on_loc_ctx_t *ctx, const uint8_t mac48[6], uint16_t short_id,
                           const char *room, int32_t x_cm, int32_t y_cm);

/* Optional per-anchor RF calibration (reference RSSI at 1 m & indoor path-loss exponent n) */
on_err_t on_loc_calibrate_anchor(on_loc_ctx_t *ctx, const uint8_t mac48[6],
                                 int8_t tx_power_1m_dbm, float path_loss_n);

/* Ingest a real-time RSSI observation (dBm, e.g. -62) of target heard by anchor */
on_err_t on_loc_observe_rssi(on_loc_ctx_t *ctx, const uint8_t anchor_mac48[6],
                             const uint8_t target_mac48[6], uint16_t target_short_id,
                             int8_t rssi_dbm, uint32_t now_ms);

/* Solve room + 2D (x_cm, y_cm) estimate for a specific target MAC48 */
bool     on_loc_solve(on_loc_ctx_t *ctx, const uint8_t target_mac48[6],
                      uint32_t now_ms, on_loc_estimate_t *out_est);

/* Format all active anchors and solved target positions into compact JSON for HTTP /api/locate */
size_t   on_loc_format_json(on_loc_ctx_t *ctx, uint32_t now_ms,
                            char *out_buf, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_LOCATE_H */
