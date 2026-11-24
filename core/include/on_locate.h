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
#define ON_LOC_LABEL_MAX_LEN       20u
#define ON_LOC_OBS_STALE_MS        20000u
#define ON_LOC_HYSTERESIS_DB       4

/* v7.1: Target radio / beacon standard origin */
typedef enum {
    ON_LOC_SRC_OPENNEAR  = 0, /* Native OpenNear 802.11 Action / Sub-G node */
    ON_LOC_SRC_IBEACON   = 1, /* Apple iBeacon (AD 0xFF, Company 0x004C, 0x02 0x15) */
    ON_LOC_SRC_EDDYSTONE = 2, /* Google Eddystone-UID / URL (AD 0x16, UUID 0xFEAA) */
    ON_LOC_SRC_BLE_RAW   = 3  /* Generic BLE advertisement / Eddystone-TLM */
} on_loc_source_t;

/* v7.1: Decoded standard BLE beacon (iBeacon or Eddystone) */
typedef struct {
    bool            valid;
    on_loc_source_t source;
    uint8_t         eddy_frame_type;  /* 0x00=UID, 0x10=URL, 0x20=TLM (when Eddystone) */
    int8_t          tx_power_1m_dbm;  /* Calibrated RSSI @ 1 m (Eddystone 0m converted via -41 dB) */
    uint8_t         uuid16[16];       /* iBeacon Proximity UUID */
    uint16_t        major;            /* iBeacon Major */
    uint16_t        minor;            /* iBeacon Minor */
    uint8_t         eddy_ns[10];      /* Eddystone-UID 10-byte Namespace */
    uint8_t         eddy_inst[6];     /* Eddystone-UID 6-byte Instance */
    uint16_t        tlm_batt_mv;      /* Eddystone-TLM Battery mV */
    int16_t         tlm_temp_c_x100;  /* Eddystone-TLM Temperature in 0.01 C */
    char            label[ON_LOC_LABEL_MAX_LEN]; /* Human-readable auto-label */
} on_ble_beacon_t;

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
    uint8_t      source;          /* on_loc_source_t */
    int8_t       tx_power_1m_dbm; /* 0 = use anchor default; <0 = beacon calibrated txPower@1m */
    char         label[ON_LOC_LABEL_MAX_LEN];
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
    uint8_t  source;
    int8_t   tx_power_1m_dbm;
    char     label[ON_LOC_LABEL_MAX_LEN];
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

/* Ingest a real-time RSSI observation (dBm, e.g. -62) of an OpenNear node heard by anchor */
on_err_t on_loc_observe_rssi(on_loc_ctx_t *ctx, const uint8_t anchor_mac48[6],
                             const uint8_t target_mac48[6], uint16_t target_short_id,
                             int8_t rssi_dbm, uint32_t now_ms);

/* v7.1: Ingest a standard BLE iBeacon / Eddystone observation with calibrated txPower@1m */
on_err_t on_loc_observe_beacon(on_loc_ctx_t *ctx, const uint8_t anchor_mac48[6],
                               const uint8_t target_mac48[6], on_loc_source_t source,
                               int8_t tx_power_1m_dbm, int8_t rssi_dbm,
                               const char *label, uint32_t now_ms);

/* v7.1: Pure C11 zero-heap parser for standard BLE Advertising Data (Apple iBeacon & Google Eddystone) */
bool     on_ble_beacon_parse(const uint8_t *ad_data, size_t ad_len, on_ble_beacon_t *out);

/* v7.1: Derive stable 6-byte identity MAC48 from iBeacon UUID/Major/Minor or Eddystone-UID
 *       (immune to iOS/Android BLE random MAC rotation across all cluster anchors) */
void     on_ble_beacon_derive_mac(const on_ble_beacon_t *b, const uint8_t fallback_ble_mac[6],
                                  uint8_t out_mac[6]);

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
