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
#define ON_LOC_CSI_BINS            16u
#define ON_LOC_MAX_GEOFENCES       8u
#define ON_LOC_MAX_RULES           8u
#define ON_LOC_RULE_COOLDOWN_MS    1500u

/* v7.2: IEEE 802.11bf Wi-Fi CSI device-free room presence state */
typedef enum {
    ON_CSI_STATE_EMPTY  = 0, /* Quiet / unoccupied (motion score < 15) */
    ON_CSI_STATE_STATIC = 1, /* Micro-motion / breathing / seated (15..44) */
    ON_CSI_STATE_MOVING = 2  /* Active walking / intrusion (45..100) */
} on_csi_state_t;

/* v7.2: Offline Edge Spatial & CSI Automation Rule Trigger Types */
typedef enum {
    ON_RULE_TRIG_NONE       = 0,
    ON_RULE_TRIG_ENTER_ROOM = 1, /* Any/target enters room <zone> */
    ON_RULE_TRIG_LEAVE_ROOM = 2, /* All targets leave room <zone> */
    ON_RULE_TRIG_IN_FENCE   = 3, /* Any/target enters 2D circular geofence <zone> */
    ON_RULE_TRIG_OUT_FENCE  = 4, /* All targets exit 2D circular geofence <zone> */
    ON_RULE_TRIG_CSI_MOTION = 5, /* Room <zone> Wi-Fi CSI score >= threshold (device-free!) */
    ON_RULE_TRIG_CSI_QUIET  = 6  /* Room <zone> Wi-Fi CSI score < threshold */
} on_loc_rule_trig_t;

/* v7.2: 2D Circular Geofence definition */
typedef struct {
    bool     active;
    char     name[ON_LOC_ROOM_MAX_LEN];
    int32_t  x_cm;
    int32_t  y_cm;
    uint16_t radius_cm;
    bool     occupied;
} on_loc_geofence_t;

/* v7.2: Offline Edge Automation Rule (persisted in NVS, runs with zero cloud/HA dependency) */
typedef struct {
    bool     active;
    uint8_t  id;                           /* 1..ON_LOC_MAX_RULES */
    uint8_t  trig_type;                    /* on_loc_rule_trig_t */
    char     zone[ON_LOC_ROOM_MAX_LEN];    /* Room name or Geofence name */
    uint8_t  csi_threshold;                /* 1..100 for CSI triggers */
    char     cmd_tag[20];                  /* Target Tag group (e.g. "书房") or "*" for local/all */
    char     cmd_payload[28];              /* e.g. "RELAY:1", "MODE:STROBE", "MODE:PULSE" */
    bool     last_cond_met;                /* Edge-trigger latch */
    uint32_t fire_count;
    uint32_t last_fired_ms;
} on_loc_rule_t;

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
    /* v7.2: IEEE 802.11bf Wi-Fi CSI passive human presence & 16-bin OFDM subcarrier profile */
    uint8_t  csi_motion_score; /* 0..100 smoothed spatial perturbation score */
    uint8_t  csi_presence;     /* on_csi_state_t (0=EMPTY, 1=STATIC, 2=MOVING) */
    uint8_t  csi_bins[ON_LOC_CSI_BINS];
    uint32_t csi_last_ms;
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
    on_loc_anchor_t   anchors[ON_LOC_MAX_ANCHORS];
    on_loc_target_t   targets[ON_LOC_MAX_TARGETS];
    on_loc_geofence_t fences[ON_LOC_MAX_GEOFENCES];
    on_loc_rule_t     rules[ON_LOC_MAX_RULES];
    uint8_t           last_fired_rule_id;
    char              last_fired_summary[64];
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

/* v7.2: Extract 0..100 motion score & 16-bin OFDM subcarrier spectrum from raw ESP32 CSI I/Q buffer */
uint8_t  on_csi_extract_iq_score(const int8_t *iq_buf, uint16_t iq_len, bool skip_first_word,
                                 float *inout_baseline, float *inout_var_ema,
                                 uint8_t out_bins16[ON_LOC_CSI_BINS]);

/* v7.2: Ingest a Wi-Fi CSI motion score & 16-bin subcarrier profile for a fixed room Anchor */
on_err_t on_loc_observe_csi(on_loc_ctx_t *ctx, const uint8_t anchor_mac48[6],
                            uint16_t anchor_short_id, uint8_t motion_score,
                            const uint8_t bins16[ON_LOC_CSI_BINS], uint32_t now_ms);

/* v7.2: Configure or remove (radius_cm == 0) a 2D circular Geofence */
on_err_t on_loc_set_geofence(on_loc_ctx_t *ctx, const char *name,
                             int32_t x_cm, int32_t y_cm, uint16_t radius_cm);

/* v7.2: Check whether (x_cm, y_cm) lies inside a named 2D circular Geofence */
bool     on_loc_check_geofence(const on_loc_ctx_t *ctx, int32_t x_cm, int32_t y_cm,
                               const char *fence_name);

/* v7.2: Configure or remove (trig_type == 0) an offline Edge Spatial / CSI Automation Rule */
on_err_t on_loc_set_rule(on_loc_ctx_t *ctx, uint8_t rule_id, uint8_t trig_type,
                         const char *zone, uint8_t csi_threshold,
                         const char *cmd_tag, const char *cmd_payload);

/* v7.2: Evaluate all active Geofences & Offline Rules; invokes on_fire callback on edge transitions */
uint32_t on_loc_evaluate_rules(on_loc_ctx_t *ctx, uint32_t now_ms,
                               void (*on_fire)(void *user_ctx, const on_loc_rule_t *rule),
                               void *user_ctx);

/* v7.2: Load out-of-the-box 4-Room + 2-Geofence + 4-Rule + Wi-Fi CSI Showcase Demo & advance virtual walk */
void     on_loc_load_demo_preset(on_loc_ctx_t *ctx, const uint8_t gw_mac48[6], uint32_t now_ms);
void     on_loc_step_demo(on_loc_ctx_t *ctx, uint32_t step_idx, uint32_t now_ms);

/* Format all active anchors, targets, geofences, rules & CSI into compact JSON for HTTP /api/locate */
size_t   on_loc_format_json(on_loc_ctx_t *ctx, uint32_t now_ms,
                            char *out_buf, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_LOCATE_H */
