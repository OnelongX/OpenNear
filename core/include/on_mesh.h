#ifndef OPENNEAR_MESH_H
#define OPENNEAR_MESH_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v6.0 Hybrid Multi-Hop Mesh (3~6 Hops) & 256-Node Epoch-TDMA Engine
 *
 * 1. Hybrid Mesh Overlay (ON_FRAME_MESH = 0xD):
 *    - Extends OpenNear's 1-hop & 2-hop deterministic tree out to 6 hops across
 *      multi-floor buildings, basements, and long corridors.
 *    - Automatic parent selection (minimum hop_to_root, LQI tie-breaker) and
 *      self-healing parent failover on missed beacons.
 *    - Automatic reverse-path route learning (dest_id -> next_hop_id) so Cloud /
 *      MQTT / BLE downlink commands walk the exact tree with zero flood storms.
 *    - 32-entry (src_id, msg_id) LRU deduplication cache preventing loops.
 *
 * 2. 256-Node Multi-Epoch TDMA Multiplexer (Epoch-TDMA):
 *    - Multiplexes up to 32 low-power sleeping IoT sensors per TDMA slot
 *      (8 shared slots x 32 epochs = 256 zero-collision nodes per cluster)
 *      while coexisting with 250 Hz real-time actuators (epoch_period = 1).
 */

#define ON_MESH_MAX_HOPS          6u
#define ON_MESH_HDR_LEN           12u
#define ON_MESH_MAX_PAYLOAD_LEN   92u
#define ON_MESH_MAX_NEIGHBORS     8u
#define ON_MESH_MAX_ROUTES        32u
#define ON_MESH_DEDUP_SIZE        32u

typedef enum {
    ON_MESH_BEACON      = 0x01, /* Periodic parent advertisement (hop_to_root, root_id) */
    ON_MESH_DATA_UP     = 0x02, /* Multi-hop sensor telemetry uplink toward Root G */
    ON_MESH_CMD_DOWN    = 0x03, /* Tree-routed downlink command from G / Cloud / MQTT / BLE */
    ON_MESH_TOPO_REPORT = 0x04  /* Child topology & parent report to Root G cluster_tbl */
} on_mesh_type_t;

#define ON_MESH_FLAG_FLOOD_FALLBACK  0x01u
#define ON_MESH_FLAG_ACK_REQ         0x02u

/* Wire packet for ON_FRAME_MESH (12B header + 0..92B payload) */
typedef struct {
    uint8_t  mesh_type;       /* on_mesh_type_t */
    uint8_t  ttl;             /* Remaining hops (1..6) */
    uint8_t  hop_count;       /* Hops traversed so far */
    uint8_t  flags;
    uint16_t src_id;          /* Original source Short-ID */
    uint16_t dst_id;          /* Final destination Short-ID (or 0xFFF broadcast) */
    uint16_t next_hop_id;     /* Immediate next-hop relay Short-ID (or 0xFFF) */
    uint16_t msg_id;          /* Per-source sequence for loop deduplication */
    uint8_t  payload_len;
    uint8_t  payload[ON_MESH_MAX_PAYLOAD_LEN];
} on_mesh_pkt_t;

typedef struct {
    bool     active;
    uint16_t neighbor_id;
    uint8_t  hop_to_root;     /* 0 = Root G, 1..5 = relay depth */
    uint8_t  lqi;             /* Link quality indicator (0..255, higher is better) */
    uint32_t last_seen_sf;
} on_mesh_neighbor_t;

typedef struct {
    bool     active;
    uint16_t dest_id;         /* Downstream node Short-ID */
    uint16_t next_hop_id;     /* Direct child relay toward dest_id */
    uint8_t  hop_dist;        /* Distance from this node to dest_id */
    uint32_t last_seen_sf;
} on_mesh_route_t;

typedef struct {
    uint16_t src_id;
    uint16_t msg_id;
    bool     used;
} on_mesh_dedup_t;

typedef struct {
    uint16_t my_id;
    uint16_t root_id;
    uint16_t parent_id;
    uint8_t  my_hop_depth;    /* 0 = Root G, 1..6 = attached depth, 0xFF = detached */
    uint16_t tx_msg_seq;
    uint32_t parent_timeout_sf;

    on_mesh_neighbor_t neighbors[ON_MESH_MAX_NEIGHBORS];
    on_mesh_route_t    routes[ON_MESH_MAX_ROUTES];
    on_mesh_dedup_t    dedup[ON_MESH_DEDUP_SIZE];
    uint8_t            dedup_cursor;

    /* Diagnostics */
    uint32_t           relayed_up_total;
    uint32_t           relayed_down_total;
    uint32_t           dedup_drops;
    uint32_t           ttl_drops;
    uint32_t           parent_switches;
} on_mesh_ctx_t;

/* Action returned by on_mesh_process_rx */
typedef enum {
    ON_MESH_ACT_DROP      = 0, /* Duplicate, expired TTL, or not for us */
    ON_MESH_ACT_DELIVER   = 1, /* Deliver payload to local application / Root G */
    ON_MESH_ACT_FORWARD   = 2, /* Forward out_fwd over radio to next_hop_id */
    ON_MESH_ACT_DELIV_FWD = 3  /* Deliver locally AND forward (broadcast) */
} on_mesh_action_t;

/* Wire codec */
on_err_t on_mesh_pkt_pack(const on_mesh_pkt_t *p, uint8_t *out, size_t cap, size_t *len);
on_err_t on_mesh_pkt_unpack(const uint8_t *in, size_t len, on_mesh_pkt_t *p);

/* Hybrid Mesh Router API */
void             on_mesh_init(on_mesh_ctx_t *m, uint16_t my_id, uint8_t initial_hop_depth,
                              uint16_t parent_id);
void             on_mesh_build_beacon(on_mesh_ctx_t *m, on_mesh_pkt_t *out);
void             on_mesh_on_beacon(on_mesh_ctx_t *m, uint16_t sender_id, uint8_t sender_hop,
                                   uint8_t lqi, uint32_t current_sf);
bool             on_mesh_check_parent_health(on_mesh_ctx_t *m, uint32_t current_sf);
on_err_t         on_mesh_build_uplink(on_mesh_ctx_t *m, on_mesh_type_t type,
                                      const uint8_t *payload, uint8_t len,
                                      on_mesh_pkt_t *out);
on_err_t         on_mesh_build_downlink(on_mesh_ctx_t *m, uint16_t target_id,
                                        const uint8_t *payload, uint8_t len,
                                        on_mesh_pkt_t *out);
on_mesh_action_t on_mesh_process_rx(on_mesh_ctx_t *m, uint16_t prev_hop_id,
                                    const on_mesh_pkt_t *in, uint32_t current_sf,
                                    on_mesh_pkt_t *out_fwd);
uint16_t         on_mesh_lookup_next_hop(const on_mesh_ctx_t *m, uint16_t dest_id);

/* ========================================================================== */
/* 256-Node Multi-Epoch TDMA Slot Scheduler (Coexists with 250 Hz Actuators)  */
/* ========================================================================== */

#define ON_EPOCH_MAX_SLOTS        8u   /* 8 shared TDMA slots dedicated to Epoch-TDMA */
#define ON_EPOCH_MAX_PERIOD       32u  /* Up to 32 epochs per slot => 256 nodes! */
#define ON_EPOCH_MAX_DEVICES      (ON_EPOCH_MAX_SLOTS * ON_EPOCH_MAX_PERIOD) /* 256 */

typedef struct {
    bool     active;
    uint16_t short_id;
    uint8_t  slot_idx;        /* 0..7 */
    uint8_t  epoch_phase;     /* 0..(epoch_period - 1) */
    uint8_t  epoch_period;    /* Power of 2: 1, 2, 4, 8, 16, or 32 */
    uint32_t last_seen_sf;
} on_epoch_grant_t;

typedef struct {
    /* 32-bit epoch occupancy mask per slot (bit e=1 => epoch e is occupied in slot s) */
    uint32_t         slot_epoch_mask[ON_EPOCH_MAX_SLOTS];
    on_epoch_grant_t grants[ON_EPOCH_MAX_DEVICES];
    uint16_t         active_count;
} on_epoch_sched_t;

void     on_epoch_sched_init(on_epoch_sched_t *s);
on_err_t on_epoch_alloc(on_epoch_sched_t *s, uint16_t short_id, uint8_t requested_period,
                        uint32_t current_sf, on_epoch_grant_t *out_grant);
bool     on_epoch_release(on_epoch_sched_t *s, uint16_t short_id);
bool     on_epoch_is_my_sf(uint32_t current_sf, uint8_t epoch_phase, uint8_t epoch_period);
uint32_t on_epoch_next_tx_sf(uint32_t current_sf, uint8_t epoch_phase, uint8_t epoch_period);
double   on_epoch_duty_cycle_pct(uint8_t epoch_period);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_MESH_H */
