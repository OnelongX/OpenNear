#ifndef OPENNEAR_NODE_H
#define OPENNEAR_NODE_H

#include "on_types.h"
#include "on_frame.h"
#include "on_sync.h"
#include "on_superframe.h"
#include "on_sec.h"
#include "on_afh.h"
#include "on_ota.h"
#include "on_gateway.h"
#include "on_mesh.h"
#include "../../hal/on_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Node registry entry maintained by Primary G and Sub-G proxy */
typedef struct {
    bool     active;
    uint8_t  mac48[6];
    uint16_t short_id;
    uint8_t  slot_idx;
    bool     is_sub_slot;
    bool     mac_reported_to_g; /* v6.5: Sub-G has propagated 2-hop child MAC to Primary G */
    uint8_t  capabilities;
    uint32_t last_seen_sf;
    int16_t  last_skew_ns;
    uint32_t rx_packets;
} on_peer_entry_t;

/* v5.1/v6.5 Cluster-wide multi-hop node record (0-hop G + 1-hop T/Sub-G + 2-hop edge + MAC48 + live data) */
#define ON_MAX_CLUSTER_NODES  24u
#define ON_TOPO_PAGE_NODES    2u  /* 6B page hdr + 2 * 18B = 42B (compact 3B IDs + 6B MAC48 + 4B data, <220us!) */

typedef struct {
    bool     active;
    uint8_t  mac48[6];        /* v6.5: Permanent hardware MAC48 (ESP32 Wi-Fi STA / nRF52 FICR) */
    uint16_t short_id;
    uint16_t parent_id;       /* 0x000 for Primary G, G_id for 1-hop, Sub-G_id for 2-hop */
    uint8_t  role;            /* on_role_t */
    uint8_t  hop_depth;       /* 0 = Primary G, 1 = 1-hop, 2 = 2-hop wall-blocked edge */
    uint8_t  slot_idx;
    uint8_t  capabilities;
    bool     is_backup;
    uint32_t last_seen_sf;
    int16_t  last_skew_ns;
    uint32_t rx_packets;
    uint16_t last_seq;
    uint8_t  last_data[8];
    uint8_t  last_data_len;
    uint8_t  ota_state;       /* on_ota_state_t */
    uint16_t ota_rx_chunks;
} on_cluster_node_t;

typedef struct {
    uint32_t          sf_seq;
    uint16_t          g_id;
    uint16_t          backup_id;
    uint16_t          sub_g_id;
    uint16_t          sub_g_id_2;
    uint16_t          tdma_bitmap;
    uint16_t          channel_map;
    uint8_t           active_count;
    bool              is_remote_cached; /* True when fetched over the air by a non-G node */
    on_cluster_node_t nodes[ON_MAX_CLUSTER_NODES];
} on_cluster_snapshot_t;

/* Optional direct ACT clock: lower envelope of recent timestamp blocks. */
#define ON_ACT_CLOCK_BLOCKS 8u
typedef struct {
    bool valid;
    int64_t net_ns;
    int64_t offset_ns;
} on_act_clock_sample_t;

typedef struct {
    bool valid;
    int64_t last_net_ns;
    int32_t rate_ppm;   /* Local elapsed time per network elapsed time, minus one */
    on_act_clock_sample_t blocks[ON_ACT_CLOCK_BLOCKS];
} on_act_clock_t;

/* Complete OpenNear Node Context */
typedef struct {
    on_hal_ops_t   hal;
    on_sf_cfg_t    sf_cfg;
    on_sync_ctx_t  sync;
    on_sec_ctx_t   sec;                /* v2.0: AES-128-CCM AEAD + monotonic anti-replay */
    on_afh_ctx_t   afh;                /* v2.0: TSCH Adaptive Frequency Hopping + blacklist */

    /* Node identity & role */
    uint8_t        mac48[6];
    uint8_t        capabilities;       /* bit0: backup_capable, bit1: sub_g_capable */
    on_role_t      role;
    uint16_t       short_id;           /* 12-bit Short-ID (0x001..0xFFE) */
    uint8_t        my_slot;            /* Assigned D-TDMA or Sub-TDMA slot */
    bool           is_sub_node;        /* True if attached behind a Sub-G relay (2-hop) */
    bool           primary_slot_leased; /* Direct assignment obtained through JOIN_ACK */
    uint32_t       primary_join_sf;    /* Ignore a map from before or during the JOIN */
    bool           heard_primary_sync; /* True once primary G SYNC_MAP has been heard */
    uint16_t       parent_id;          /* Primary G or Sub-G Short-ID */

    /* Superframe runtime state */
    uint32_t       current_sf_seq;
    int64_t        current_sf_start_net_ns;
    uint16_t       tdma_bitmap;
    uint16_t       sub_tdma_bitmap;
    uint16_t       backup_id;
    uint16_t       sub_g_id;
    uint16_t       sub_g_id_2;         /* v3.1: Zone-1 secondary FDMA Sub-G relay proxy */
    uint8_t        sub_g_zone;         /* 0 = Zone-0, 1 = Zone-1 (+8 orthogonal channel offset) */
    bool           in_subg_z1_ch;      /* True while Zone-1 Sub-G is hopped onto orthogonal channel */
    int16_t        last_sync_tx_err_ns;/* v3.1: Measured SYNC TX departure error for PTP follow-up */
    uint8_t        act_cmd;
    bool           act_lookahead_enabled; /* Opt-in for direct, binary GPIO demo only */
    struct {
        bool      has_beacon;
        uint32_t  beacon_sf;
        uint32_t  session_salt;
        uint32_t  level_sf;         /* Oldest beacon allowed to replace current level */
        bool      pending;
        uint32_t  target_sf;
        int64_t   deadline_local_ns;
        uint8_t   next_level;
    } act_lookahead;
    bool           act_clock_enabled; /* Independent phase estimate for pending GPIO plans */
    uint32_t       act_clock_bias_us; /* Nominal TX to earliest local RX observation floor */
    on_act_clock_t act_clock;
    bool           last_act_was_scheduled;
    int64_t        last_act_deadline_local_ns;
    bool           sync_rx_this_sf;
    uint32_t       missed_sync_count;
    uint32_t       peer_lease_timeout_sf; /* Silent superframes before reclaiming a peer */
    uint8_t        backup_miss_threshold; /* Consecutive missed beacons before takeover */
    int64_t        backup_min_sync_silence_ns; /* Optional real local time without a valid beacon */
    int64_t        last_sync_accept_local_ns;  /* Local processing time, not the network clock */
    bool           act_fired_this_sf;
    bool           tdma_tx_done_this_sf;
    bool           sub_sync_tx_done_this_sf;
    bool           cra_tx_done_this_sf;
    bool           ch_pretuned_this_sf;

    /* v2.0 Piggyback Group-ACK & Slot-Local Fast HARQ */
    uint16_t       rx_slot_mask_this_sf;
    uint16_t       sub_rx_slot_mask_this_sf;
    uint16_t       last_ack_bitmap;
    uint16_t       last_sub_ack_bitmap;
    bool           awaiting_ack;
    bool           harq_retry_pending;
    on_data_payload_t last_tx_data;
    uint32_t       harq_retx_total;
    uint32_t       harq_recovered_total;
    uint16_t       pending_channel_map;
    bool           has_pending_ch_map;
    uint8_t        afh_countdown;      /* Multi-SF countdown (BLE Instant style) for atomic map switch */
    bool           pending_key_rotation; /* Deferred to next superframe 0 us boundary (zero dropped frames) */

    /* RTT calibration state */
    bool           rtt_in_flight;
    int64_t        rtt_t1_local_ns;
    uint16_t       rtt_token;
    uint32_t       rtt_next_sf;

    /* Pending emergency alert (sent via C-RA contention micro-slot) */
    bool           pending_alert;
    uint8_t        alert_code;

    /* Sub-G 4-slot FIFO relay ring buffer (prevents multi-edge packet overwrite starvation) */
    on_data_payload_t relay_q[ON_DEFAULT_SUBG_MAX_SLOTS];
    uint8_t           relay_head;
    uint8_t           relay_tail;
    uint8_t           relay_count;

    /* Peer table (used by G, mirrored by Backup_ID, and used by Sub-G for child nodes) */
    on_peer_entry_t peers[ON_MAX_NODES_TABLE];
    uint16_t        next_alloc_id;
    uint16_t        data_tx_seq;

    /* Power / duty cycle & protocol metrics */
    uint64_t       radio_active_ns;
    uint64_t       total_time_ns;
    int64_t        last_step_local_ns;
    uint32_t       sync_rx_total;
    uint32_t       hdr_crc_drops;
    uint32_t       pay_crc_drops;
    uint32_t       tdma_tx_total;
    uint32_t       tdma_rx_total;
    uint32_t       sub_sync_tx_total;
    uint32_t       failover_count;
    uint32_t       rejoin_count;       /* Discarded assignments after session change or slot expiry */
    uint32_t       lease_reclaim_count;
    uint32_t       subg_failover_count;
    uint32_t       act_fire_total;
    int64_t        last_act_local_ns;

    /* v3.0 AFH unjoined discovery scanner state */
    int64_t        scan_last_hop_local_ns;
    uint8_t        scan_ch_idx;

    /* v5.0 Optional Node OTA & Public Wi-Fi/BLE Gateway Bridge */
    on_ota_ctx_t  *ota;
    on_gw_ctx_t   *gw;
    bool           service_tx_done_this_sf;
    bool           sub_dl_tx_done_this_sf;
    bool           service_dl_reserved_this_sf; /* Set from SYNC_MAP flags bit 3 */
    uint32_t       ota_tx_total;
    uint32_t       ota_rx_total;
    uint32_t       gw_tx_total;
    uint32_t       gw_rx_total;

    /* Pending terminal-side uplink GW message */
    bool               gw_ul_pending;
    on_gw_bridge_msg_t gw_ul_msg;

    /* Sub-G transparent 2-hop relay slots for OTA & Gateway bridge frames */
    bool           sub_dl_pending;
    uint8_t        sub_dl_type;
    uint16_t       sub_dl_dst_id;
    uint8_t        sub_dl_len;
    uint8_t        sub_dl_payload[ON_MAX_PAYLOAD_LEN];

    bool           sub_ul_pending;
    uint8_t        sub_ul_type;
    uint8_t        sub_ul_len;
    uint8_t        sub_ul_payload[ON_MAX_PAYLOAD_LEN];
    uint8_t        sub_dl_listen_sfs;  /* Sub-G temporary C-RA RX window after 2-hop uplink */

    /* v5.1 Custom application telemetry payload (sent in node's TDMA / Sub-TDMA slot) */
    uint8_t        app_tx_data[8];
    uint8_t        app_tx_data_len;

    /* v5.1 Full cluster-wide topology & live node data table (maintained by G, cached on query) */
    on_cluster_node_t cluster_tbl[ON_MAX_CLUSTER_NODES];
    uint16_t          cluster_g_id;
    bool              cluster_snapshot_valid;
    bool              topo_query_in_flight;
    uint16_t          topo_rx_pages_mask;
    uint8_t           topo_rx_total_pages;
    on_gw_ring_t      topo_resp_q;        /* Coordinator G paginated TOPO_RESP downlink queue */

    /* v6.0 Leaderless Cold-Boot Auto-Election, Epoch-TDMA & 3~6-Hop Hybrid Mesh */
    bool              leaderless_boot_enabled;
    int64_t           leaderless_base_timeout_ns;
    int64_t           boot_listen_start_ns;
    uint8_t           epoch_phase;        /* 0..(epoch_period - 1) */
    uint8_t           epoch_period;       /* 0 or 1 = every SF; 2/4/8/16/32 = sleeping IoT */
    on_mesh_ctx_t    *mesh;
    bool              mesh_tx_pending;
    on_mesh_pkt_t     mesh_tx_pkt;
    uint32_t          mesh_rx_total;
    uint32_t          mesh_tx_total;
    bool              self_mac_reported_to_g; /* v6.8: false after parent failover -> 1-shot MAC_REPORT */

    /* v7.0: Real-time Anchor RSSI overhearing & zero-collision reporting to Primary G */
    bool              rssi_report_enabled;
    uint32_t          last_rssi_rep_sf;
    uint8_t           rssi_obs_count;
    struct {
        uint16_t short_id;
        int8_t   rssi_dbm;
    } rssi_obs[6];
    void (*on_rssi_observed)(void *ctx, uint16_t anchor_id, uint16_t target_id, int8_t rssi_dbm);
    void             *rssi_obs_ctx;

    /* v7.1: Passive BLE iBeacon/Eddystone multi-anchor observation queue & callback */
    uint32_t          last_ble_rep_sf;
    uint8_t           ble_obs_count;
    struct {
        uint8_t tag_mac48[6];
        uint8_t source;
        int8_t  tx_power_1m_dbm;
        int8_t  rssi_dbm;
    } ble_obs[4];
    void (*on_ble_beacon_observed)(void *ctx, uint16_t anchor_id, const uint8_t tag_mac48[6],
                                   uint8_t source, int8_t tx_power_1m_dbm, int8_t rssi_dbm);
    void             *ble_obs_ctx;

    /* v7.2: IEEE 802.11bf Wi-Fi CSI passive human presence observation queue & callback */
    uint32_t          last_csi_rep_sf;
    bool              csi_obs_pending;
    uint8_t           csi_obs_score;
    uint8_t           csi_obs_bins[16];
    void (*on_csi_observed)(void *ctx, uint16_t anchor_id, uint8_t motion_score,
                            const uint8_t bins16[16]);
    void             *csi_obs_ctx;

    /* v7.3: 700m+ Long-Range Mode (0 = 4ms/54M Fast, 1 = 20ms/1M DSSS 500m, 2 = 20ms/250K LR 700m+) */
    uint8_t           lr_mode;
    uint8_t           pending_lr_mode;
    bool              has_pending_lr_mode;
} on_node_t;

/* Initialize node as Primary G coordinator or Unjoined Terminal */
void     on_node_init(on_node_t *node, const on_hal_ops_t *hal,
                      on_role_t initial_role, uint8_t capabilities);

/* Pre-register a static peer on G (optional, for fast bring-up or pre-assigned topology) */
on_err_t on_node_register_static_peer(on_node_t *node, const uint8_t mac48[6],
                                      uint16_t short_id, uint8_t slot_idx,
                                      uint8_t capabilities, bool is_sub_slot);

/* Configure Primary G's designated Backup_ID and Sub-G proxy (single or dual-zone FDMA) */
void     on_node_set_backup_and_subg(on_node_t *node, uint16_t backup_id, uint16_t sub_g_id);
void     on_node_set_multi_subg(on_node_t *node, uint16_t sub_g_zone0_id, uint16_t sub_g_zone1_id);

/* v3.1: Report actual hardware TX departure error (ns) of the last SYNC beacon for PTP Follow-Up */
void     on_node_record_sync_tx_err_ns(on_node_t *node, int16_t tx_err_ns);

/* v2.0: Enable AES-128-CCM AEAD encryption & monotonic anti-replay with 16-byte master key */
void     on_node_enable_security(on_node_t *node, const uint8_t key16[ON_SEC_KEY_LEN]);

/* v3.0: Configure Primary G boot session salt and trigger live online key epoch rotation */
void     on_node_set_session_salt(on_node_t *node, uint32_t session_salt);
void     on_node_rotate_key_epoch(on_node_t *node);

/* v2.0: Enable TSCH Adaptive Frequency Hopping & dynamic Wi-Fi interference blacklisting */
void     on_node_enable_afh(on_node_t *node, bool enable);

/* Queue a high-priority emergency alert to transmit in the next C-RA micro-slot */
void     on_node_trigger_alert(on_node_t *node, uint8_t alert_code);

/* Feed a raw received frame with exact hardware RX capture timestamp (local_ns) */
void     on_node_on_rx_raw(on_node_t *node, const uint8_t *frame, size_t len, int64_t rx_local_ns);

/* Optional second RX timestamp in the same local-clock domain, only for the
 * independent ACT clock. The normal protocol still uses rx_local_ns. */
void     on_node_on_rx_raw_timed(on_node_t *node, const uint8_t *frame, size_t len,
                                int64_t rx_local_ns, int64_t act_rx_local_ns);

/* v7.0: RSSI-aware RX entry points (rssi_dbm < 0 enables real-time multi-anchor indoor positioning) */
void     on_node_on_rx_raw_rssi(on_node_t *node, const uint8_t *frame, size_t len,
                                int64_t rx_local_ns, int8_t rssi_dbm);
void     on_node_on_rx_raw_timed_rssi(on_node_t *node, const uint8_t *frame, size_t len,
                                      int64_t rx_local_ns, int64_t act_rx_local_ns, int8_t rssi_dbm);

/* Advance node state machine to current local_ns (called by timer interrupt or simulator) */
void     on_node_step(on_node_t *node, int64_t now_local_ns);

/* Query next local hardware timestamp (ns) when node needs attention (for low-power deep sleep) */
int64_t  on_node_next_wakeup_local_ns(const on_node_t *node, int64_t now_local_ns);

/* Query whether the node's RX radio is powered ON at now_local_ns (for low-power TDMA gating) */
bool     on_node_is_rx_open(const on_node_t *node, int64_t now_local_ns);

/* Compute radio duty cycle percentage (0.0 .. 100.0) */
double   on_node_duty_cycle_pct(const on_node_t *node);

/* v5.0: Attach Node OTA engine & Public Wi-Fi/BLE Gateway Bridge */
void     on_node_attach_ota(on_node_t *node, on_ota_ctx_t *ota);
void     on_node_attach_gateway(on_node_t *node, on_gw_ctx_t *gw);

/* v5.0: Coordinator G OTA session control */
on_err_t on_node_ota_start_mem(on_node_t *node, uint16_t ota_id, uint32_t fw_version,
                               uint16_t target_id, uint8_t hw_compat, uint8_t flags,
                               const uint8_t *image_buf, uint32_t image_size,
                               uint16_t chunk_size);
on_err_t on_node_ota_commit(on_node_t *node, uint32_t lead_sfs, uint8_t flags);

/* v5.0: Bidirectional Public Wi-Fi / BLE Gateway tunnel helpers */
on_err_t on_node_send_gw_uplink(on_node_t *node, on_gw_msg_type_t type,
                                const uint8_t *payload, uint8_t len);
on_err_t on_node_send_gw_downlink(on_node_t *node, uint16_t target_id,
                                  on_gw_msg_type_t type, const uint8_t *payload, uint8_t len);

/* v5.1: Application sensor payload & Cluster-Wide Topology / Live Data Introspection */
void     on_node_set_tx_payload(on_node_t *node, const uint8_t *data, uint8_t len);
on_err_t on_node_request_cluster_snapshot(on_node_t *node);
size_t   on_node_get_cluster_snapshot(const on_node_t *node, on_cluster_snapshot_t *out);
size_t   on_node_format_cluster_json(const on_node_t *node, bool include_data,
                                     char *out_buf, size_t out_cap);
/* v6.8: Resolve permanent hardware MAC48 -> current dynamic 12-bit Short-ID (0x000 if unknown) */
uint16_t on_node_find_short_id_by_mac(const on_node_t *node, const uint8_t mac48[6]);

/* v7.0: Reverse lookup Short-ID -> permanent MAC48 & Anchor RSSI reporting hooks */
bool     on_node_find_mac_by_short_id(const on_node_t *node, uint16_t short_id, uint8_t mac_out[6]);
void     on_node_enable_rssi_anchor_report(on_node_t *node, bool enable);
void     on_node_set_rssi_observer(on_node_t *node,
                                   void (*cb)(void *ctx, uint16_t anchor_id,
                                              uint16_t target_id, int8_t rssi_dbm),
                                   void *user_ctx);

/* v7.1: Standard BLE iBeacon/Eddystone observation queue (Anchor->G relay) & Primary G observer */
void     on_node_report_ble_beacon(on_node_t *node, const uint8_t tag_mac48[6],
                                   uint8_t source, int8_t tx_power_1m_dbm, int8_t rssi_dbm);
void     on_node_set_ble_beacon_observer(on_node_t *node,
                                         void (*cb)(void *ctx, uint16_t anchor_id,
                                                    const uint8_t tag_mac48[6], uint8_t source,
                                                    int8_t tx_power_1m_dbm, int8_t rssi_dbm),
                                         void *user_ctx);

/* v7.2: IEEE 802.11bf Wi-Fi CSI passive presence report (Anchor->G relay) & Primary G observer */
void     on_node_report_csi(on_node_t *node, uint8_t motion_score, const uint8_t bins16[16]);
void     on_node_set_csi_observer(on_node_t *node,
                                  void (*cb)(void *ctx, uint16_t anchor_id,
                                             uint8_t motion_score, const uint8_t bins16[16]),
                                  void *user_ctx);

/* v6.0: Leaderless Auto-Boot, 256-Node Epoch-TDMA & 3~6-Hop Hybrid Mesh */
void     on_node_enable_leaderless_boot(on_node_t *node, uint32_t base_timeout_ms);
void     on_node_set_epoch_schedule(on_node_t *node, uint8_t epoch_phase, uint8_t epoch_period);
void     on_node_attach_mesh(on_node_t *node, on_mesh_ctx_t *mesh);
on_err_t on_node_send_mesh_uplink(on_node_t *node, const uint8_t *payload, uint8_t len);
on_err_t on_node_send_mesh_topo_mac(on_node_t *node);
on_err_t on_node_send_mesh_downlink(on_node_t *node, uint16_t target_id,
                                    const uint8_t *payload, uint8_t len);

/* v7.3: 700m+ Long-Range Mode (0 = 4ms Fast, 1 = 20ms 1M DSSS ~500m, 2 = 20ms 250K LR Spread 700m+) */
void     on_node_set_long_range_mode(on_node_t *node, uint8_t lr_mode);
uint8_t  on_node_get_long_range_mode(const on_node_t *node);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_NODE_H */
