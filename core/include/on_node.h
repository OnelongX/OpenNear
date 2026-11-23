#ifndef OPENNEAR_NODE_H
#define OPENNEAR_NODE_H

#include "on_types.h"
#include "on_frame.h"
#include "on_sync.h"
#include "on_superframe.h"
#include "on_sec.h"
#include "on_afh.h"
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
    uint8_t  capabilities;
    uint32_t last_seen_sf;
    int16_t  last_skew_ns;
    uint32_t rx_packets;
} on_peer_entry_t;

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

/* Advance node state machine to current local_ns (called by timer interrupt or simulator) */
void     on_node_step(on_node_t *node, int64_t now_local_ns);

/* Query next local hardware timestamp (ns) when node needs attention (for low-power deep sleep) */
int64_t  on_node_next_wakeup_local_ns(const on_node_t *node, int64_t now_local_ns);

/* Query whether the node's RX radio is powered ON at now_local_ns (for low-power TDMA gating) */
bool     on_node_is_rx_open(const on_node_t *node, int64_t now_local_ns);

/* Compute radio duty cycle percentage (0.0 .. 100.0) */
double   on_node_duty_cycle_pct(const on_node_t *node);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_NODE_H */
