#ifndef OPENNEAR_FRAME_H
#define OPENNEAR_FRAME_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear 6-Byte Compact MAC Header (replaces 24+ byte 802.11 header)
 *
 * Byte 0: FrameCtrl  [7:6] version(2b) | [5:2] type(4b) | [1] sec_en(1b) | [0] is_sub_g(1b)
 * Byte 1: Src_Dst[0] src_id[11:4]
 * Byte 2: Src_Dst[1] src_id[3:0] | dst_id[11:8]
 * Byte 3: Src_Dst[2] dst_id[7:0]
 * Byte 4: Seq_Slot   [7:4] sf_seq_low(4b) | [3:0] slot_idx(4b)
 * Byte 5: Hdr_CRC8   CRC-8 (poly 0x07) over Bytes 0..4 (early hardware/SW drop on corrupt header)
 */
typedef struct {
    uint8_t  version;     /* 2-bit */
    uint8_t  type;        /* 4-bit (on_frame_type_t) */
    bool     sec_en;      /* 1-bit hardware AES-CCM flag */
    bool     is_sub_g;    /* 1-bit Sub-G sub-superframe flag */
    uint16_t src_id;      /* 12-bit Short-ID */
    uint16_t dst_id;      /* 12-bit Short-ID */
    uint8_t  sf_seq_low;  /* 4-bit superframe sequence low nibble */
    uint8_t  slot_idx;    /* 4-bit slot index (0..15) */
} on_mac_hdr_t;

/* Decoded SYNC/MAP and SUB_SYNC payload (44 bytes wire format, v3.0 authenticated beacon) */
typedef struct {
    uint32_t sf_seq;            /* Monotonic superframe sequence number */
    int64_t  t_tx_net_ns;       /* Exact network TX timestamp (ns) at preamble correlation point */
    uint16_t t_sf_us;           /* Programmable superframe duration (1000..20000 us) */
    uint16_t backup_id;         /* 12-bit Short-ID of hot-standby Backup G node */
    uint16_t sub_g_id;          /* 12-bit Short-ID of Zone-0 Sub-G relay proxy (0 if none) */
    uint16_t sub_g_id_2;        /* v3.1: 12-bit Short-ID of Zone-1 FDMA Sub-G relay proxy */
    uint16_t sub_g_win_start_us;/* Start offset of carved Sub-G window inside D-TDMA (us) */
    uint16_t sub_g_win_len_us;  /* Length of carved Sub-G window (us) */
    uint16_t tdma_bitmap;       /* 16-bit allocated slot occupancy bitmap */
    uint16_t act_fire_off_us;   /* Offset from superframe start for synchronized ACT trigger */
    uint16_t channel_map;       /* 16-bit TSCH/AFH active channel mask (1=clean, 0=blacklisted) */
    uint16_t ack_bitmap;        /* 16-bit Piggyback Group-ACK mask for previous SF D-TDMA slots */
    uint32_t session_salt;      /* v3.0: Per-boot random salt preventing post-reboot Nonce reuse */
    int16_t  prev_tx_err_ns;    /* v3.1: IEEE 1588 PTP Two-Step follow-up TX departure error (ns) */
    uint8_t  key_epoch;         /* v3.0: Active key rotation epoch (0..255) */
    uint8_t  act_cmd;           /* Synchronized action payload (e.g. LED blink pattern / epoch) */
    uint8_t  hop_depth;         /* 0 for primary G, 1 for Sub-G */
    uint8_t  flags;             /* [7:4] afh_countdown | [3:0] L1 waveform indicator */
    uint8_t  beacon_mac[4];     /* v3.0: 32-bit AES-CBC-MAC anti-spoofing tag over beacon */
} on_sync_payload_t;

#define ON_SYNC_PAYLOAD_WIRE_LEN  44u

/* JOIN_REQ payload (sent in C-RA with 48-bit hardware MAC) */
typedef struct {
    uint8_t  mac48[6];
    uint8_t  capabilities; /* bit0: can_be_backup, bit1: can_be_sub_g */
    uint8_t  hop_hint;     /* 0 = primary G, 1 = via Sub-G */
} on_join_req_t;

#define ON_JOIN_REQ_WIRE_LEN  8u

/* JOIN_ACK payload (assigns 12-bit Short-ID & private D-TDMA slot) */
typedef struct {
    uint8_t  mac48[6];
    uint16_t assigned_id;
    uint8_t  assigned_slot;
    uint8_t  is_sub_slot;
} on_join_ack_t;

#define ON_JOIN_ACK_WIRE_LEN  10u

/* RTT_REQ payload (carries requester's local T1 timestamp) */
typedef struct {
    int64_t  t1_local_ns;
    uint16_t token;
} on_rtt_req_t;

#define ON_RTT_REQ_WIRE_LEN  10u

/* RTT_RESP payload (carries T1, T2, T3 for 4-timestamp link delay calibration) */
typedef struct {
    int64_t  t1_local_ns;
    int64_t  t2_g_rx_ns;
    int64_t  t3_g_tx_ns;
    uint16_t token;
} on_rtt_resp_t;

#define ON_RTT_RESP_WIRE_LEN  26u

/* TDMA_DATA / CRA_ALERT compact telemetry payload */
typedef struct {
    uint16_t origin_id;     /* Original producer Short-ID (preserved when relayed by Sub-G) */
    uint16_t seq;
    int16_t  skew_ns_est;   /* Reported residual sync skew (ns) */
    uint8_t  hop_count;
    uint8_t  flags;         /* bit0: emergency alert, bit1: relayed */
    uint8_t  data[8];
    uint8_t  data_len;
} on_data_payload_t;

#define ON_DATA_PAYLOAD_MIN_WIRE_LEN  9u

/* CRC helpers */
uint8_t  on_crc8(const uint8_t *data, size_t len);
uint16_t on_crc16_ccitt(const uint8_t *data, size_t len);

/* 6-byte MAC header pack / unpack */
on_err_t on_hdr_pack(const on_mac_hdr_t *hdr, uint8_t out6[ON_MAC_HDR_LEN]);
on_err_t on_hdr_unpack(const uint8_t in6[ON_MAC_HDR_LEN], on_mac_hdr_t *hdr);

/* Full frame pack / unpack (6B header + payload + 2B CRC16) */
on_err_t on_frame_pack(const on_mac_hdr_t *hdr,
                       const uint8_t *payload, size_t payload_len,
                       uint8_t *out_buf, size_t out_cap, size_t *out_len);

on_err_t on_frame_unpack(const uint8_t *in_buf, size_t in_len,
                         on_mac_hdr_t *hdr,
                         const uint8_t **payload_ptr, size_t *payload_len);

/* Typed payload codecs (little-endian wire encoding) */
on_err_t on_sync_payload_pack(const on_sync_payload_t *p, uint8_t *out, size_t cap, size_t *len);
on_err_t on_sync_payload_unpack(const uint8_t *in, size_t len, on_sync_payload_t *p);

on_err_t on_join_req_pack(const on_join_req_t *p, uint8_t *out, size_t cap, size_t *len);
on_err_t on_join_req_unpack(const uint8_t *in, size_t len, on_join_req_t *p);

on_err_t on_join_ack_pack(const on_join_ack_t *p, uint8_t *out, size_t cap, size_t *len);
on_err_t on_join_ack_unpack(const uint8_t *in, size_t len, on_join_ack_t *p);

on_err_t on_rtt_req_pack(const on_rtt_req_t *p, uint8_t *out, size_t cap, size_t *len);
on_err_t on_rtt_req_unpack(const uint8_t *in, size_t len, on_rtt_req_t *p);

on_err_t on_rtt_resp_pack(const on_rtt_resp_t *p, uint8_t *out, size_t cap, size_t *len);
on_err_t on_rtt_resp_unpack(const uint8_t *in, size_t len, on_rtt_resp_t *p);

on_err_t on_data_payload_pack(const on_data_payload_t *p, uint8_t *out, size_t cap, size_t *len);
on_err_t on_data_payload_unpack(const uint8_t *in, size_t len, on_data_payload_t *p);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_FRAME_H */
