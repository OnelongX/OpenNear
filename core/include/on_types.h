#ifndef OPENNEAR_TYPES_H
#define OPENNEAR_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Protocol version & Short-ID (12-bit) address space */
#define ON_PROTO_VERSION          1u
#define ON_SHORT_ID_UNASSIGNED    0x000u
#define ON_SHORT_ID_G_DEFAULT     0x001u
#define ON_SHORT_ID_MAX_UNICAST   0xFFEu
#define ON_SHORT_ID_BROADCAST     0xFFFu
#define ON_SHORT_ID_MASK          0xFFFu

/* Compact 6-byte MAC header + 2-byte payload CRC16 */
#define ON_MAC_HDR_LEN            6u
#define ON_PAYLOAD_CRC_LEN        2u
#define ON_MAX_PAYLOAD_LEN        120u
#define ON_MAX_FRAME_LEN          (ON_MAC_HDR_LEN + ON_MAX_PAYLOAD_LEN + ON_PAYLOAD_CRC_LEN)

/* Default 4-phase dynamic superframe timing (microseconds), T_SF = 4,000 us (250 Hz) */
#define ON_DEFAULT_TSF_US         4000u
#define ON_DEFAULT_SYNC_END_US    240u   /* 52B authenticated SYNC/MAP @ 2Mbps = 216 us air + 24 us guard */
#define ON_DEFAULT_TDMA_START_US  240u
#define ON_DEFAULT_SLOT_LEN_US    160u
#define ON_DEFAULT_MAX_SLOTS      11u    /* 240 .. 2000 us (11 primary 1-hop slots, 40 us guard before Sub-G) */
#define ON_DEFAULT_SUBG_START_US  2040u  /* Carved Sub-G staggered window: 2040 .. 3160 us */
#define ON_DEFAULT_SUBG_LEN_US    1120u
#define ON_DEFAULT_SUBG_SYNC_US   240u   /* Sub-SYNC at offset 0..240 us inside Sub-G window */
#define ON_DEFAULT_SUBG_MAX_SLOTS 4u     /* 4 sub-TDMA slots: 240..880 us inside Sub-G window */
#define ON_DEFAULT_SUBG_CRA_OFF   900u   /* Sub-CRA: 900..1100 us inside Sub-G window */
#define ON_DEFAULT_CRA_START_US   3200u  /* Primary C-RA contention window: 3200 .. 3600 us */
#define ON_DEFAULT_CRA_END_US     3600u
#define ON_DEFAULT_ACT_START_US   3600u  /* ACT/QUIET silent guard band: 3600 .. 4000 us */
#define ON_DEFAULT_ACT_FIRE_US    3700u  /* Exact global synchronized GPIO/LED trigger instant */

/* Failover, lease expiry & Sub-G recovery thresholds */
#define ON_BACKUP_MISS_THRESHOLD  3u     /* 3 missed primary SYNC -> 4th frame 0 us Backup_ID takeover */
#define ON_PEER_LEASE_TIMEOUT_SF  30u    /* Reclaim silent peer's Short-ID & slot after 30 silent SFs */
#define ON_SUB_NODE_DETACH_MISS   8u     /* 2-hop edge node re-scans for new Sub-G after 8 missed Sub-SYNCs */
#define ON_MAX_NODES_TABLE        16u

/* L1 Physical waveform modes */
typedef enum {
    ON_PHY_LE_MSK_2M  = 0,  /* Ultra-low-power single-carrier (zero PAPR, 60-120 us air time) */
    ON_PHY_LE_QPSK_4M = 1,
    ON_PHY_HS_OFDM_20M = 2, /* High-throughput multi-carrier OFDM (15-25 us air time) */
    ON_PHY_ESP32_RAW54M = 3 /* Route C: ESP32 802.11 0xD0 Vendor Action @ 54 Mbps */
} on_phy_mode_t;

/* Frame types (4-bit field inside FrameCtrl) */
typedef enum {
    ON_FRAME_SYNC_MAP   = 0x0,
    ON_FRAME_SUB_SYNC   = 0x1,
    ON_FRAME_TDMA_DATA  = 0x2,
    ON_FRAME_JOIN_REQ   = 0x3,
    ON_FRAME_JOIN_ACK   = 0x4,
    ON_FRAME_RTT_REQ    = 0x5,
    ON_FRAME_RTT_RESP   = 0x6,
    ON_FRAME_CRA_ALERT  = 0x7
} on_frame_type_t;

/* Node operational roles */
typedef enum {
    ON_ROLE_UNJOINED = 0,
    ON_ROLE_G        = 1, /* Primary Grant/Coordinator node */
    ON_ROLE_T        = 2, /* Synchronized Terminal node */
    ON_ROLE_SUB_G    = 3  /* 1-hop T node delegated as Sub-G staggered multi-hop relay */
} on_role_t;

/* 4-phase superframe segments */
typedef enum {
    ON_PHASE_SYNC_MAP  = 0,
    ON_PHASE_D_TDMA    = 1,
    ON_PHASE_SUB_G     = 2,
    ON_PHASE_C_RA      = 3,
    ON_PHASE_ACT_QUIET = 4
} on_phase_t;

/* Return status codes */
typedef enum {
    ON_OK              = 0,
    ON_ERR_INVALID_ARG = -1,
    ON_ERR_TOO_SHORT   = -2,
    ON_ERR_TOO_LONG    = -3,
    ON_ERR_HDR_CRC     = -4,
    ON_ERR_PAY_CRC     = -5,
    ON_ERR_VERSION     = -6,
    ON_ERR_NO_MEM      = -7,
    ON_ERR_NOT_READY   = -8,
    ON_ERR_DECODE_FAIL = -9
} on_err_t;

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_TYPES_H */
