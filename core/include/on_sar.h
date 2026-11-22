#ifndef OPENNEAR_SAR_H
#define OPENNEAR_SAR_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v3.0 Lightweight Segmentation & Reassembly (SAR) + 3-Level Priority QoS
 *
 * - Enables bulk streams (audio frames, sensor waveforms, OTA firmware blocks up to 1024 B)
 *   over compact OpenNear short TDMA frames (e.g. 64 B MTU per fragment).
 * - Strict preemptive QoS scheduler:
 *     Priority 0 (ON_QOS_CTRL_RT)  : Real-time control / emergency alerts (never blocked by OTA)
 *     Priority 1 (ON_QOS_STREAM)   : Isochronous audio / sensor telemetry
 *     Priority 2 (ON_QOS_BULK_OTA) : Background bulk transfer / firmware OTA
 */

#define ON_SAR_MAX_MSG_LEN    1024u
#define ON_SAR_FRAG_MTU       64u     /* Payload bytes per fragment */
#define ON_SAR_FRAG_HDR_LEN   4u      /* msg_id(1B) | frag_idx(1B) | frag_cnt(1B) | prio_flags(1B) */
#define ON_SAR_MAX_FRAGS      16u
#define ON_QOS_LEVELS         3u
#define ON_QOS_QUEUE_CAP      24u

typedef enum {
    ON_QOS_CTRL_RT  = 0,
    ON_QOS_STREAM   = 1,
    ON_QOS_BULK_OTA = 2
} on_qos_prio_t;

typedef struct {
    bool          valid;
    on_qos_prio_t prio;
    uint8_t       wire[ON_SAR_FRAG_HDR_LEN + ON_SAR_FRAG_MTU];
    size_t        wire_len;
} on_sar_frag_t;

typedef struct {
    uint8_t       next_msg_id;
    on_sar_frag_t q[ON_QOS_LEVELS][ON_QOS_QUEUE_CAP];
    uint8_t       head[ON_QOS_LEVELS];
    uint8_t       tail[ON_QOS_LEVELS];
    uint8_t       count[ON_QOS_LEVELS];
} on_sar_tx_t;

typedef struct {
    bool     active;
    uint8_t  msg_id;
    uint8_t  frag_cnt;
    uint16_t rx_mask;
    uint8_t  buf[ON_SAR_MAX_MSG_LEN];
    size_t   total_len;
    size_t   last_frag_len;
} on_sar_rx_t;

void     on_sar_tx_init(on_sar_tx_t *tx);
on_err_t on_sar_tx_enqueue(on_sar_tx_t *tx, on_qos_prio_t prio,
                           const uint8_t *msg, size_t msg_len, uint8_t *out_msg_id);
/* Dequeues highest-priority ready fragment (Priority 0 always preempts Priority 1 & 2) */
bool     on_sar_tx_pop(on_sar_tx_t *tx, uint8_t *out_wire, size_t cap,
                       size_t *out_len, on_qos_prio_t *out_prio);

void     on_sar_rx_init(on_sar_rx_t *rx);
/* Feeds a received SAR fragment; returns true when complete message is reassembled */
bool     on_sar_rx_push(on_sar_rx_t *rx, const uint8_t *frag_wire, size_t frag_len,
                        uint8_t *out_msg, size_t out_cap, size_t *out_msg_len);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_SAR_H */
