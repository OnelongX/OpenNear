#include "on_sar.h"
#include <string.h>

void on_sar_tx_init(on_sar_tx_t *tx) {
    if (!tx) return;
    memset(tx, 0, sizeof(*tx));
    tx->next_msg_id = 1u;
}

on_err_t on_sar_tx_enqueue(on_sar_tx_t *tx, on_qos_prio_t prio,
                           const uint8_t *msg, size_t msg_len, uint8_t *out_msg_id) {
    if (!tx || !msg || msg_len == 0u || msg_len > ON_SAR_MAX_MSG_LEN) return ON_ERR_INVALID_ARG;
    if ((unsigned)prio >= ON_QOS_LEVELS) return ON_ERR_INVALID_ARG;

    uint8_t nfrags = (uint8_t)((msg_len + ON_SAR_FRAG_MTU - 1u) / ON_SAR_FRAG_MTU);
    if (nfrags > ON_SAR_MAX_FRAGS) return ON_ERR_TOO_LONG;
    if ((uint32_t)tx->count[prio] + nfrags > ON_QOS_QUEUE_CAP) return ON_ERR_NO_MEM;

    uint8_t mid = tx->next_msg_id++;
    if (tx->next_msg_id == 0u) tx->next_msg_id = 1u;

    for (uint8_t i = 0u; i < nfrags; ++i) {
        size_t off = (size_t)i * ON_SAR_FRAG_MTU;
        size_t chunk = msg_len - off;
        if (chunk > ON_SAR_FRAG_MTU) chunk = ON_SAR_FRAG_MTU;

        on_sar_frag_t *slot = &tx->q[prio][tx->tail[prio]];
        slot->valid    = true;
        slot->prio     = prio;
        slot->wire[0]  = mid;
        slot->wire[1]  = i;
        slot->wire[2]  = nfrags;
        slot->wire[3]  = (uint8_t)prio;
        memcpy(slot->wire + ON_SAR_FRAG_HDR_LEN, msg + off, chunk);
        slot->wire_len = ON_SAR_FRAG_HDR_LEN + chunk;

        tx->tail[prio] = (uint8_t)((tx->tail[prio] + 1u) % ON_QOS_QUEUE_CAP);
        tx->count[prio]++;
    }

    if (out_msg_id) *out_msg_id = mid;
    return ON_OK;
}

bool on_sar_tx_pop(on_sar_tx_t *tx, uint8_t *out_wire, size_t cap,
                   size_t *out_len, on_qos_prio_t *out_prio) {
    if (!tx || !out_wire || !out_len) return false;
    for (unsigned p = 0u; p < ON_QOS_LEVELS; ++p) {
        if (tx->count[p] > 0u) {
            on_sar_frag_t *slot = &tx->q[p][tx->head[p]];
            if (cap < slot->wire_len) return false;
            memcpy(out_wire, slot->wire, slot->wire_len);
            *out_len = slot->wire_len;
            if (out_prio) *out_prio = (on_qos_prio_t)p;
            slot->valid = false;
            tx->head[p] = (uint8_t)((tx->head[p] + 1u) % ON_QOS_QUEUE_CAP);
            tx->count[p]--;
            return true;
        }
    }
    return false;
}

void on_sar_rx_init(on_sar_rx_t *rx) {
    if (!rx) return;
    memset(rx, 0, sizeof(*rx));
}

bool on_sar_rx_push(on_sar_rx_t *rx, const uint8_t *frag_wire, size_t frag_len,
                    uint8_t *out_msg, size_t out_cap, size_t *out_msg_len) {
    if (!rx || !frag_wire || frag_len <= ON_SAR_FRAG_HDR_LEN) return false;

    uint8_t mid    = frag_wire[0];
    uint8_t fidx   = frag_wire[1];
    uint8_t fcnt   = frag_wire[2];
    size_t  chunk  = frag_len - ON_SAR_FRAG_HDR_LEN;
    if (fcnt == 0u || fcnt > ON_SAR_MAX_FRAGS || fidx >= fcnt || chunk > ON_SAR_FRAG_MTU) {
        return false;
    }

    if (!rx->active || rx->msg_id != mid) {
        memset(rx, 0, sizeof(*rx));
        rx->active   = true;
        rx->msg_id   = mid;
        rx->frag_cnt = fcnt;
    }

    size_t off = (size_t)fidx * ON_SAR_FRAG_MTU;
    if (off + chunk > ON_SAR_MAX_MSG_LEN) return false;
    memcpy(rx->buf + off, frag_wire + ON_SAR_FRAG_HDR_LEN, chunk);
    rx->rx_mask |= (uint16_t)(1u << fidx);
    if (fidx == fcnt - 1u) {
        rx->last_frag_len = chunk;
    }

    uint16_t full_mask = (uint16_t)((1u << fcnt) - 1u);
    if (rx->rx_mask == full_mask) {
        size_t total = (size_t)(fcnt - 1u) * ON_SAR_FRAG_MTU + rx->last_frag_len;
        if (out_msg && out_cap >= total && out_msg_len) {
            memcpy(out_msg, rx->buf, total);
            *out_msg_len = total;
        }
        rx->active = false;
        return true;
    }
    return false;
}
