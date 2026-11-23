#include "on_frame.h"
#include <string.h>

/* Little-endian wire serialization helpers */
static void put_u16_le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void put_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t get_u32_le(const uint8_t *p) {
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static void put_i64_le(uint8_t *p, int64_t v) {
    uint64_t u = (uint64_t)v;
    for (int i = 0; i < 8; ++i) {
        p[i] = (uint8_t)((u >> (i * 8)) & 0xFFu);
    }
}

static int64_t get_i64_le(const uint8_t *p) {
    uint64_t u = 0;
    for (int i = 0; i < 8; ++i) {
        u |= ((uint64_t)p[i] << (i * 8));
    }
    return (int64_t)u;
}

uint8_t on_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00u;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1) ^ 0x07u);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint16_t on_crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

on_err_t on_hdr_pack(const on_mac_hdr_t *hdr, uint8_t out6[ON_MAC_HDR_LEN]) {
    if (!hdr || !out6) {
        return ON_ERR_INVALID_ARG;
    }
    uint16_t src = hdr->src_id & ON_SHORT_ID_MASK;
    uint16_t dst = hdr->dst_id & ON_SHORT_ID_MASK;

    /* Byte 0: FrameCtrl */
    out6[0] = (uint8_t)(((hdr->version & 0x03u) << 6) |
                        ((hdr->type & 0x0Fu) << 2)    |
                        ((hdr->sec_en ? 1u : 0u) << 1)|
                        (hdr->is_sub_g ? 1u : 0u));

    /* Bytes 1..3: 12-bit Src_ID + 12-bit Dst_ID packed into 3 bytes */
    out6[1] = (uint8_t)((src >> 4) & 0xFFu);
    out6[2] = (uint8_t)(((src & 0x0Fu) << 4) | ((dst >> 8) & 0x0Fu));
    out6[3] = (uint8_t)(dst & 0xFFu);

    /* Byte 4: Seq_Slot */
    out6[4] = (uint8_t)(((hdr->sf_seq_low & 0x0Fu) << 4) | (hdr->slot_idx & 0x0Fu));

    /* Byte 5: Independent Hdr_CRC8 */
    out6[5] = on_crc8(out6, 5u);
    return ON_OK;
}

on_err_t on_hdr_unpack(const uint8_t in6[ON_MAC_HDR_LEN], on_mac_hdr_t *hdr) {
    if (!in6 || !hdr) {
        return ON_ERR_INVALID_ARG;
    }
    /* Fast independent header CRC8 rejection before payload decode */
    uint8_t expected_crc = on_crc8(in6, 5u);
    if (in6[5] != expected_crc) {
        return ON_ERR_HDR_CRC;
    }

    hdr->version    = (uint8_t)((in6[0] >> 6) & 0x03u);
    hdr->type       = (uint8_t)((in6[0] >> 2) & 0x0Fu);
    hdr->sec_en     = ((in6[0] >> 1) & 0x01u) != 0u;
    hdr->is_sub_g   = (in6[0] & 0x01u) != 0u;

    if (hdr->version != ON_PROTO_VERSION) {
        return ON_ERR_VERSION;
    }

    hdr->src_id     = (uint16_t)(((uint16_t)in6[1] << 4) | ((in6[2] >> 4) & 0x0Fu));
    hdr->dst_id     = (uint16_t)((((uint16_t)in6[2] & 0x0Fu) << 8) | in6[3]);
    hdr->sf_seq_low = (uint8_t)((in6[4] >> 4) & 0x0Fu);
    hdr->slot_idx   = (uint8_t)(in6[4] & 0x0Fu);
    return ON_OK;
}

on_err_t on_frame_pack(const on_mac_hdr_t *hdr,
                       const uint8_t *payload, size_t payload_len,
                       uint8_t *out_buf, size_t out_cap, size_t *out_len) {
    if (!hdr || !out_buf || !out_len) {
        return ON_ERR_INVALID_ARG;
    }
    if (payload_len > ON_MAX_PAYLOAD_LEN) {
        return ON_ERR_TOO_LONG;
    }
    size_t total = ON_MAC_HDR_LEN + payload_len + ON_PAYLOAD_CRC_LEN;
    if (out_cap < total) {
        return ON_ERR_NO_MEM;
    }

    on_err_t rc = on_hdr_pack(hdr, out_buf);
    if (rc != ON_OK) {
        return rc;
    }
    if (payload_len > 0u) {
        if (!payload) {
            return ON_ERR_INVALID_ARG;
        }
        memcpy(out_buf + ON_MAC_HDR_LEN, payload, payload_len);
    }
    uint16_t crc16 = on_crc16_ccitt(out_buf + ON_MAC_HDR_LEN, payload_len);
    put_u16_le(out_buf + ON_MAC_HDR_LEN + payload_len, crc16);
    *out_len = total;
    return ON_OK;
}

on_err_t on_frame_pack_implicit(const on_mac_hdr_t *hdr,
                                const uint8_t *payload, size_t payload_len,
                                uint8_t *out_buf, size_t out_cap, size_t *out_len) {
    if (!hdr || !out_buf || !out_len) return ON_ERR_INVALID_ARG;
    if (payload_len > ON_MAX_PAYLOAD_LEN) return ON_ERR_TOO_LONG;
    size_t total = ON_IMPLICIT_HDR_LEN + payload_len + ON_PAYLOAD_CRC_LEN;
    if (out_cap < total) return ON_ERR_NO_MEM;

    uint8_t virt6[ON_MAC_HDR_LEN];
    on_err_t rc = on_hdr_pack(hdr, virt6);
    if (rc != ON_OK) return rc;

    /* Byte 0: [7:4] sf_seq_low | [3:2] type[1:0] | [1] sec_en | [0] is_sub_g */
    out_buf[0] = (uint8_t)(((hdr->sf_seq_low & 0x0Fu) << 4) |
                           ((hdr->type & 0x03u) << 2)       |
                           ((hdr->sec_en ? 1u : 0u) << 1)   |
                           (hdr->is_sub_g ? 1u : 0u));
    /* Byte 1: Virtual Hdr_CRC8 covering (src_id, dst_id, slot_idx, sf_seq_low, FrameCtrl) */
    out_buf[1] = virt6[5];

    if (payload_len > 0u) {
        if (!payload) return ON_ERR_INVALID_ARG;
        memcpy(out_buf + ON_IMPLICIT_HDR_LEN, payload, payload_len);
    }
    uint16_t crc16 = on_crc16_ccitt(out_buf + ON_IMPLICIT_HDR_LEN, payload_len);
    put_u16_le(out_buf + ON_IMPLICIT_HDR_LEN + payload_len, crc16);
    *out_len = total;
    return ON_OK;
}

on_err_t on_frame_unpack_implicit(const uint8_t *in_buf, size_t in_len,
                                  uint16_t implied_src_id, uint16_t implied_dst_id,
                                  uint8_t implied_slot_idx,
                                  on_mac_hdr_t *hdr,
                                  const uint8_t **payload_ptr, size_t *payload_len) {
    if (!in_buf || !hdr || !payload_ptr || !payload_len) return ON_ERR_INVALID_ARG;
    if (in_len < ON_IMPLICIT_HDR_LEN + ON_PAYLOAD_CRC_LEN) return ON_ERR_TOO_SHORT;

    hdr->version    = ON_PROTO_VERSION;
    hdr->sf_seq_low = (uint8_t)((in_buf[0] >> 4) & 0x0Fu);
    hdr->type       = (uint8_t)((in_buf[0] >> 2) & 0x03u);
    hdr->sec_en     = ((in_buf[0] >> 1) & 0x01u) != 0u;
    hdr->is_sub_g   = (in_buf[0] & 0x01u) != 0u;
    hdr->src_id     = implied_src_id & ON_SHORT_ID_MASK;
    hdr->dst_id     = implied_dst_id & ON_SHORT_ID_MASK;
    hdr->slot_idx   = implied_slot_idx & 0x0Fu;

    uint8_t virt6[ON_MAC_HDR_LEN];
    on_hdr_pack(hdr, virt6);
    if (in_buf[1] != virt6[5]) {
        return ON_ERR_HDR_CRC;
    }

    size_t plen = in_len - ON_IMPLICIT_HDR_LEN - ON_PAYLOAD_CRC_LEN;
    uint16_t rx_crc   = get_u16_le(in_buf + ON_IMPLICIT_HDR_LEN + plen);
    uint16_t calc_crc = on_crc16_ccitt(in_buf + ON_IMPLICIT_HDR_LEN, plen);
    if (rx_crc != calc_crc) {
        return ON_ERR_PAY_CRC;
    }

    *payload_ptr = in_buf + ON_IMPLICIT_HDR_LEN;
    *payload_len = plen;
    return ON_OK;
}

on_err_t on_frame_unpack(const uint8_t *in_buf, size_t in_len,
                         on_mac_hdr_t *hdr,
                         const uint8_t **payload_ptr, size_t *payload_len) {
    if (!in_buf || !hdr || !payload_ptr || !payload_len) {
        return ON_ERR_INVALID_ARG;
    }
    if (in_len < ON_MAC_HDR_LEN + ON_PAYLOAD_CRC_LEN) {
        return ON_ERR_TOO_SHORT;
    }
    on_err_t rc = on_hdr_unpack(in_buf, hdr);
    if (rc != ON_OK) {
        return rc;
    }
    size_t plen = in_len - ON_MAC_HDR_LEN - ON_PAYLOAD_CRC_LEN;
    uint16_t got_crc = get_u16_le(in_buf + ON_MAC_HDR_LEN + plen);
    uint16_t exp_crc = on_crc16_ccitt(in_buf + ON_MAC_HDR_LEN, plen);
    if (got_crc != exp_crc) {
        return ON_ERR_PAY_CRC;
    }
    *payload_ptr = in_buf + ON_MAC_HDR_LEN;
    *payload_len = plen;
    return ON_OK;
}

on_err_t on_sync_payload_pack(const on_sync_payload_t *p, uint8_t *out, size_t cap, size_t *len) {
    if (!p || !out || !len) return ON_ERR_INVALID_ARG;
    if (cap < ON_SYNC_PAYLOAD_WIRE_LEN) return ON_ERR_NO_MEM;

    uint16_t sg2 = p->sub_g_id_2 & ON_SHORT_ID_MASK;
    uint16_t bk_packed  = (uint16_t)((p->backup_id & ON_SHORT_ID_MASK) | ((sg2 & 0x00Fu) << 12));
    uint16_t sg1_packed = (uint16_t)((p->sub_g_id  & ON_SHORT_ID_MASK) | (((sg2 >> 4) & 0x00Fu) << 12));
    /* Byte 36: [7:4] Sub-G-2 ID, [3:1] optional ACT lead, [0] hop depth. */
    uint8_t  hop_packed = (uint8_t)((p->hop_depth & 1u) | (((sg2 >> 8) & 0x00Fu) << 4));
    if (p->hop_depth == 0u && p->act_flip_lead_sf <= 7u) {
        hop_packed |= (uint8_t)(p->act_flip_lead_sf << 1);
    }

    put_u32_le(out + 0,  p->sf_seq);
    put_i64_le(out + 4,  p->t_tx_net_ns);
    put_u16_le(out + 12, p->t_sf_us);
    put_u16_le(out + 14, bk_packed);
    put_u16_le(out + 16, sg1_packed);
    put_u16_le(out + 18, p->sub_g_win_start_us);
    put_u16_le(out + 20, p->sub_g_win_len_us);
    put_u16_le(out + 22, p->tdma_bitmap);
    put_u16_le(out + 24, p->act_fire_off_us);
    put_u16_le(out + 26, p->channel_map);
    put_u16_le(out + 28, p->ack_bitmap);
    put_u32_le(out + 30, p->session_salt);
    out[34] = p->key_epoch;
    out[35] = p->act_cmd;
    out[36] = hop_packed;
    out[37] = p->flags;
    put_u16_le(out + 38, (uint16_t)p->prev_tx_err_ns);
    memcpy(out + 40, p->beacon_mac, 4u);
    *len = ON_SYNC_PAYLOAD_WIRE_LEN;
    return ON_OK;
}

on_err_t on_sync_payload_unpack(const uint8_t *in, size_t len, on_sync_payload_t *p) {
    if (!in || !p) return ON_ERR_INVALID_ARG;
    if (len < ON_SYNC_PAYLOAD_WIRE_LEN) return ON_ERR_TOO_SHORT;

    uint16_t bk_raw  = get_u16_le(in + 14);
    uint16_t sg1_raw = get_u16_le(in + 16);
    uint8_t  hop_raw = in[36];

    p->sf_seq             = get_u32_le(in + 0);
    p->t_tx_net_ns        = get_i64_le(in + 4);
    p->t_sf_us            = get_u16_le(in + 12);
    p->backup_id          = bk_raw  & ON_SHORT_ID_MASK;
    p->sub_g_id           = sg1_raw & ON_SHORT_ID_MASK;
    p->sub_g_id_2         = (uint16_t)(((bk_raw >> 12) & 0x00Fu) |
                                       (((sg1_raw >> 12) & 0x00Fu) << 4) |
                                       (((uint16_t)(hop_raw >> 4) & 0x00Fu) << 8));
    p->sub_g_win_start_us = get_u16_le(in + 18);
    p->sub_g_win_len_us   = get_u16_le(in + 20);
    p->tdma_bitmap        = get_u16_le(in + 22);
    p->act_fire_off_us    = get_u16_le(in + 24);
    p->channel_map        = get_u16_le(in + 26);
    p->ack_bitmap         = get_u16_le(in + 28);
    p->session_salt       = get_u32_le(in + 30);
    p->key_epoch          = in[34];
    p->act_cmd            = in[35];
    p->hop_depth          = hop_raw & 1u;
    p->act_flip_lead_sf   = p->hop_depth == 0u ? (hop_raw >> 1) & 7u : 0u;
    p->flags              = in[37];
    p->prev_tx_err_ns     = (int16_t)get_u16_le(in + 38);
    memcpy(p->beacon_mac, in + 40, 4u);
    return ON_OK;
}

on_err_t on_join_req_pack(const on_join_req_t *p, uint8_t *out, size_t cap, size_t *len) {
    if (!p || !out || !len) return ON_ERR_INVALID_ARG;
    if (cap < ON_JOIN_REQ_WIRE_LEN) return ON_ERR_NO_MEM;
    memcpy(out, p->mac48, 6u);
    out[6] = p->capabilities;
    out[7] = p->hop_hint;
    *len = ON_JOIN_REQ_WIRE_LEN;
    return ON_OK;
}

on_err_t on_join_req_unpack(const uint8_t *in, size_t len, on_join_req_t *p) {
    if (!in || !p) return ON_ERR_INVALID_ARG;
    if (len < ON_JOIN_REQ_WIRE_LEN) return ON_ERR_TOO_SHORT;
    memcpy(p->mac48, in, 6u);
    p->capabilities = in[6];
    p->hop_hint     = in[7];
    return ON_OK;
}

on_err_t on_join_ack_pack(const on_join_ack_t *p, uint8_t *out, size_t cap, size_t *len) {
    if (!p || !out || !len) return ON_ERR_INVALID_ARG;
    if (cap < ON_JOIN_ACK_WIRE_LEN) return ON_ERR_NO_MEM;
    memcpy(out, p->mac48, 6u);
    put_u16_le(out + 6, p->assigned_id & ON_SHORT_ID_MASK);
    out[8] = p->assigned_slot;
    out[9] = p->is_sub_slot;
    *len = ON_JOIN_ACK_WIRE_LEN;
    return ON_OK;
}

on_err_t on_join_ack_unpack(const uint8_t *in, size_t len, on_join_ack_t *p) {
    if (!in || !p) return ON_ERR_INVALID_ARG;
    if (len < ON_JOIN_ACK_WIRE_LEN) return ON_ERR_TOO_SHORT;
    memcpy(p->mac48, in, 6u);
    p->assigned_id   = get_u16_le(in + 6) & ON_SHORT_ID_MASK;
    p->assigned_slot = in[8];
    p->is_sub_slot   = in[9];
    return ON_OK;
}

on_err_t on_rtt_req_pack(const on_rtt_req_t *p, uint8_t *out, size_t cap, size_t *len) {
    if (!p || !out || !len) return ON_ERR_INVALID_ARG;
    if (cap < ON_RTT_REQ_WIRE_LEN) return ON_ERR_NO_MEM;
    put_i64_le(out + 0, p->t1_local_ns);
    put_u16_le(out + 8, p->token);
    *len = ON_RTT_REQ_WIRE_LEN;
    return ON_OK;
}

on_err_t on_rtt_req_unpack(const uint8_t *in, size_t len, on_rtt_req_t *p) {
    if (!in || !p) return ON_ERR_INVALID_ARG;
    if (len < ON_RTT_REQ_WIRE_LEN) return ON_ERR_TOO_SHORT;
    p->t1_local_ns = get_i64_le(in + 0);
    p->token       = get_u16_le(in + 8);
    return ON_OK;
}

on_err_t on_rtt_resp_pack(const on_rtt_resp_t *p, uint8_t *out, size_t cap, size_t *len) {
    if (!p || !out || !len) return ON_ERR_INVALID_ARG;
    if (cap < ON_RTT_RESP_WIRE_LEN) return ON_ERR_NO_MEM;
    put_i64_le(out + 0,  p->t1_local_ns);
    put_i64_le(out + 8,  p->t2_g_rx_ns);
    put_i64_le(out + 16, p->t3_g_tx_ns);
    put_u16_le(out + 24, p->token);
    *len = ON_RTT_RESP_WIRE_LEN;
    return ON_OK;
}

on_err_t on_rtt_resp_unpack(const uint8_t *in, size_t len, on_rtt_resp_t *p) {
    if (!in || !p) return ON_ERR_INVALID_ARG;
    if (len < ON_RTT_RESP_WIRE_LEN) return ON_ERR_TOO_SHORT;
    p->t1_local_ns = get_i64_le(in + 0);
    p->t2_g_rx_ns  = get_i64_le(in + 8);
    p->t3_g_tx_ns  = get_i64_le(in + 16);
    p->token       = get_u16_le(in + 24);
    return ON_OK;
}

on_err_t on_data_payload_pack(const on_data_payload_t *p, uint8_t *out, size_t cap, size_t *len) {
    if (!p || !out || !len) return ON_ERR_INVALID_ARG;
    uint8_t dlen = p->data_len > 8u ? 8u : p->data_len;
    size_t need = ON_DATA_PAYLOAD_MIN_WIRE_LEN + dlen;
    if (cap < need) return ON_ERR_NO_MEM;
    put_u16_le(out + 0, p->origin_id & ON_SHORT_ID_MASK);
    put_u16_le(out + 2, p->seq);
    put_u16_le(out + 4, (uint16_t)p->skew_ns_est);
    out[6] = p->hop_count;
    out[7] = p->flags;
    out[8] = dlen;
    if (dlen > 0u) {
        memcpy(out + 9, p->data, dlen);
    }
    *len = need;
    return ON_OK;
}

on_err_t on_data_payload_unpack(const uint8_t *in, size_t len, on_data_payload_t *p) {
    if (!in || !p) return ON_ERR_INVALID_ARG;
    if (len < ON_DATA_PAYLOAD_MIN_WIRE_LEN) return ON_ERR_TOO_SHORT;
    p->origin_id   = get_u16_le(in + 0) & ON_SHORT_ID_MASK;
    p->seq         = get_u16_le(in + 2);
    p->skew_ns_est = (int16_t)get_u16_le(in + 4);
    p->hop_count   = in[6];
    p->flags       = in[7];
    p->data_len    = in[8];
    if (p->data_len > 8u || len < ON_DATA_PAYLOAD_MIN_WIRE_LEN + p->data_len) {
        return ON_ERR_TOO_SHORT;
    }
    if (p->data_len > 0u) {
        memcpy(p->data, in + 9, p->data_len);
    }
    return ON_OK;
}
