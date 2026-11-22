#include "on_node.h"
#include <string.h>

static void emit_event(on_node_t *node, const char *name, uint32_t a1, int64_t a2) {
    if (node && node->hal.on_event) {
        node->hal.on_event(node->hal.user_ctx, name, a1, a2);
    }
}

static on_peer_entry_t *find_peer_by_mac(on_node_t *node, const uint8_t mac48[6]) {
    for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
        if (node->peers[i].active && memcmp(node->peers[i].mac48, mac48, 6u) == 0) {
            return &node->peers[i];
        }
    }
    return NULL;
}

static on_peer_entry_t *find_peer_by_id(on_node_t *node, uint16_t short_id) {
    for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
        if (node->peers[i].active && node->peers[i].short_id == short_id) {
            return &node->peers[i];
        }
    }
    return NULL;
}

static uint16_t select_next_backup_id(const on_node_t *node) {
    for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
        if (node->peers[i].active &&
            !node->peers[i].is_sub_slot &&
            node->peers[i].short_id != node->short_id &&
            node->peers[i].short_id != node->sub_g_id &&
            (node->peers[i].capabilities & 0x01u)) {
            return node->peers[i].short_id;
        }
    }
    /* Fallback: any active 1-hop peer */
    for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
        if (node->peers[i].active &&
            !node->peers[i].is_sub_slot &&
            node->peers[i].short_id != node->short_id) {
            return node->peers[i].short_id;
        }
    }
    return ON_SHORT_ID_UNASSIGNED;
}

static uint16_t select_next_subg_id(const on_node_t *node) {
    for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
        if (node->peers[i].active &&
            !node->peers[i].is_sub_slot &&
            node->peers[i].short_id != node->short_id &&
            node->peers[i].short_id != node->backup_id &&
            (node->peers[i].capabilities & 0x02u)) {
            return node->peers[i].short_id;
        }
    }
    return ON_SHORT_ID_UNASSIGNED;
}

static uint16_t alloc_unique_short_id(on_node_t *node) {
    for (uint32_t tries = 0u; tries < 250u; ++tries) {
        uint16_t cand = node->next_alloc_id++;
        if (node->next_alloc_id > 0x0FEu || node->next_alloc_id < 0x002u) {
            node->next_alloc_id = 0x002u;
        }
        if (cand < 0x002u || cand > 0x0FEu) {
            cand = 0x002u;
        }
        if (cand != node->short_id && find_peer_by_id(node, cand) == NULL) {
            return cand;
        }
    }
    return 0x0FEu;
}

static bool is_secured_frame_type(uint8_t type) {
    return (type == ON_FRAME_TDMA_DATA ||
            type == ON_FRAME_CRA_ALERT ||
            type == ON_FRAME_RTT_REQ   ||
            type == ON_FRAME_RTT_RESP  ||
            type == ON_FRAME_JOIN_REQ  ||
            type == ON_FRAME_JOIN_ACK);
}

static void tune_channel(on_node_t *node, uint8_t ch_idx) {
    node->afh.current_ch = ch_idx;
    if (node->afh.enabled && node->hal.radio_set_channel) {
        node->hal.radio_set_channel(node->hal.user_ctx, ch_idx);
    }
}

static uint8_t popcount16(uint16_t x) {
    uint8_t c = 0u;
    while (x) {
        c += (uint8_t)(x & 1u);
        x >>= 1u;
    }
    return c;
}

static void send_frame(on_node_t *node, const on_mac_hdr_t *hdr,
                       const uint8_t *payload, size_t plen, int64_t tx_local_ns) {
    on_mac_hdr_t tx_hdr = *hdr;
    uint8_t sec_buf[ON_MAX_PAYLOAD_LEN];
    const uint8_t *wire_pay = payload;
    size_t wire_plen = plen;

    if (node->sec.enabled && is_secured_frame_type(tx_hdr.type)) {
        if (plen + ON_SEC_MIC_LEN > ON_MAX_PAYLOAD_LEN) return;
        tx_hdr.sec_en = true;
        uint8_t aad6[ON_MAC_HDR_LEN];
        uint8_t nonce[ON_SEC_NONCE_LEN];
        if (on_hdr_pack(&tx_hdr, aad6) != ON_OK) return;
        on_sec_build_nonce_salted(&tx_hdr, node->current_sf_seq, node->sec.session_salt, nonce);
        if (on_ccm_encrypt(node->sec.key, nonce, aad6, ON_MAC_HDR_LEN,
                           payload, plen, sec_buf, sec_buf + plen) != ON_OK) {
            return;
        }
        wire_pay  = sec_buf;
        wire_plen = plen + ON_SEC_MIC_LEN;
    }

    uint8_t buf[ON_MAX_FRAME_LEN];
    size_t flen = 0u;
    if (on_frame_pack(&tx_hdr, wire_pay, wire_plen, buf, sizeof(buf), &flen) == ON_OK) {
        if (node->hal.tx_raw) {
            node->hal.tx_raw(node->hal.user_ctx, buf, flen, tx_local_ns);
            /* Approximate LE 2Mbps air-time (8 us preamble + 4 us/byte) for duty-cycle accounting */
            node->radio_active_ns += (uint64_t)(8000u + flen * 4000u);
        }
    }
}

static void send_sync_map(on_node_t *node, bool is_sub_sync) {
    int64_t target_tx_net_ns = is_sub_sync
        ? (node->current_sf_start_net_ns + (int64_t)node->sf_cfg.sub_g_start_us * 1000LL)
        : node->current_sf_start_net_ns;
    int64_t target_tx_loc_ns = on_sync_net_to_local(&node->sync, target_tx_net_ns);

    on_sync_payload_t sp;
    memset(&sp, 0, sizeof(sp));
    sp.sf_seq             = node->current_sf_seq;
    sp.t_tx_net_ns        = target_tx_net_ns;
    sp.t_sf_us            = node->sf_cfg.t_sf_us;
    sp.backup_id          = is_sub_sync ? ON_SHORT_ID_UNASSIGNED : node->backup_id;
    sp.sub_g_id           = node->sub_g_id;
    sp.sub_g_win_start_us = node->sf_cfg.sub_g_start_us;
    sp.sub_g_win_len_us   = node->sf_cfg.sub_g_len_us;
    sp.tdma_bitmap        = is_sub_sync ? node->sub_tdma_bitmap : node->tdma_bitmap;
    sp.act_fire_off_us    = node->sf_cfg.act_fire_off_us;
    sp.channel_map        = node->has_pending_ch_map ? node->pending_channel_map : node->afh.channel_map;
    sp.ack_bitmap         = is_sub_sync ? node->last_sub_ack_bitmap : node->last_ack_bitmap;
    sp.session_salt       = node->sec.session_salt;
    sp.key_epoch          = node->sec.key_epoch;
    sp.act_cmd            = node->act_cmd;
    sp.hop_depth          = is_sub_sync ? 1u : 0u;
    sp.flags              = (uint8_t)(((node->afh_countdown & 0x0Fu) << 4) | (ON_PHY_LE_MSK_2M & 0x0Fu));

    uint8_t pbuf[ON_SYNC_PAYLOAD_WIRE_LEN];
    size_t plen = 0u;
    on_sync_payload_pack(&sp, pbuf, sizeof(pbuf), &plen);

    on_mac_hdr_t hdr;
    hdr.version    = ON_PROTO_VERSION;
    hdr.type       = is_sub_sync ? (uint8_t)ON_FRAME_SUB_SYNC : (uint8_t)ON_FRAME_SYNC_MAP;
    hdr.sec_en     = false;
    hdr.is_sub_g   = is_sub_sync;
    hdr.src_id     = node->short_id;
    hdr.dst_id     = ON_SHORT_ID_BROADCAST;
    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
    hdr.slot_idx   = 0u;

    /* v3.0 Compute 32-bit AES-CBC-MAC anti-spoofing tag over header + beacon body */
    if (node->sec.enabled) {
        uint8_t hdr6[ON_MAC_HDR_LEN];
        if (on_hdr_pack(&hdr, hdr6) == ON_OK) {
            on_sec_compute_beacon_mac(&node->sec, hdr6, pbuf, sp.sf_seq,
                                      sp.session_salt, pbuf + 40);
        }
    }

    send_frame(node, &hdr, pbuf, plen, target_tx_loc_ns);
    if (is_sub_sync) {
        node->sub_sync_tx_total++;
    }
}

void on_node_init(on_node_t *node, const on_hal_ops_t *hal,
                  on_role_t initial_role, uint8_t capabilities) {
    if (!node) return;
    memset(node, 0, sizeof(*node));
    if (hal) {
        node->hal = *hal;
    }
    on_sf_cfg_default(&node->sf_cfg);
    on_sync_init(&node->sync);
    on_sec_init(&node->sec);
    on_afh_init(&node->afh, false);

    if (node->hal.get_mac48) {
        node->hal.get_mac48(node->hal.user_ctx, node->mac48);
    }

    node->capabilities = capabilities;
    node->role         = initial_role;
    node->next_alloc_id = 0x002u;

    if (initial_role == ON_ROLE_G) {
        node->short_id = ON_SHORT_ID_G_DEFAULT;
        node->sync.locked = true;
        node->sync.ref_local_ns = node->hal.now_ns ? node->hal.now_ns(node->hal.user_ctx) : 0;
        node->sync.ref_net_ns   = node->sync.ref_local_ns;
        node->current_sf_start_net_ns = node->sync.ref_net_ns;
        node->current_sf_seq = 0u;
        node->act_cmd = 1u;
    } else {
        node->short_id = ON_SHORT_ID_UNASSIGNED;
        node->rtt_next_sf = 3u;
    }
}

on_err_t on_node_register_static_peer(on_node_t *node, const uint8_t mac48[6],
                                      uint16_t short_id, uint8_t slot_idx,
                                      uint8_t capabilities, bool is_sub_slot) {
    if (!node || !mac48) return ON_ERR_INVALID_ARG;
    for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
        if (!node->peers[i].active) {
            node->peers[i].active       = true;
            memcpy(node->peers[i].mac48, mac48, 6u);
            node->peers[i].short_id     = short_id & ON_SHORT_ID_MASK;
            node->peers[i].slot_idx     = slot_idx;
            node->peers[i].is_sub_slot  = is_sub_slot;
            node->peers[i].capabilities = capabilities;
            if (is_sub_slot) {
                node->sub_tdma_bitmap |= (uint16_t)(1u << slot_idx);
            } else {
                node->tdma_bitmap |= (uint16_t)(1u << slot_idx);
            }
            if (short_id >= node->next_alloc_id) {
                node->next_alloc_id = (uint16_t)(short_id + 1u);
            }
            return ON_OK;
        }
    }
    return ON_ERR_NO_MEM;
}

void on_node_set_backup_and_subg(on_node_t *node, uint16_t backup_id, uint16_t sub_g_id) {
    if (!node) return;
    node->backup_id = backup_id & ON_SHORT_ID_MASK;
    node->sub_g_id  = sub_g_id & ON_SHORT_ID_MASK;
}

void on_node_enable_security(on_node_t *node, const uint8_t key16[ON_SEC_KEY_LEN]) {
    if (!node || !key16) return;
    on_sec_set_key(&node->sec, key16);
}

void on_node_set_session_salt(on_node_t *node, uint32_t session_salt) {
    if (!node) return;
    on_sec_set_session(&node->sec, session_salt, node->sec.key_epoch);
}

void on_node_rotate_key_epoch(on_node_t *node) {
    if (!node || !node->sec.enabled) return;
    /* Defer switch to next superframe 0 us boundary so in-flight D-TDMA slots never drop */
    node->pending_key_rotation = true;
}

void on_node_enable_afh(on_node_t *node, bool enable) {
    if (!node) return;
    on_afh_init(&node->afh, enable);
    if (enable) {
        tune_channel(node, on_afh_channel_for(&node->afh, node->current_sf_seq, 0u));
    }
}

void on_node_trigger_alert(on_node_t *node, uint8_t alert_code) {
    if (!node) return;
    node->pending_alert = true;
    node->alert_code    = alert_code;
}

/* Handle JOIN_REQ received by Primary G or Sub-G proxy */
static void handle_join_req(on_node_t *node, const on_mac_hdr_t *hdr,
                            const uint8_t *payload, size_t plen, int64_t rx_local_ns) {
    bool as_sub = (node->role == ON_ROLE_SUB_G && hdr->is_sub_g);
    bool as_pri = (node->role == ON_ROLE_G && !hdr->is_sub_g);
    if (!as_sub && !as_pri) return;

    on_join_req_t req;
    if (on_join_req_unpack(payload, plen, &req) != ON_OK) return;

    on_peer_entry_t *pe = find_peer_by_mac(node, req.mac48);
    if (!pe) {
        /* Allocate Short-ID and free slot */
        uint8_t max_slots = as_sub ? ON_DEFAULT_SUBG_MAX_SLOTS : node->sf_cfg.max_slots;
        uint16_t *bmap = as_sub ? &node->sub_tdma_bitmap : &node->tdma_bitmap;
        uint8_t free_slot = 0xFFu;
        for (uint8_t s = 0; s < max_slots; ++s) {
            if (((*bmap) & (1u << s)) == 0u) {
                free_slot = s;
                break;
            }
        }
        if (free_slot == 0xFFu) return;

        for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
            if (!node->peers[i].active) {
                pe = &node->peers[i];
                memset(pe, 0, sizeof(*pe));
                pe->active       = true;
                memcpy(pe->mac48, req.mac48, 6u);
                pe->short_id     = as_sub ? (uint16_t)(0x100u + free_slot) : alloc_unique_short_id(node);
                pe->slot_idx     = free_slot;
                pe->is_sub_slot  = as_sub;
                pe->capabilities = req.capabilities;
                *bmap |= (uint16_t)(1u << free_slot);
                break;
            }
        }
        if (!pe) return;

        /* Auto-elect Backup_ID or Sub-G if Primary G has none assigned yet */
        if (as_pri) {
            if (node->backup_id == ON_SHORT_ID_UNASSIGNED && (req.capabilities & 0x01u)) {
                node->backup_id = pe->short_id;
            } else if (node->sub_g_id == ON_SHORT_ID_UNASSIGNED && (req.capabilities & 0x02u)) {
                node->sub_g_id = pe->short_id;
            }
        }
    }

    pe->last_seen_sf = node->current_sf_seq;

    on_join_ack_t ack;
    memcpy(ack.mac48, pe->mac48, 6u);
    ack.assigned_id   = pe->short_id;
    ack.assigned_slot = pe->slot_idx;
    ack.is_sub_slot   = as_sub ? 1u : 0u;

    uint8_t abuf[ON_JOIN_ACK_WIRE_LEN];
    size_t alen = 0u;
    on_join_ack_pack(&ack, abuf, sizeof(abuf), &alen);

    on_mac_hdr_t rhdr;
    rhdr.version    = ON_PROTO_VERSION;
    rhdr.type       = (uint8_t)ON_FRAME_JOIN_ACK;
    rhdr.sec_en     = false;
    rhdr.is_sub_g   = as_sub;
    rhdr.src_id     = node->short_id;
    rhdr.dst_id     = ON_SHORT_ID_BROADCAST;
    rhdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
    rhdr.slot_idx   = pe->slot_idx;

    /* Respond 25 us after frame reception completes (strictly in the future) */
    int64_t now_loc = node->hal.now_ns ? node->hal.now_ns(node->hal.user_ctx) : rx_local_ns;
    int64_t tx_loc  = (now_loc > rx_local_ns ? now_loc : rx_local_ns) + 25000LL;
    send_frame(node, &rhdr, abuf, alen, tx_loc);
    emit_event(node, as_sub ? "SUB_JOIN_ASSIGNED" : "JOIN_ASSIGNED", pe->short_id, pe->slot_idx);
}

void on_node_on_rx_raw(on_node_t *node, const uint8_t *frame, size_t len, int64_t rx_local_ns) {
    if (!node || !frame) return;

    on_mac_hdr_t hdr;
    const uint8_t *payload = NULL;
    size_t plen = 0u;

    on_err_t rc = on_frame_unpack(frame, len, &hdr, &payload, &plen);
    if (rc == ON_ERR_HDR_CRC) {
        node->hdr_crc_drops++;
        return;
    }
    if (rc != ON_OK) {
        if (rc == ON_ERR_PAY_CRC) node->pay_crc_drops++;
        return;
    }

    /* Account RX air time */
    node->radio_active_ns += (uint64_t)(8000u + len * 4000u);

    /* Ignore own loopback */
    if (node->short_id != ON_SHORT_ID_UNASSIGNED && hdr.src_id == node->short_id) {
        return;
    }

    /* Fast L2 destination filter: skip unicast frames not addressed to us (except Backup_ID passive mirror) */
    bool is_for_me = (hdr.dst_id == ON_SHORT_ID_BROADCAST) ||
                     (node->short_id != ON_SHORT_ID_UNASSIGNED && hdr.dst_id == node->short_id) ||
                     (node->short_id != ON_SHORT_ID_UNASSIGNED && node->short_id == node->backup_id &&
                      hdr.type == ON_FRAME_TDMA_DATA && !hdr.is_sub_g);
    if (!is_for_me) {
        return;
    }

    /* v2.0 AES-128-CCM AEAD decryption & monotonic anti-replay enforcement */
    uint8_t dec_buf[ON_MAX_PAYLOAD_LEN];
    if (hdr.sec_en || (node->sec.enabled && is_secured_frame_type(hdr.type))) {
        if (!node->sec.enabled || !hdr.sec_en || plen < ON_SEC_MIC_LEN) {
            node->sec.auth_failures++;
            return;
        }
        /* Reconcile full 32-bit sf_seq from 4-bit header low nibble */
        uint32_t rx_sf_seq = node->current_sf_seq;
        if ((rx_sf_seq & 0x0Fu) != hdr.sf_seq_low) {
            if (((rx_sf_seq + 1u) & 0x0Fu) == hdr.sf_seq_low) {
                rx_sf_seq += 1u;
            } else if (rx_sf_seq > 0u && ((rx_sf_seq - 1u) & 0x0Fu) == hdr.sf_seq_low) {
                rx_sf_seq -= 1u;
            }
        }
        size_t plain_len = plen - ON_SEC_MIC_LEN;
        uint8_t nonce[ON_SEC_NONCE_LEN];
        on_sec_build_nonce_salted(&hdr, rx_sf_seq, node->sec.session_salt, nonce);
        if (on_ccm_decrypt(node->sec.key, nonce, frame, ON_MAC_HDR_LEN,
                           payload, plain_len, payload + plain_len, dec_buf) != ON_OK) {
            node->sec.auth_failures++;
            emit_event(node, "SEC_AUTH_DROP", hdr.src_id, (int64_t)rx_sf_seq);
            return;
        }
        if (!on_sec_check_replay(&node->sec, hdr.src_id, rx_sf_seq, hdr.slot_idx, hdr.type)) {
            emit_event(node, "SEC_REPLAY_DROP", hdr.src_id, (int64_t)rx_sf_seq);
            return;
        }
        payload = dec_buf;
        plen    = plain_len;
    }

    switch ((on_frame_type_t)hdr.type) {
        case ON_FRAME_SYNC_MAP:
        case ON_FRAME_SUB_SYNC: {
            bool is_sub = (hdr.type == ON_FRAME_SUB_SYNC);

            if (node->role == ON_ROLE_G) {
                /* Split-brain reconciliation: yield if a lower Short-ID Primary G is active */
                if (!is_sub && hdr.src_id < node->short_id) {
                    node->role = ON_ROLE_T;
                    emit_event(node, "G_YIELD_TO_PRIMARY", hdr.src_id, (int64_t)node->current_sf_seq);
                } else {
                    break;
                }
            }

            /* Never let a 1-hop node that hears Primary SYNC_MAP downgrade to SUB_SYNC */
            if (!is_sub) {
                node->heard_primary_sync = true;
            } else if (node->heard_primary_sync) {
                break;
            }

            /* 1-hop joined nodes ignore SUB_SYNC; 2-hop edge nodes only lock to SUB_SYNC */
            if (node->role != ON_ROLE_UNJOINED) {
                if (!node->is_sub_node && is_sub) break;
                if (node->is_sub_node && !is_sub) break;
            }

            on_sync_payload_t sp;
            if (on_sync_payload_unpack(payload, plen, &sp) != ON_OK) break;

            /* v3.0 Verify 32-bit AES-CBC-MAC anti-spoofing tag before trusting beacon */
            if (node->sec.enabled) {
                if (!on_sec_verify_beacon_mac(&node->sec, frame, payload, sp.sf_seq, sp.session_salt)) {
                    emit_event(node, "BEACON_SPOOF_DROP", hdr.src_id, (int64_t)sp.sf_seq);
                    break;
                }
                /* Adopt Primary G's session_salt & key_epoch (online KDF key rotation) */
                on_sec_set_session(&node->sec, sp.session_salt, sp.key_epoch);
            }

            /* Step 1 + Step 3: Hardware RX capture feeds TC-PLL / NCO lock */
            on_sync_on_beacon(&node->sync, sp.t_tx_net_ns, rx_local_ns);
            node->sync_rx_total++;
            node->sync_rx_this_sf   = true;
            node->missed_sync_count = 0u;
            node->parent_id         = hdr.src_id;
            node->current_sf_seq    = sp.sf_seq;
            node->sf_cfg.t_sf_us         = sp.t_sf_us;
            node->sf_cfg.sub_g_start_us  = sp.sub_g_win_start_us;
            node->sf_cfg.sub_g_len_us    = sp.sub_g_win_len_us;
            node->sf_cfg.act_fire_off_us = sp.act_fire_off_us;
            node->act_cmd           = sp.act_cmd;

            if (is_sub) {
                /* Sub-SYNC starts at offset sub_g_win_start_us inside primary superframe */
                node->current_sf_start_net_ns = sp.t_tx_net_ns - (int64_t)sp.sub_g_win_start_us * 1000LL;
                node->sub_tdma_bitmap = sp.tdma_bitmap;
                if (node->role == ON_ROLE_UNJOINED) {
                    node->is_sub_node = true;
                }
            } else {
                node->current_sf_start_net_ns = sp.t_tx_net_ns;
                node->tdma_bitmap = sp.tdma_bitmap;
                node->backup_id   = sp.backup_id;
                node->sub_g_id    = sp.sub_g_id;
                if (node->role == ON_ROLE_UNJOINED) {
                    node->is_sub_node = false;
                }
                /* Promote to Sub-G proxy if designated by primary G */
                if (node->role == ON_ROLE_T &&
                    node->short_id != ON_SHORT_ID_UNASSIGNED &&
                    node->short_id == sp.sub_g_id) {
                    node->role = ON_ROLE_SUB_G;
                    emit_event(node, "PROMOTED_SUB_G", node->short_id, (int64_t)sp.sf_seq);
                }
            }

            /* v2.0 AFH synchronized countdown channel_map adoption (BLE Instant style) */
            if (node->afh.enabled && sp.channel_map != 0u) {
                uint8_t cd = (uint8_t)((sp.flags >> 4) & 0x0Fu);
                if (cd > 0u) {
                    node->pending_channel_map = sp.channel_map;
                    node->has_pending_ch_map  = true;
                    node->afh_countdown       = cd;
                } else if (!node->has_pending_ch_map && sp.channel_map != node->afh.channel_map) {
                    /* Late-joining node catches up immediately when no countdown is active */
                    on_afh_apply_map(&node->afh, sp.channel_map);
                }
            }

            /* v2.0 Piggyback Group-ACK Fast HARQ check for previous superframe's D-TDMA TX */
            if ((node->role == ON_ROLE_T || node->role == ON_ROLE_SUB_G) && node->awaiting_ack) {
                bool acked = ((sp.ack_bitmap & (uint16_t)(1u << node->my_slot)) != 0u);
                if (acked) {
                    if (node->last_tx_data.flags & 0x04u) {
                        node->harq_recovered_total++;
                    }
                    node->awaiting_ack       = false;
                    node->harq_retry_pending = false;
                } else {
                    /* NACK -> trigger immediate slot-local HARQ retransmission this superframe */
                    node->harq_retry_pending = true;
                }
            }

            /* Reset per-superframe action flags when a new SYNC arrives */
            node->act_fired_this_sf        = false;
            node->tdma_tx_done_this_sf     = false;
            node->sub_sync_tx_done_this_sf = false;
            node->cra_tx_done_this_sf      = false;
            node->ch_pretuned_this_sf      = false;
            break;
        }

        case ON_FRAME_JOIN_REQ:
            handle_join_req(node, &hdr, payload, plen, rx_local_ns);
            break;

        case ON_FRAME_JOIN_ACK: {
            if (node->role != ON_ROLE_UNJOINED) break;
            on_join_ack_t ack;
            if (on_join_ack_unpack(payload, plen, &ack) != ON_OK) break;
            if (memcmp(ack.mac48, node->mac48, 6u) == 0) {
                node->short_id    = ack.assigned_id;
                node->my_slot     = ack.assigned_slot;
                node->is_sub_node = (ack.is_sub_slot != 0u);
                node->parent_id   = hdr.src_id;
                node->role        = ON_ROLE_T;
                node->rtt_next_sf = node->current_sf_seq + 1u;
                emit_event(node, node->is_sub_node ? "JOINED_VIA_SUBG" : "JOINED_PRIMARY",
                           node->short_id, node->my_slot);
            }
            break;
        }

        case ON_FRAME_RTT_REQ: {
            if (hdr.dst_id != node->short_id) break;
            on_rtt_req_t req;
            if (on_rtt_req_unpack(payload, plen, &req) != ON_OK) break;

            int64_t now_loc = node->hal.now_ns ? node->hal.now_ns(node->hal.user_ctx) : rx_local_ns;
            int64_t t2_net = (node->role == ON_ROLE_G)
                           ? rx_local_ns
                           : on_sync_local_to_net(&node->sync, rx_local_ns);
            /* Schedule hardware TX strictly AFTER RX completion so T3 matches physical TX start */
            int64_t tx_local = (now_loc > rx_local_ns ? now_loc : rx_local_ns) + 25000LL;
            int64_t t3_net = (node->role == ON_ROLE_G)
                           ? tx_local
                           : on_sync_local_to_net(&node->sync, tx_local);

            on_rtt_resp_t resp;
            resp.t1_local_ns = req.t1_local_ns;
            resp.t2_g_rx_ns  = t2_net;
            resp.t3_g_tx_ns  = t3_net;
            resp.token       = req.token;

            uint8_t rbuf[ON_RTT_RESP_WIRE_LEN];
            size_t rlen = 0u;
            on_rtt_resp_pack(&resp, rbuf, sizeof(rbuf), &rlen);

            on_mac_hdr_t rhdr;
            rhdr.version    = ON_PROTO_VERSION;
            rhdr.type       = (uint8_t)ON_FRAME_RTT_RESP;
            rhdr.sec_en     = false;
            rhdr.is_sub_g   = hdr.is_sub_g;
            rhdr.src_id     = node->short_id;
            rhdr.dst_id     = hdr.src_id;
            rhdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
            rhdr.slot_idx   = hdr.slot_idx;

            send_frame(node, &rhdr, rbuf, rlen, tx_local);
            break;
        }

        case ON_FRAME_RTT_RESP: {
            if (hdr.dst_id != node->short_id || !node->rtt_in_flight) break;
            on_rtt_resp_t resp;
            if (on_rtt_resp_unpack(payload, plen, &resp) != ON_OK) break;
            if (resp.token != node->rtt_token) break;

            int64_t tau_ns = 0;
            if (on_sync_on_rtt(&node->sync, resp.t1_local_ns,
                               resp.t2_g_rx_ns, resp.t3_g_tx_ns,
                               rx_local_ns, &tau_ns) == ON_OK) {
                node->rtt_in_flight = false;
                node->rtt_next_sf   = node->current_sf_seq + 25u;
                emit_event(node, "RTT_CALIBRATED", node->short_id, tau_ns);
            }
            break;
        }

        case ON_FRAME_TDMA_DATA:
        case ON_FRAME_CRA_ALERT: {
            on_data_payload_t dp;
            if (on_data_payload_unpack(payload, plen, &dp) != ON_OK) break;

            if (node->role == ON_ROLE_G && hdr.dst_id == node->short_id) {
                node->tdma_rx_total++;
                if (hdr.type == ON_FRAME_TDMA_DATA) {
                    node->rx_slot_mask_this_sf |= (uint16_t)(1u << (hdr.slot_idx & 0x0Fu));
                }
                on_peer_entry_t *pe = find_peer_by_id(node, hdr.src_id);
                if (pe) {
                    pe->last_seen_sf = node->current_sf_seq;
                    pe->last_skew_ns = dp.skew_ns_est;
                    pe->rx_packets++;
                }
                if (hdr.type == ON_FRAME_CRA_ALERT) {
                    emit_event(node, "CRA_ALERT_RX", dp.origin_id, dp.data_len > 0 ? dp.data[0] : 0);
                }
            } else if (node->role == ON_ROLE_SUB_G && hdr.is_sub_g && hdr.dst_id == node->short_id) {
                /* Sub-G receives edge node telemetry inside carved Sub-G window -> enqueue in 4-slot FIFO */
                node->tdma_rx_total++;
                if (hdr.type == ON_FRAME_TDMA_DATA) {
                    node->sub_rx_slot_mask_this_sf |= (uint16_t)(1u << (hdr.slot_idx & 0x0Fu));
                }
                on_peer_entry_t *pe = find_peer_by_id(node, hdr.src_id);
                if (pe) {
                    pe->last_seen_sf = node->current_sf_seq;
                    pe->last_skew_ns = dp.skew_ns_est;
                    pe->rx_packets++;
                }
                on_data_payload_t rdp = dp;
                rdp.hop_count++;
                rdp.flags |= 0x02u; /* mark relayed */
                if (node->relay_count < ON_DEFAULT_SUBG_MAX_SLOTS) {
                    node->relay_q[node->relay_tail] = rdp;
                    node->relay_tail = (uint8_t)((node->relay_tail + 1u) % ON_DEFAULT_SUBG_MAX_SLOTS);
                    node->relay_count++;
                }
            } else if (node->short_id == node->backup_id && !hdr.is_sub_g) {
                /* Hot-standby Backup_ID passively mirrors active 1-hop peer table & capabilities */
                uint8_t peer_caps = (uint8_t)((dp.flags >> 6) & 0x03u);
                if (peer_caps == 0u) peer_caps = 0x01u;
                on_peer_entry_t *pe = find_peer_by_id(node, hdr.src_id);
                if (!pe) {
                    for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
                        if (!node->peers[i].active) {
                            node->peers[i].active       = true;
                            node->peers[i].short_id     = hdr.src_id;
                            node->peers[i].slot_idx     = hdr.slot_idx;
                            node->peers[i].is_sub_slot  = false;
                            node->peers[i].capabilities = peer_caps;
                            node->peers[i].last_seen_sf = node->current_sf_seq;
                            break;
                        }
                    }
                } else {
                    pe->capabilities = peer_caps;
                    pe->last_seen_sf = node->current_sf_seq;
                }
            }
            break;
        }

        default:
            break;
    }
}

void on_node_step(on_node_t *node, int64_t now_local_ns) {
    if (!node) return;
    if (node->last_step_local_ns > 0 && now_local_ns > node->last_step_local_ns) {
        node->total_time_ns += (uint64_t)(now_local_ns - node->last_step_local_ns);
    }
    node->last_step_local_ns = now_local_ns;

    const int64_t t_sf_ns = (int64_t)node->sf_cfg.t_sf_us * 1000LL;

    /* ---------------------------------------------------------------------- */
    /* Role 1: Primary G Coordinator                                          */
    /* ---------------------------------------------------------------------- */
    if (node->role == ON_ROLE_G) {
        int64_t now_net_ns = on_sync_local_to_net(&node->sync, now_local_ns);
        int64_t elapsed_ns = now_net_ns - node->current_sf_start_net_ns;

        /* Start of new superframe -> broadcast SYNC/MAP at offset 0 us */
        if (elapsed_ns >= t_sf_ns || (node->current_sf_seq == 0u && !node->sync_rx_this_sf)) {
            if (node->sync_rx_this_sf) {
                /* Snapshot Piggyback Group-ACK mask from completed superframe */
                node->last_ack_bitmap = node->rx_slot_mask_this_sf;
                node->rx_slot_mask_this_sf = 0u;

                /* Atomic AFH countdown progression (BLE LL_CHANNEL_MAP_IND Instant style) */
                if (node->has_pending_ch_map) {
                    if (node->afh_countdown > 1u) {
                        node->afh_countdown--;
                    } else {
                        on_afh_apply_map(&node->afh, node->pending_channel_map);
                        node->has_pending_ch_map = false;
                        node->afh_countdown      = 0u;
                    }
                }

                /* Evaluate completed channel health & update AFH blacklist when no switch is in flight */
                if (node->afh.enabled && !node->has_pending_ch_map) {
                    uint8_t prev_ch = node->afh.current_ch;
                    uint16_t old_map = node->afh.channel_map;
                    uint8_t expected = popcount16(node->tdma_bitmap);
                    if (expected > 0u) {
                        uint8_t received = popcount16(node->last_ack_bitmap);
                        bool ch_ok = (received * 2u >= expected);
                        on_afh_report_rx(&node->afh, prev_ch, ch_ok, node->current_sf_seq);
                    }
                    on_afh_tick(&node->afh, node->current_sf_seq + 1u);
                    if (node->afh.channel_map != old_map) {
                        /* Announce new map for 6 consecutive SFs so all 1-hop & 2-hop nodes hear it */
                        node->pending_channel_map = node->afh.channel_map;
                        node->has_pending_ch_map  = true;
                        node->afh_countdown       = 6u;
                        node->afh.channel_map     = old_map;
                        emit_event(node, "AFH_MAP_SCHEDULED", node->pending_channel_map,
                                   (int64_t)(node->current_sf_seq + 6u));
                    }
                }

                node->current_sf_seq++;
                node->current_sf_start_net_ns += t_sf_ns;
                node->act_cmd = (uint8_t)((node->current_sf_seq / 10u) & 0x01u); /* Toggle LED every 10 SF (40ms) */

                /* Atomic key epoch rotation at exact 0 us superframe boundary */
                if (node->pending_key_rotation && node->sec.enabled) {
                    node->pending_key_rotation = false;
                    uint8_t next_ep = (uint8_t)(node->sec.key_epoch + 1u);
                    on_sec_set_session(&node->sec, node->sec.session_salt, next_ep);
                    emit_event(node, "KEY_EPOCH_ROTATED", next_ep, (int64_t)node->current_sf_seq);
                }

                /* v3.0 Lease Expiry GC & automatic Sub-G / Backup_ID re-election */
                for (uint32_t i = 0u; i < ON_MAX_NODES_TABLE; ++i) {
                    if (node->peers[i].active && !node->peers[i].is_sub_slot &&
                        node->current_sf_seq > node->peers[i].last_seen_sf + ON_PEER_LEASE_TIMEOUT_SF) {
                        uint16_t dead_id = node->peers[i].short_id;
                        uint8_t  dead_slot = node->peers[i].slot_idx;
                        node->tdma_bitmap &= (uint16_t)(~(1u << dead_slot));
                        node->peers[i].active = false;
                        node->lease_reclaim_count++;
                        emit_event(node, "PEER_LEASE_EXPIRED", dead_id, dead_slot);

                        if (dead_id == node->backup_id) {
                            node->backup_id = select_next_backup_id(node);
                        }
                        if (dead_id == node->sub_g_id) {
                            node->sub_g_id = select_next_subg_id(node);
                            node->subg_failover_count++;
                            emit_event(node, "SUBG_REELECTED", node->sub_g_id, (int64_t)node->current_sf_seq);
                        }
                    }
                }
            }
            /* Tune to this superframe's TSCH channel before SYNC/MAP TX */
            if (node->afh.enabled) {
                tune_channel(node, on_afh_channel_for(&node->afh, node->current_sf_seq, 0u));
            }
            node->sync_rx_this_sf   = true;
            node->act_fired_this_sf = false;
            send_sync_map(node, false);
            elapsed_ns = now_net_ns - node->current_sf_start_net_ns;
        }

        /* Synchronized ACT/QUIET trigger */
        int64_t act_off_ns = (int64_t)node->sf_cfg.act_fire_off_us * 1000LL;
        if (!node->act_fired_this_sf && elapsed_ns >= act_off_ns) {
            node->act_fired_this_sf = true;
            node->act_fire_total++;
            node->last_act_local_ns = now_local_ns;
            if (node->hal.on_act_trigger) {
                node->hal.on_act_trigger(node->hal.user_ctx, node->current_sf_seq,
                                         node->act_cmd, now_local_ns);
            }
        }
        return;
    }

    /* ---------------------------------------------------------------------- */
    /* Role 2: Unjoined / T / Sub-G Terminal Nodes                            */
    /* ---------------------------------------------------------------------- */
    if (!node->sync.locked) {
        /* Listening for initial SYNC/MAP or SUB_SYNC (with v3.0 AFH channel scan) */
        if (node->hal.radio_set_rx) node->hal.radio_set_rx(node->hal.user_ctx, true);
        if (node->afh.enabled) {
            if (node->scan_last_hop_local_ns == 0) {
                node->scan_last_hop_local_ns = now_local_ns;
            } else if ((now_local_ns - node->scan_last_hop_local_ns) >= 16000000LL) {
                /* Dwell 16 ms (4 superframes) per channel then sweep across all 16 channels */
                node->scan_last_hop_local_ns = now_local_ns;
                node->scan_ch_idx = (uint8_t)((node->scan_ch_idx + 1u) % ON_AFH_NUM_CHANNELS);
                tune_channel(node, node->scan_ch_idx);
            }
        }
        return;
    }

    int64_t now_net_ns = on_sync_local_to_net(&node->sync, now_local_ns);
    int64_t elapsed_ns = now_net_ns - node->current_sf_start_net_ns;

    /*
     * ZERO-VOTE HOT-STANDBY FAILOVER (Backup_ID):
     * If primary G missed 3 consecutive SYNC frames (missed_sync_count >= 3),
     * then on the 4th frame at 0 us (elapsed_ns >= t_sf_ns), Backup_ID immediately
     * assumes Primary G role inheriting sf_seq, tdma_bitmap, sub_g_id, and locked NCO slope!
     */
    if (node->role == ON_ROLE_T &&
        !node->is_sub_node &&
        node->short_id != ON_SHORT_ID_UNASSIGNED &&
        node->short_id == node->backup_id &&
        node->missed_sync_count >= ON_BACKUP_MISS_THRESHOLD &&
        elapsed_ns >= t_sf_ns) {
        node->current_sf_seq++;
        node->current_sf_start_net_ns += t_sf_ns;
        node->role = ON_ROLE_G;
        node->failover_count++;
        /* Anchor G clock smoothly to our locked NCO network time */
        node->sync.ref_local_ns = now_local_ns;
        node->sync.ref_net_ns   = node->current_sf_start_net_ns;
        node->backup_id         = select_next_backup_id(node);
        node->sync_rx_this_sf   = true;
        node->act_fired_this_sf = false;
        node->act_cmd           = (uint8_t)((node->current_sf_seq / 10u) & 0x01u);

        send_sync_map(node, false);
        emit_event(node, "BACKUP_TAKEOVER", node->short_id, (int64_t)node->current_sf_seq);
        return;
    }

    /* Superframe rollover on missed SYNC / SUB_SYNC (280 us grace past expected beacon) */
    int64_t expected_next_sync_ns = t_sf_ns +
        (node->is_sub_node ? (int64_t)node->sf_cfg.sub_g_start_us * 1000LL : 0LL);
    if (elapsed_ns >= expected_next_sync_ns + 280000LL) {
        node->missed_sync_count++;
        on_sync_on_missed_beacon(&node->sync);
        node->current_sf_seq++;
        node->current_sf_start_net_ns += t_sf_ns;
        node->sync_rx_this_sf          = false;
        node->act_fired_this_sf        = false;
        node->tdma_tx_done_this_sf     = false;
        node->sub_sync_tx_done_this_sf = false;
        node->cra_tx_done_this_sf      = false;
        node->ch_pretuned_this_sf      = false;
        if (node->afh.enabled) {
            tune_channel(node, on_afh_channel_for(&node->afh, node->current_sf_seq, 0u));
        }

        /* v3.0 2-hop edge node self-healing: if Sub-G proxy dies, detach & re-join new Sub-G */
        if (node->is_sub_node && node->missed_sync_count >= ON_SUB_NODE_DETACH_MISS) {
            node->role              = ON_ROLE_UNJOINED;
            node->short_id          = ON_SHORT_ID_UNASSIGNED;
            node->sync.locked       = false;
            node->missed_sync_count = 0u;
            node->awaiting_ack      = false;
            node->harq_retry_pending = false;
            emit_event(node, "SUB_NODE_DETACHED", node->parent_id, (int64_t)node->current_sf_seq);
            return;
        }
        elapsed_ns = now_net_ns - node->current_sf_start_net_ns;
    }

    /* Phase 2a: Primary D-TDMA private slot TX (with Piggyback Group-ACK Fast HARQ) */
    if ((node->role == ON_ROLE_T || node->role == ON_ROLE_SUB_G) &&
        !node->is_sub_node && !node->tdma_tx_done_this_sf) {
        int64_t slot_ns = (int64_t)on_sf_slot_start_us(&node->sf_cfg, node->my_slot) * 1000LL;
        if (elapsed_ns >= slot_ns && elapsed_ns < slot_ns + 80000LL) {
            node->tdma_tx_done_this_sf = true;
            on_data_payload_t dp;
            if (node->harq_retry_pending) {
                dp = node->last_tx_data;
                dp.flags |= 0x04u; /* HARQ fast retransmission flag */
                node->last_tx_data = dp;
                node->harq_retry_pending = false;
                node->harq_retx_total++;
            } else if (node->role == ON_ROLE_SUB_G && node->relay_count > 0u) {
                dp = node->relay_q[node->relay_head];
                node->relay_head = (uint8_t)((node->relay_head + 1u) % ON_DEFAULT_SUBG_MAX_SLOTS);
                node->relay_count--;
                dp.flags = (uint8_t)((dp.flags & 0x3Fu) | ((node->capabilities & 0x03u) << 6));
                node->last_tx_data = dp;
            } else {
                memset(&dp, 0, sizeof(dp));
                dp.origin_id   = node->short_id;
                dp.seq         = node->data_tx_seq++;
                dp.skew_ns_est = (int16_t)(node->sync.last_err_ns > 32000 ? 32000 :
                                          (node->sync.last_err_ns < -32000 ? -32000 : node->sync.last_err_ns));
                dp.hop_count   = 1u;
                dp.flags       = (uint8_t)((node->capabilities & 0x03u) << 6);
                dp.data[0]     = (uint8_t)(node->current_sf_seq & 0xFFu);
                dp.data_len    = 1u;
                node->last_tx_data = dp;
            }
            node->awaiting_ack = true;
            uint8_t dbuf[ON_DATA_PAYLOAD_MIN_WIRE_LEN + 8u];
            size_t dlen = 0u;
            on_data_payload_pack(&dp, dbuf, sizeof(dbuf), &dlen);

            on_mac_hdr_t hdr;
            hdr.version    = ON_PROTO_VERSION;
            hdr.type       = (uint8_t)ON_FRAME_TDMA_DATA;
            hdr.sec_en     = false;
            hdr.is_sub_g   = false;
            hdr.src_id     = node->short_id;
            hdr.dst_id     = node->parent_id;
            hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
            hdr.slot_idx   = node->my_slot;

            send_frame(node, &hdr, dbuf, dlen, now_local_ns);
            node->tdma_tx_total++;
        }
    }

    /* Phase 2b: Sub-G Staggered Sub-Superframe (carved 2040..3160 us window) */
    if (node->role == ON_ROLE_SUB_G && !node->sub_sync_tx_done_this_sf) {
        int64_t sub_start_ns = (int64_t)on_sf_subg_sync_start_us(&node->sf_cfg) * 1000LL;
        if (elapsed_ns >= sub_start_ns && elapsed_ns < sub_start_ns + 80000LL) {
            node->sub_sync_tx_done_this_sf = true;
            node->last_sub_ack_bitmap      = node->sub_rx_slot_mask_this_sf;
            node->sub_rx_slot_mask_this_sf = 0u;

            /* Sub-G Lease Expiry GC for 2-hop child edge slots */
            for (uint32_t i = 0u; i < ON_MAX_NODES_TABLE; ++i) {
                if (node->peers[i].active && node->peers[i].is_sub_slot &&
                    node->current_sf_seq > node->peers[i].last_seen_sf + ON_PEER_LEASE_TIMEOUT_SF) {
                    uint8_t dead_slot = node->peers[i].slot_idx;
                    node->sub_tdma_bitmap &= (uint16_t)(~(1u << dead_slot));
                    node->peers[i].active = false;
                    node->lease_reclaim_count++;
                    emit_event(node, "SUB_PEER_LEASE_EXPIRED", node->peers[i].short_id, dead_slot);
                }
            }
            send_sync_map(node, true);
        }
    }

    /* Phase 2c: 2-hop Edge Node private Sub-TDMA TX inside Sub-G window (with Fast HARQ) */
    if (node->role == ON_ROLE_T && node->is_sub_node && !node->tdma_tx_done_this_sf) {
        int64_t sub_slot_ns = (int64_t)on_sf_subg_slot_start_us(&node->sf_cfg, node->my_slot) * 1000LL;
        if (elapsed_ns >= sub_slot_ns && elapsed_ns < sub_slot_ns + 80000LL) {
            node->tdma_tx_done_this_sf = true;
            on_data_payload_t dp;
            if (node->harq_retry_pending) {
                dp = node->last_tx_data;
                dp.flags |= 0x04u;
                node->last_tx_data = dp;
                node->harq_retry_pending = false;
                node->harq_retx_total++;
            } else {
                memset(&dp, 0, sizeof(dp));
                dp.origin_id   = node->short_id;
                dp.seq         = node->data_tx_seq++;
                dp.skew_ns_est = (int16_t)(node->sync.last_err_ns > 32000 ? 32000 :
                                          (node->sync.last_err_ns < -32000 ? -32000 : node->sync.last_err_ns));
                dp.hop_count   = 1u;
                dp.data[0]     = 0xE8u; /* edge sensor marker */
                dp.data_len    = 1u;
                node->last_tx_data = dp;
            }
            node->awaiting_ack = true;

            uint8_t dbuf[ON_DATA_PAYLOAD_MIN_WIRE_LEN + 8u];
            size_t dlen = 0u;
            on_data_payload_pack(&dp, dbuf, sizeof(dbuf), &dlen);

            on_mac_hdr_t hdr;
            hdr.version    = ON_PROTO_VERSION;
            hdr.type       = (uint8_t)ON_FRAME_TDMA_DATA;
            hdr.sec_en     = false;
            hdr.is_sub_g   = true;
            hdr.src_id     = node->short_id;
            hdr.dst_id     = node->parent_id;
            hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
            hdr.slot_idx   = node->my_slot;

            send_frame(node, &hdr, dbuf, dlen, now_local_ns);
            node->tdma_tx_total++;
        }
    }

    /* Phase 3: C-RA Contention Micro-Slot (JOIN_REQ, RTT_REQ, or Emergency Alert) */
    if (!node->cra_tx_done_this_sf) {
        /* Deterministic stagger by MAC byte[5] to minimize C-RA join collisions */
        uint32_t my_turn = (node->mac48[5] % 7u);
        int64_t cra_ns = node->is_sub_node
            ? (int64_t)on_sf_subg_cra_start_us(&node->sf_cfg) * 1000LL
            : (int64_t)on_sf_cra_slot_start_us(&node->sf_cfg, (uint8_t)(node->mac48[5] & 1u)) * 1000LL;

        if (elapsed_ns >= cra_ns && elapsed_ns < cra_ns + 60000LL) {
            if (node->role == ON_ROLE_UNJOINED && ((node->current_sf_seq % 7u) == my_turn)) {
                node->cra_tx_done_this_sf = true;
                on_join_req_t req;
                memcpy(req.mac48, node->mac48, 6u);
                req.capabilities = node->capabilities;
                req.hop_hint     = node->is_sub_node ? 1u : 0u;

                uint8_t jbuf[ON_JOIN_REQ_WIRE_LEN];
                size_t jlen = 0u;
                on_join_req_pack(&req, jbuf, sizeof(jbuf), &jlen);

                on_mac_hdr_t hdr;
                hdr.version    = ON_PROTO_VERSION;
                hdr.type       = (uint8_t)ON_FRAME_JOIN_REQ;
                hdr.sec_en     = false;
                hdr.is_sub_g   = node->is_sub_node;
                hdr.src_id     = ON_SHORT_ID_UNASSIGNED;
                hdr.dst_id     = node->parent_id;
                hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                hdr.slot_idx   = 0u;

                send_frame(node, &hdr, jbuf, jlen, now_local_ns);
            } else if ((node->role == ON_ROLE_T || node->role == ON_ROLE_SUB_G) &&
                       node->pending_alert) {
                node->cra_tx_done_this_sf = true;
                node->pending_alert = false;

                on_data_payload_t dp;
                memset(&dp, 0, sizeof(dp));
                dp.origin_id = node->short_id;
                dp.seq       = node->data_tx_seq++;
                dp.flags     = 0x01u; /* Emergency alert */
                dp.data[0]   = node->alert_code;
                dp.data_len  = 1u;

                uint8_t abuf[ON_DATA_PAYLOAD_MIN_WIRE_LEN + 8u];
                size_t alen = 0u;
                on_data_payload_pack(&dp, abuf, sizeof(abuf), &alen);

                on_mac_hdr_t hdr;
                hdr.version    = ON_PROTO_VERSION;
                hdr.type       = (uint8_t)ON_FRAME_CRA_ALERT;
                hdr.sec_en     = false;
                hdr.is_sub_g   = node->is_sub_node;
                hdr.src_id     = node->short_id;
                hdr.dst_id     = node->parent_id;
                hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                hdr.slot_idx   = 0u;

                send_frame(node, &hdr, abuf, alen, now_local_ns);
            } else if ((node->role == ON_ROLE_T || node->role == ON_ROLE_SUB_G) &&
                       node->current_sf_seq >= node->rtt_next_sf &&
                       ((node->current_sf_seq % 7u) == my_turn)) {
                /* Periodic 4-timestamp RTT calibration */
                node->cra_tx_done_this_sf = true;
                node->rtt_in_flight   = true;
                node->rtt_t1_local_ns = now_local_ns;
                node->rtt_token++;

                on_rtt_req_t req;
                req.t1_local_ns = now_local_ns;
                req.token       = node->rtt_token;

                uint8_t rbuf[ON_RTT_REQ_WIRE_LEN];
                size_t rlen = 0u;
                on_rtt_req_pack(&req, rbuf, sizeof(rbuf), &rlen);

                on_mac_hdr_t hdr;
                hdr.version    = ON_PROTO_VERSION;
                hdr.type       = (uint8_t)ON_FRAME_RTT_REQ;
                hdr.sec_en     = false;
                hdr.is_sub_g   = node->is_sub_node;
                hdr.src_id     = node->short_id;
                hdr.dst_id     = node->parent_id;
                hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                hdr.slot_idx   = 0u;

                send_frame(node, &hdr, rbuf, rlen, now_local_ns);
            }
        }
    }

    /* Phase 4: ACT / QUIET Silent Guard Band -> Synchronized Hardware Action Trigger */
    int64_t act_off_ns = (int64_t)node->sf_cfg.act_fire_off_us * 1000LL;
    if (!node->act_fired_this_sf && elapsed_ns >= act_off_ns) {
        node->act_fired_this_sf = true;
        node->act_fire_total++;
        node->last_act_local_ns = now_local_ns;
        if (node->hal.on_act_trigger) {
            node->hal.on_act_trigger(node->hal.user_ctx, node->current_sf_seq,
                                     node->act_cmd, now_local_ns);
        }
    }

    /* End of ACT/QUIET: advance AFH countdown & pre-tune radio for next superframe's TSCH channel */
    if (node->afh.enabled && !node->ch_pretuned_this_sf && elapsed_ns >= t_sf_ns - 120000LL) {
        node->ch_pretuned_this_sf = true;
        if (node->has_pending_ch_map) {
            if (node->afh_countdown > 1u) {
                node->afh_countdown--;
            } else {
                on_afh_apply_map(&node->afh, node->pending_channel_map);
                node->has_pending_ch_map = false;
                node->afh_countdown      = 0u;
            }
        }
        tune_channel(node, on_afh_channel_for(&node->afh, node->current_sf_seq + 1u, 0u));
    }
}

int64_t on_node_next_wakeup_local_ns(const on_node_t *node, int64_t now_local_ns) {
    if (!node || !node->sync.locked) {
        return now_local_ns + 50000LL; /* 50 us poll while acquiring */
    }
    int64_t now_net_ns = on_sync_local_to_net(&node->sync, now_local_ns);
    int64_t sf0        = node->current_sf_start_net_ns;
    int64_t t_sf_ns    = (int64_t)node->sf_cfg.t_sf_us * 1000LL;

    int64_t candidates[6];
    int n = 0;

    if (node->role == ON_ROLE_G) {
        candidates[n++] = sf0 + (int64_t)node->sf_cfg.act_fire_off_us * 1000LL;
        candidates[n++] = sf0 + t_sf_ns;
    } else {
        if (!node->is_sub_node && node->role != ON_ROLE_UNJOINED) {
            candidates[n++] = sf0 + (int64_t)on_sf_slot_start_us(&node->sf_cfg, node->my_slot) * 1000LL;
        }
        if (node->role == ON_ROLE_SUB_G) {
            candidates[n++] = sf0 + (int64_t)on_sf_subg_sync_start_us(&node->sf_cfg) * 1000LL;
        }
        if (node->is_sub_node && node->role == ON_ROLE_T) {
            candidates[n++] = sf0 + (int64_t)on_sf_subg_slot_start_us(&node->sf_cfg, node->my_slot) * 1000LL;
        }
        int64_t cra_off = node->is_sub_node
            ? (int64_t)on_sf_subg_cra_start_us(&node->sf_cfg) * 1000LL
            : (int64_t)on_sf_cra_slot_start_us(&node->sf_cfg, (uint8_t)(node->mac48[5] & 1u)) * 1000LL;
        candidates[n++] = sf0 + cra_off;
        candidates[n++] = sf0 + (int64_t)node->sf_cfg.act_fire_off_us * 1000LL;
        candidates[n++] = sf0 + t_sf_ns + (node->is_sub_node ? (int64_t)node->sf_cfg.sub_g_start_us * 1000LL : 0LL);
    }

    int64_t best_net = sf0 + t_sf_ns + 160000LL;
    for (int i = 0; i < n; ++i) {
        if (candidates[i] > now_net_ns && candidates[i] < best_net) {
            best_net = candidates[i];
        }
    }
    return on_sync_net_to_local(&node->sync, best_net);
}

bool on_node_is_rx_open(const on_node_t *node, int64_t now_local_ns) {
    if (!node) return false;
    /* Primary G and unjoined nodes keep RX open for slots/acquisition */
    if (node->role == ON_ROLE_G || node->role == ON_ROLE_UNJOINED || !node->sync.locked) {
        return true;
    }
    /* Waiting for RTT_RESP */
    if (node->rtt_in_flight) {
        return true;
    }

    int64_t now_net_ns = on_sync_local_to_net(&node->sync, now_local_ns);
    int64_t elapsed_ns = now_net_ns - node->current_sf_start_net_ns;
    int64_t t_sf_ns    = (int64_t)node->sf_cfg.t_sf_us * 1000LL;
    /* Wrap elapsed_ns into [0, t_sf_ns) for window check */
    while (elapsed_ns >= t_sf_ns) elapsed_ns -= t_sf_ns;
    while (elapsed_ns < 0)        elapsed_ns += t_sf_ns;
    uint32_t off_us = (uint32_t)(elapsed_ns / 1000LL);

    if (node->is_sub_node) {
        /* 2-hop edge node ONLY wakes RX during Sub-G SYNC window (2040..2310 us) */
        uint32_t sub0 = node->sf_cfg.sub_g_start_us;
        return (off_us >= sub0 && off_us < sub0 + 270u);
    }

    /* 1-hop T node & Sub-G wake RX during primary SYNC/MAP (0..270 us) */
    if (off_us < 270u || off_us >= (uint32_t)(node->sf_cfg.t_sf_us - 40u)) {
        return true;
    }

    /* Hot-standby Backup_ID periodically snoops 1-hop D-TDMA window (every 8th SF) to mirror peer table */
    if (node->short_id != ON_SHORT_ID_UNASSIGNED &&
        node->short_id == node->backup_id &&
        (node->current_sf_seq & 0x07u) == 0u &&
        off_us < node->sf_cfg.sub_g_start_us) {
        return true;
    }

    /* Sub-G proxy also keeps RX open during its carved Sub-G child uplink/CRA window */
    if (node->role == ON_ROLE_SUB_G) {
        uint32_t sub0 = node->sf_cfg.sub_g_start_us;
        if (off_us >= sub0 + ON_DEFAULT_SUBG_SYNC_US &&
            off_us < sub0 + node->sf_cfg.sub_g_len_us) {
            return true;
        }
    }

    /* Otherwise radio is powered OFF in TDMA deep sleep (>95% of superframe!) */
    return false;
}

double on_node_duty_cycle_pct(const on_node_t *node) {
    if (!node || node->total_time_ns == 0u) return 0.0;
    return 100.0 * ((double)node->radio_active_ns / (double)node->total_time_ns);
}
