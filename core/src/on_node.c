#include "on_node.h"
#include <stdio.h>
#include <string.h>

#define ON_ACT_DEMO_PERIOD_SF 10u

static uint8_t demo_act_level(uint32_t sf) {
    return (uint8_t)((sf / ON_ACT_DEMO_PERIOD_SF) & 1u);
}

static uint8_t demo_act_flip_lead(uint32_t sf) {
    uint32_t lead = ON_ACT_DEMO_PERIOD_SF - sf % ON_ACT_DEMO_PERIOD_SF;
    /* This demo's last block before uint32 wrap is shorter. */
    if (lead > UINT32_MAX - sf) lead = UINT32_MAX - sf + 1u;
    return lead <= 7u ? (uint8_t)lead : 0u;
}

static bool sf_after(uint32_t a, uint32_t b) {
    return a != b && a - b <= UINT32_MAX / 2u;
}

#define ON_ACT_CLOCK_BLOCK_NS 160000000LL
#define ON_ACT_CLOCK_WINDOW_NS (ON_ACT_CLOCK_BLOCK_NS * ON_ACT_CLOCK_BLOCKS)
#define ON_ACT_CLOCK_MAX_PPM 200

static int64_t act_clock_offset_at(const on_act_clock_sample_t *s, int64_t net_ns, int32_t rate_ppm) {
    return s->offset_ns + (net_ns - s->net_ns) * rate_ppm / 1000000LL;
}

static const on_act_clock_sample_t *act_clock_floor(const on_act_clock_t *clock,
                                                   int64_t from_block, int64_t to_block,
                                                   int64_t at_net_ns) {
    const on_act_clock_sample_t *first = NULL, *second = NULL;
    int64_t lo = 0, next_lo = 0;
    unsigned count = 0;
    for (unsigned i = 0; i < ON_ACT_CLOCK_BLOCKS; ++i) {
        const on_act_clock_sample_t *s = &clock->blocks[i];
        if (!s->valid) continue;
        int64_t b = s->net_ns / ON_ACT_CLOCK_BLOCK_NS;
        if (b < from_block || b > to_block) continue;
        int64_t x = act_clock_offset_at(s, at_net_ns, clock->rate_ppm);
        ++count;
        if (!first || x < lo) {
            second = first;
            next_lo = lo;
            first = s;
            lo = x;
        } else if (!second || x < next_lo) {
            second = s;
            next_lo = x;
        }
    }
    /* Use a repeated low observation rather than trusting one anomalous minimum. */
    return count >= 3u ? second : first;
}

static void act_clock_observe(on_act_clock_t *clock, int64_t net_ns, int64_t rx_ns) {
    if (net_ns < 0 || rx_ns < 0 || (clock->valid && net_ns <= clock->last_net_ns)) return;
    int64_t block = net_ns / ON_ACT_CLOCK_BLOCK_NS;
    int64_t old_block = clock->last_net_ns / ON_ACT_CLOCK_BLOCK_NS;
    if (clock->valid && net_ns - clock->last_net_ns >= ON_ACT_CLOCK_WINDOW_NS) {
        memset(clock, 0, sizeof(*clock));
    }
    if (clock->valid && block != old_block) {
        /* Compare the low samples of two non-overlapping groups of completed
         * blocks. Large delay-floor changes are not credible crystal drift. */
        const on_act_clock_sample_t *older = act_clock_floor(clock, block - 8, block - 5, net_ns);
        const on_act_clock_sample_t *recent = act_clock_floor(clock, block - 4, block - 1, net_ns);
        if (older && recent && recent->net_ns - older->net_ns >= 2 * ON_ACT_CLOCK_BLOCK_NS) {
            int64_t dt = recent->net_ns - older->net_ns;
            int64_t delta = recent->offset_ns - older->offset_ns;
            if (delta >= -dt / 1000 && delta <= dt / 1000) {
                int32_t ppm = (int32_t)(delta * 1000000LL / dt);
                if (ppm < -ON_ACT_CLOCK_MAX_PPM) ppm = -ON_ACT_CLOCK_MAX_PPM;
                if (ppm >  ON_ACT_CLOCK_MAX_PPM) ppm =  ON_ACT_CLOCK_MAX_PPM;
                clock->rate_ppm = (3 * clock->rate_ppm + ppm) / 4;
            }
        }
    }
    on_act_clock_sample_t *s = &clock->blocks[block % ON_ACT_CLOCK_BLOCKS];
    const on_act_clock_sample_t sample = {true, net_ns, rx_ns - net_ns};
    if (!s->valid || s->net_ns / ON_ACT_CLOCK_BLOCK_NS != block ||
        sample.offset_ns < act_clock_offset_at(s, net_ns, clock->rate_ppm)) {
        *s = sample;
    }
    clock->valid = true;
    clock->last_net_ns = net_ns;
}

static int64_t act_clock_deadline(const on_node_t *node, int64_t target_net_ns) {
    const on_act_clock_t *clock = &node->act_clock;
    if (node->act_clock_enabled && clock->valid) {
        int64_t block = clock->last_net_ns / ON_ACT_CLOCK_BLOCK_NS;
        const on_act_clock_sample_t *floor = act_clock_floor(clock, block - (ON_ACT_CLOCK_BLOCKS - 1),
                                                             block, target_net_ns);
        if (floor) {
            return target_net_ns + act_clock_offset_at(floor, target_net_ns, clock->rate_ppm)
                   - (int64_t)node->act_clock_bias_us * 1000LL;
        }
    }
    return on_sync_net_to_local(&node->sync, target_net_ns);
}

static void reset_act_lookahead(on_node_t *node) {
    memset(&node->act_lookahead, 0, sizeof(node->act_lookahead));
    memset(&node->act_clock, 0, sizeof(node->act_clock));
}

static void receive_act_lookahead(on_node_t *node, const on_sync_payload_t *sp, int64_t act_rx_local_ns) {
    if (node->act_lookahead.has_beacon && !sf_after(sp->sf_seq, node->act_lookahead.beacon_sf)) return;
    if (node->act_clock_enabled) act_clock_observe(&node->act_clock, sp->t_tx_net_ns, act_rx_local_ns);
    node->act_lookahead.beacon_sf = sp->sf_seq;
    node->act_lookahead.session_salt = sp->session_salt;
    if (!node->act_lookahead.has_beacon || !sf_after(node->act_lookahead.level_sf, sp->sf_seq)) {
        const bool reached_target = node->act_lookahead.pending &&
            !sf_after(node->act_lookahead.target_sf, sp->sf_seq);
        /* Preserve the local deadline when the new level confirms the plan.
         * A conflicting current level (e.g. after a long outage) supersedes it. */
        if (!reached_target || node->act_lookahead.next_level != sp->act_cmd) {
            if (reached_target) node->act_lookahead.pending = false;
            node->act_cmd = sp->act_cmd;
            node->act_lookahead.level_sf = sp->sf_seq;
        }
    }
    node->act_lookahead.has_beacon = true;
    if (!sp->act_flip_lead_sf || sp->act_cmd > 1u || !sp->t_sf_us || sp->act_fire_off_us >= sp->t_sf_us) return;
    uint32_t target = sp->sf_seq + sp->act_flip_lead_sf;
    if (!sf_after(target, node->act_lookahead.level_sf)) return;
    int64_t target_net_ns = sp->t_tx_net_ns +
        ((int64_t)sp->act_flip_lead_sf * sp->t_sf_us + sp->act_fire_off_us) * 1000LL;
    int64_t deadline = act_clock_deadline(node, target_net_ns);
    /* Pick the earliest projection across repeated announcements. A delayed
     * beacon cannot postpone an armed deadline; the TDMA clock is unchanged. */
    if (!node->act_lookahead.pending || target != node->act_lookahead.target_sf ||
        deadline < node->act_lookahead.deadline_local_ns) {
        node->act_lookahead.pending = true;
        node->act_lookahead.target_sf = target;
        node->act_lookahead.deadline_local_ns = deadline;
        node->act_lookahead.next_level = sp->act_cmd ^ 1u;
    }
}

static void fire_act(on_node_t *node, uint32_t sf, int64_t now_local_ns, bool scheduled) {
    node->act_fire_total++;
    node->last_act_local_ns = now_local_ns;
    node->last_act_was_scheduled = scheduled;
    if (scheduled) node->last_act_deadline_local_ns = node->act_lookahead.deadline_local_ns;
    if (node->hal.on_act_trigger) {
        node->hal.on_act_trigger(node->hal.user_ctx, sf, node->act_cmd, now_local_ns);
    }
}

static void emit_event(on_node_t *node, const char *name, uint32_t a1, int64_t a2) {
    if (node && node->hal.on_event) {
        node->hal.on_event(node->hal.user_ctx, name, a1, a2);
    }
}

static void sync_mesh_identity(on_node_t *node) {
    if (!node || !node->mesh) return;
    node->mesh->my_id = node->short_id & ON_SHORT_ID_MASK;
    if (node->role == ON_ROLE_G) {
        node->mesh->my_hop_depth = 0u;
        node->mesh->parent_id    = ON_SHORT_ID_UNASSIGNED;
        node->mesh->root_id      = node->short_id & ON_SHORT_ID_MASK;
    } else if (node->role == ON_ROLE_UNJOINED) {
        node->mesh->my_hop_depth = 0xFFu;
        node->mesh->parent_id    = ON_SHORT_ID_UNASSIGNED;
    } else if (node->mesh->my_hop_depth == 0xFFu || node->mesh->my_hop_depth <= 2u) {
        node->mesh->my_hop_depth = node->is_sub_node ? 2u : 1u;
        node->mesh->parent_id    = node->parent_id & ON_SHORT_ID_MASK;
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

/* ========================================================================== */
/* v5.1 Cluster-Wide Topology & Live Node Data Registry Helpers               */
/* ========================================================================== */

static on_cluster_node_t *cluster_find_or_alloc(on_node_t *node, uint16_t short_id) {
    if (!node || short_id == ON_SHORT_ID_UNASSIGNED) return NULL;
    on_cluster_node_t *empty = NULL;
    for (uint32_t i = 0u; i < ON_MAX_CLUSTER_NODES; ++i) {
        if (node->cluster_tbl[i].active && node->cluster_tbl[i].short_id == short_id) {
            return &node->cluster_tbl[i];
        }
        if (!node->cluster_tbl[i].active && !empty) {
            empty = &node->cluster_tbl[i];
        }
    }
    if (empty) {
        memset(empty, 0, sizeof(*empty));
        empty->active   = true;
        empty->short_id = short_id & ON_SHORT_ID_MASK;
    }
    return empty;
}

static void cluster_mark_inactive(on_node_t *node, uint16_t short_id) {
    if (!node) return;
    for (uint32_t i = 0u; i < ON_MAX_CLUSTER_NODES; ++i) {
        if (node->cluster_tbl[i].active && node->cluster_tbl[i].short_id == short_id) {
            node->cluster_tbl[i].active = false;
            break;
        }
    }
}

static bool mac48_nonzero(const uint8_t m[6]) {
    return (m[0] | m[1] | m[2] | m[3] | m[4] | m[5]) != 0u;
}

static void format_mac48(const uint8_t m[6], char out[18]) {
    snprintf(out, 18u, "%02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned)m[0], (unsigned)m[1], (unsigned)m[2],
             (unsigned)m[3], (unsigned)m[4], (unsigned)m[5]);
}

static void cluster_record_tdma(on_node_t *node, const on_mac_hdr_t *hdr,
                                const on_data_payload_t *dp, const on_peer_entry_t *pe) {
    if (!node || !hdr || !dp) return;
    /* 1. Always refresh the direct 1-hop sender */
    on_cluster_node_t *c1 = cluster_find_or_alloc(node, hdr->src_id);
    if (c1) {
        if (pe && mac48_nonzero(pe->mac48)) memcpy(c1->mac48, pe->mac48, 6u);
        c1->parent_id    = node->short_id;
        c1->hop_depth    = 1u;
        c1->role         = (hdr->src_id == node->sub_g_id || hdr->src_id == node->sub_g_id_2)
                           ? (uint8_t)ON_ROLE_SUB_G : (uint8_t)ON_ROLE_T;
        c1->slot_idx     = pe ? pe->slot_idx : hdr->slot_idx;
        c1->capabilities = pe ? pe->capabilities : (uint8_t)((dp->flags >> 6) & 0x03u);
        c1->is_backup    = (hdr->src_id == node->backup_id);
        c1->last_seen_sf = node->current_sf_seq;
        c1->rx_packets++;
        if (dp->origin_id == hdr->src_id && (dp->flags & 0x02u) == 0u) {
            c1->last_skew_ns  = dp->skew_ns_est;
            c1->last_seq      = dp->seq;
            c1->last_data_len = (dp->data_len <= 8u) ? dp->data_len : 8u;
            if (c1->last_data_len > 0u) memcpy(c1->last_data, dp->data, c1->last_data_len);
        }
    }
    /* 2. If relayed from a 2-hop wall-blocked edge node via Sub-G, track it too! */
    if ((dp->flags & 0x02u) != 0u || (dp->origin_id != ON_SHORT_ID_UNASSIGNED && dp->origin_id != hdr->src_id)) {
        on_cluster_node_t *c2 = cluster_find_or_alloc(node, dp->origin_id);
        if (c2) {
            c2->parent_id     = hdr->src_id; /* Sub-G proxy ID */
            c2->hop_depth     = (dp->hop_count >= 2u) ? dp->hop_count : 2u;
            c2->role          = (uint8_t)ON_ROLE_T;
            c2->slot_idx      = (uint8_t)(dp->origin_id & 0x0Fu);
            c2->is_backup     = false;
            c2->last_seen_sf  = node->current_sf_seq;
            c2->last_skew_ns  = dp->skew_ns_est;
            c2->rx_packets++;
            c2->last_seq      = dp->seq;
            c2->last_data_len = (dp->data_len <= 8u) ? dp->data_len : 8u;
            if (c2->last_data_len > 0u) memcpy(c2->last_data, dp->data, c2->last_data_len);
        }
    }
}

static void cluster_record_ota(on_node_t *node, const on_ota_status_t *st) {
    if (!node || !st) return;
    on_cluster_node_t *c = cluster_find_or_alloc(node, st->node_id);
    if (c) {
        c->ota_state     = st->state;
        c->ota_rx_chunks = st->rx_chunks;
        c->last_seen_sf  = node->current_sf_seq;
    }
}

static void topo_put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}
static uint16_t topo_get_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void topo_queue_resp_pages(on_node_t *node, uint16_t requester_id) {
    if (!node || node->role != ON_ROLE_G) return;
    on_cluster_snapshot_t snap;
    size_t n = on_node_get_cluster_snapshot(node, &snap);
    uint8_t total_pages = (uint8_t)((n + ON_TOPO_PAGE_NODES - 1u) / ON_TOPO_PAGE_NODES);
    if (total_pages == 0u) total_pages = 1u;

    for (uint8_t p = 0u; p < total_pages; ++p) {
        on_gw_bridge_msg_t resp;
        memset(&resp, 0, sizeof(resp));
        resp.bridge_type = (uint8_t)ON_GW_MSG_TOPO_RESP;
        resp.origin_id   = requester_id & ON_SHORT_ID_MASK;
        resp.seq         = ++node->data_tx_seq;

        uint8_t start = (uint8_t)(p * ON_TOPO_PAGE_NODES);
        uint8_t cnt   = (start < (uint8_t)n) ? (uint8_t)(n - start) : 0u;
        if (cnt > ON_TOPO_PAGE_NODES) cnt = ON_TOPO_PAGE_NODES;

        uint8_t *b = resp.payload;
        b[0] = p;
        b[1] = total_pages;
        b[2] = (uint8_t)n;
        b[3] = cnt;
        topo_put_u16(b + 4, node->short_id);

        size_t off = 6u;
        for (uint8_t k = 0u; k < cnt; ++k) {
            const on_cluster_node_t *cn = &snap.nodes[start + k];
            uint16_t sid = cn->short_id & ON_SHORT_ID_MASK;
            uint16_t pid = cn->parent_id & ON_SHORT_ID_MASK;
            b[off + 0] = (uint8_t)(sid >> 4);
            b[off + 1] = (uint8_t)(((sid & 0x0Fu) << 4) | ((pid >> 8) & 0x0Fu));
            b[off + 2] = (uint8_t)(pid & 0xFFu);
            b[off + 3] = (uint8_t)(((cn->hop_depth & 0x03u) << 6) |
                                   ((cn->role & 0x03u) << 4) |
                                   (cn->slot_idx & 0x0Fu));
            uint8_t dlen = (cn->last_data_len <= 4u) ? cn->last_data_len : 4u;
            /* Bit 3 carries hop_depth bit2 (supporting 0..7 hops without growing wire record) */
            b[off + 4] = (uint8_t)(((cn->ota_state & 0x0Fu) << 4) |
                                   ((cn->hop_depth & 0x04u) << 1) |
                                   (dlen & 0x07u));
            topo_put_u16(b + off + 5, (uint16_t)cn->last_skew_ns);
            b[off + 7] = (uint8_t)(cn->last_seq & 0xFFu);
            memset(b + off + 8, 0, 4u);
            if (dlen > 0u) memcpy(b + off + 8, cn->last_data, dlen);
            memcpy(b + off + 12, cn->mac48, 6u);
            off += 18u;
        }
        resp.payload_len = (uint8_t)off;

        uint8_t nt = (uint8_t)((node->topo_resp_q.tail + 1u) & (ON_GW_RING_DEPTH - 1u));
        if (nt != node->topo_resp_q.head) {
            node->topo_resp_q.items[node->topo_resp_q.tail] = resp;
            node->topo_resp_q.tail = nt;
        }
    }
}

static void topo_ingest_resp_page(on_node_t *node, const on_gw_bridge_msg_t *msg) {
    if (!node || !msg || msg->payload_len < 6u) return;
    const uint8_t *b = msg->payload;
    uint8_t page_idx    = b[0];
    uint8_t total_pages = b[1];
    uint8_t cnt         = b[3];
    uint16_t g_id       = topo_get_u16(b + 4) & ON_SHORT_ID_MASK;
    if (total_pages == 0u || total_pages > 12u || page_idx >= total_pages) return;
    if ((size_t)msg->payload_len < 6u + (size_t)cnt * 18u) return;

    if (node->topo_rx_pages_mask == 0u) {
        memset(node->cluster_tbl, 0, sizeof(node->cluster_tbl));
    }
    node->cluster_g_id        = g_id;
    node->topo_rx_total_pages = total_pages;

    size_t off = 6u;
    for (uint8_t k = 0u; k < cnt; ++k) {
        uint16_t sid = (uint16_t)(((uint16_t)b[off + 0] << 4) | ((b[off + 1] >> 4) & 0x0Fu));
        uint16_t pid = (uint16_t)(((uint16_t)(b[off + 1] & 0x0Fu) << 8) | b[off + 2]);
        on_cluster_node_t *cn = cluster_find_or_alloc(node, sid);
        if (cn) {
            cn->parent_id     = pid & ON_SHORT_ID_MASK;
            uint8_t packed    = b[off + 3];
            cn->hop_depth     = (uint8_t)(((packed >> 6) & 0x03u) | ((b[off + 4] & 0x08u) >> 1));
            cn->role          = (uint8_t)((packed >> 4) & 0x03u);
            cn->slot_idx      = (uint8_t)(packed & 0x0Fu);
            cn->ota_state     = (uint8_t)((b[off + 4] >> 4) & 0x0Fu);
            cn->last_data_len = (uint8_t)(b[off + 4] & 0x07u);
            if (cn->last_data_len > 4u) cn->last_data_len = 4u;
            cn->last_skew_ns  = (int16_t)topo_get_u16(b + off + 5);
            cn->last_seq      = b[off + 7];
            cn->is_backup     = (sid == node->backup_id);
            cn->last_seen_sf  = node->current_sf_seq;
            if (cn->last_data_len > 0u) {
                memcpy(cn->last_data, b + off + 8, cn->last_data_len);
            }
            memcpy(cn->mac48, b + off + 12, 6u);
        }
        off += 18u;
    }

    node->topo_rx_pages_mask |= (uint16_t)(1u << page_idx);
    uint16_t expect_mask = (uint16_t)((1u << total_pages) - 1u);
    if ((node->topo_rx_pages_mask & expect_mask) == expect_mask) {
        node->cluster_snapshot_valid = true;
        node->topo_query_in_flight   = false;
        emit_event(node, "CLUSTER_SNAPSHOT_RX", g_id, (int64_t)b[2]);
    }
}

static uint16_t select_next_backup_id(const on_node_t *node) {
    /* First priority: 1-hop peer with Wi-Fi/MQTT Gateway capability (bit 2 = 0x04) + Backup (bit 0 = 0x01) */
    for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
        if (node->peers[i].active &&
            !node->peers[i].is_sub_slot &&
            node->peers[i].short_id != node->short_id &&
            node->peers[i].short_id != node->sub_g_id &&
            (node->peers[i].capabilities & 0x05u) == 0x05u) {
            return node->peers[i].short_id;
        }
    }
    /* Second priority: any 1-hop peer with Backup capability (bit 0 = 0x01) */
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
    return (type == ON_FRAME_TDMA_DATA  ||
            type == ON_FRAME_CRA_ALERT  ||
            type == ON_FRAME_RTT_REQ    ||
            type == ON_FRAME_RTT_RESP   ||
            type == ON_FRAME_JOIN_REQ   ||
            type == ON_FRAME_JOIN_ACK   ||
            type == ON_FRAME_OTA_META   ||
            type == ON_FRAME_OTA_CHUNK  ||
            type == ON_FRAME_OTA_STATUS ||
            type == ON_FRAME_OTA_COMMIT ||
            type == ON_FRAME_GW_BRIDGE  ||
            type == ON_FRAME_MESH);
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
    sp.sub_g_id_2         = node->sub_g_id_2;
    sp.sub_g_win_start_us = node->sf_cfg.sub_g_start_us;
    sp.sub_g_win_len_us   = node->sf_cfg.sub_g_len_us;
    sp.tdma_bitmap        = is_sub_sync ? node->sub_tdma_bitmap : node->tdma_bitmap;
    sp.act_fire_off_us    = node->sf_cfg.act_fire_off_us;
    sp.channel_map        = node->has_pending_ch_map ? node->pending_channel_map : node->afh.channel_map;
    sp.ack_bitmap         = is_sub_sync ? node->last_sub_ack_bitmap : node->last_ack_bitmap;
    sp.session_salt       = node->sec.session_salt;
    sp.prev_tx_err_ns     = node->last_sync_tx_err_ns;
    node->last_sync_tx_err_ns = 0; /* Consume once in follow-up */
    sp.key_epoch          = node->sec.key_epoch;
    sp.act_cmd            = node->act_cmd;
    sp.hop_depth          = is_sub_sync ? 1u : 0u;
    if (!is_sub_sync && node->act_lookahead_enabled) {
        sp.act_flip_lead_sf = demo_act_flip_lead(sp.sf_seq);
    }
    bool has_dl_service = is_sub_sync
        ? (node->sub_dl_pending && ((node->current_sf_seq & 1u) != 0u))
        : (((node->current_sf_seq & 1u) == 0u) &&
           ((node->ota && node->ota->tx.active) ||
            (on_gw_ring_count(&node->topo_resp_q) > 0u) ||
            (node->mesh_tx_pending) ||
            (node->gw && on_gw_ring_count(&node->gw->downlink_q) > 0u)));
    uint8_t phy_ind = (node->lr_mode == 2u) ? (uint8_t)ON_PHY_LR_SPREAD_250K
                    : ((node->lr_mode == 1u) ? (uint8_t)ON_PHY_LR_DSSS_1M : (uint8_t)ON_PHY_LE_MSK_2M);
    sp.flags              = (uint8_t)(((node->afh_countdown & 0x0Fu) << 4) |
                                      (has_dl_service ? 0x08u : 0x00u) |
                                      (phy_ind & 0x07u));

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
    node->backup_miss_threshold = ON_BACKUP_MISS_THRESHOLD;
    node->peer_lease_timeout_sf = ON_PEER_LEASE_TIMEOUT_SF;
    on_sync_init(&node->sync);
    on_sync_set_queued_tx_mode(&node->sync, node->hal.tx_timing_best_effort);
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

void on_node_set_multi_subg(on_node_t *node, uint16_t sub_g_zone0_id, uint16_t sub_g_zone1_id) {
    if (!node) return;
    node->sub_g_id   = sub_g_zone0_id & ON_SHORT_ID_MASK;
    node->sub_g_id_2 = sub_g_zone1_id & ON_SHORT_ID_MASK;
}

void on_node_record_sync_tx_err_ns(on_node_t *node, int16_t tx_err_ns) {
    if (!node) return;
    node->last_sync_tx_err_ns = tx_err_ns;
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

void on_node_set_long_range_mode(on_node_t *node, uint8_t lr_mode) {
    if (!node) return;
    if (lr_mode > 2u) lr_mode = 2u;
    if (node->role == ON_ROLE_G && node->sync_rx_this_sf) {
        /* Defer gear transition to exact next 0 us superframe boundary so in-flight
         * D-TDMA/Sub-G slots finish cleanly and Backup_ID never false-triggers failover! */
        node->pending_lr_mode     = lr_mode;
        node->has_pending_lr_mode = true;
    } else {
        node->lr_mode             = lr_mode;
        node->has_pending_lr_mode = false;
        (void)on_sf_cfg_set_gear(&node->sf_cfg,
                                 (lr_mode > 0u) ? ON_SF_GEAR_LONGRANGE_20MS : ON_SF_GEAR_MESH_4MS);
    }
}

uint8_t on_node_get_long_range_mode(const on_node_t *node) {
    return node ? node->lr_mode : 0u;
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
                pe->short_id     = as_sub ? (uint16_t)((node->sub_g_zone ? 0x200u : 0x100u) + free_slot)
                                          : alloc_unique_short_id(node);
                pe->slot_idx     = free_slot;
                pe->is_sub_slot  = as_sub;
                pe->capabilities = req.capabilities;
                *bmap |= (uint16_t)(1u << free_slot);
                break;
            }
        }
        if (!pe) return;

        /* Auto-elect Backup_ID or Sub-G if Primary G has none assigned yet
         * (or preempt a plain terminal Backup_ID when a secondary Gateway-capable node joins!) */
        if (as_pri) {
            bool cur_bk_is_gw = false;
            if (node->backup_id != ON_SHORT_ID_UNASSIGNED) {
                const on_peer_entry_t *bk_pe = find_peer_by_id(node, node->backup_id);
                cur_bk_is_gw = (bk_pe && (bk_pe->capabilities & 0x04u) != 0u);
            }
            if ((node->backup_id == ON_SHORT_ID_UNASSIGNED && (req.capabilities & 0x01u)) ||
                (!cur_bk_is_gw && (req.capabilities & 0x05u) == 0x05u)) {
                uint16_t old_bk = node->backup_id;
                node->backup_id = pe->short_id;
                if (old_bk != ON_SHORT_ID_UNASSIGNED && node->sub_g_id == ON_SHORT_ID_UNASSIGNED) {
                    const on_peer_entry_t *old_pe = find_peer_by_id(node, old_bk);
                    if (old_pe && (old_pe->capabilities & 0x02u)) {
                        node->sub_g_id = old_bk;
                    }
                }
            } else if (node->sub_g_id == ON_SHORT_ID_UNASSIGNED && (req.capabilities & 0x02u)) {
                node->sub_g_id = pe->short_id;
            }
        }
    }

    /* v7.2 Zero-Touch Dual-Gateway Config Sync: when a Standby Gateway (caps & 0x04) joins Primary G,
     * automatically push Wi-Fi SSID/password, MQTT Broker credentials & Admin Token over AES-128-CCM! */
    if (as_pri && node->gw && (req.capabilities & 0x04u) != 0u) {
        char pbuf[ON_GW_MAX_PAYLOAD_LEN + 1u];
        size_t wlen = on_gw_format_wifi_prov(node->gw, pbuf, sizeof(pbuf));
        if (wlen > 0u) {
            on_node_send_gw_downlink(node, pe->short_id, ON_GW_MSG_WIFI_PROV,
                                     (const uint8_t *)pbuf, (uint8_t)wlen);
        }
        size_t mlen = on_gw_format_mqtt_prov(node->gw, pbuf, sizeof(pbuf));
        if (mlen > 0u) {
            on_node_send_gw_downlink(node, pe->short_id, ON_GW_MSG_WIFI_PROV,
                                     (const uint8_t *)pbuf, (uint8_t)mlen);
        }
        if (node->gw->cfg.admin_token[0] != '\0') {
            int tlen = snprintf(pbuf, sizeof(pbuf), "TOKEN:%s", node->gw->cfg.admin_token);
            if (tlen > 6 && (size_t)tlen <= ON_GW_MAX_PAYLOAD_LEN) {
                on_node_send_gw_downlink(node, pe->short_id, ON_GW_MSG_WIFI_PROV,
                                         (const uint8_t *)pbuf, (uint8_t)tlen);
            }
        }
        if (node->lr_mode > 0u) {
            int llen = snprintf(pbuf, sizeof(pbuf), "LRMODE:%u", (unsigned)node->lr_mode);
            if (llen == 8) {
                on_node_send_gw_downlink(node, pe->short_id, ON_GW_MSG_WIFI_PROV,
                                         (const uint8_t *)pbuf, (uint8_t)llen);
            }
        }
    }

    pe->last_seen_sf = node->current_sf_seq;
    if (as_pri) {
        on_cluster_node_t *cn = cluster_find_or_alloc(node, pe->short_id);
        if (cn) {
            memcpy(cn->mac48, pe->mac48, 6u);
            cn->parent_id    = node->short_id;
            cn->hop_depth    = 1u;
            cn->slot_idx     = pe->slot_idx;
            cn->capabilities = pe->capabilities;
            cn->last_seen_sf = node->current_sf_seq;
        }
    } else if (as_sub) {
        pe->mac_reported_to_g = false;
    }

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

/* Discard the old assignment without losing link counters. */
static void rejoin_primary(on_node_t *node, bool reset_clock) {
    node->role = ON_ROLE_UNJOINED;
    node->short_id = ON_SHORT_ID_UNASSIGNED;
    node->my_slot = 0u;
    node->primary_slot_leased = false;
    node->awaiting_ack = false;
    node->harq_retry_pending = false;
    node->rtt_in_flight = false;
    node->rtt_next_sf = 0u;
    node->last_sync_tx_err_ns = 0;
    node->has_pending_ch_map = false;
    node->afh_countdown = 0u;
    node->in_subg_z1_ch = false;
    node->pending_key_rotation = false;
    node->rx_slot_mask_this_sf = 0u;
    node->sub_rx_slot_mask_this_sf = 0u;
    node->last_ack_bitmap = 0u;
    node->last_sub_ack_bitmap = 0u;
    node->sub_tdma_bitmap = 0u;
    node->relay_head = node->relay_tail = node->relay_count = 0u;
    node->sub_g_zone = 0u;
    node->next_alloc_id = 0x002u;
    memset(node->peers, 0, sizeof(node->peers));

    /* Only a new primary session invalidates the synchronized clock timeline. */
    if (reset_clock) {
        const int64_t ant_delay_ns = node->sync.ant_delay_ns;
        on_sync_reset(&node->sync);
        on_sync_set_ant_delay_ns(&node->sync, ant_delay_ns);
    }
    sync_mesh_identity(node);
    node->rejoin_count++;
}

void on_node_on_rx_raw(on_node_t *node, const uint8_t *frame, size_t len, int64_t rx_local_ns) {
    on_node_on_rx_raw_timed_rssi(node, frame, len, rx_local_ns, rx_local_ns, 0);
}

void on_node_on_rx_raw_timed(on_node_t *node, const uint8_t *frame, size_t len,
                             int64_t rx_local_ns, int64_t act_rx_local_ns) {
    on_node_on_rx_raw_timed_rssi(node, frame, len, rx_local_ns, act_rx_local_ns, 0);
}

void on_node_on_rx_raw_rssi(on_node_t *node, const uint8_t *frame, size_t len,
                            int64_t rx_local_ns, int8_t rssi_dbm) {
    on_node_on_rx_raw_timed_rssi(node, frame, len, rx_local_ns, rx_local_ns, rssi_dbm);
}

static void record_overheard_rssi(on_node_t *node, uint16_t src_id, int8_t rssi_dbm) {
    if (!node || src_id == ON_SHORT_ID_UNASSIGNED || rssi_dbm >= 0 || rssi_dbm < -120) return;
    if (node->role == ON_ROLE_G) {
        if (node->on_rssi_observed) {
            node->on_rssi_observed(node->rssi_obs_ctx, node->short_id, src_id, rssi_dbm);
        }
        return;
    }
    if (!node->rssi_report_enabled) return;
    for (uint8_t i = 0u; i < node->rssi_obs_count; ++i) {
        if (node->rssi_obs[i].short_id == src_id) {
            node->rssi_obs[i].rssi_dbm = rssi_dbm;
            return;
        }
    }
    if (node->rssi_obs_count < 6u) {
        node->rssi_obs[node->rssi_obs_count].short_id = src_id;
        node->rssi_obs[node->rssi_obs_count].rssi_dbm = rssi_dbm;
        node->rssi_obs_count++;
    } else {
        /* Overwrite oldest slot round-robin */
        uint8_t idx = (uint8_t)(node->current_sf_seq % 6u);
        node->rssi_obs[idx].short_id = src_id;
        node->rssi_obs[idx].rssi_dbm = rssi_dbm;
    }
}

void on_node_on_rx_raw_timed_rssi(on_node_t *node, const uint8_t *frame, size_t len,
                                  int64_t rx_local_ns, int64_t act_rx_local_ns, int8_t rssi_dbm) {
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

    /* Ignore own loopback (except equal-ID Primary G split-brain resolution via session_salt) */
    if (node->short_id != ON_SHORT_ID_UNASSIGNED && hdr.src_id == node->short_id &&
        !(node->role == ON_ROLE_G && hdr.type == ON_FRAME_SYNC_MAP && node->leaderless_boot_enabled)) {
        return;
    }

    /* v7.0 Promiscuous RSSI capture BEFORE unicast destination filter (enables multi-anchor indoor positioning) */
    if (rssi_dbm < 0 && hdr.src_id != ON_SHORT_ID_UNASSIGNED) {
        record_overheard_rssi(node, hdr.src_id, rssi_dbm);
    }

    /* Fast L2 destination filter: skip unicast frames not addressed to us (except Backup_ID / Standby Gateway passive mirror) */
    bool is_standby_mirror = (node->short_id != ON_SHORT_ID_UNASSIGNED &&
                              (node->short_id == node->backup_id || (node->capabilities & 0x04u) != 0u));
    bool is_for_me = (hdr.dst_id == ON_SHORT_ID_BROADCAST) ||
                     (node->short_id != ON_SHORT_ID_UNASSIGNED && hdr.dst_id == node->short_id) ||
                     (is_standby_mirror &&
                      (hdr.type == ON_FRAME_TDMA_DATA || hdr.type == ON_FRAME_GW_BRIDGE) && !hdr.is_sub_g);
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

            /* A current primary only considers beacons from a lower-ID primary
             * (or equal-ID leaderless split-brain with strictly lower session_salt). */
            bool yielding_primary = node->role == ON_ROLE_G;
            if (yielding_primary && (is_sub || hdr.src_id > node->short_id)) break;
            if (yielding_primary && hdr.src_id == node->short_id &&
                !node->leaderless_boot_enabled) break;

            /* Never let a 1-hop node that hears Primary SYNC_MAP downgrade to SUB_SYNC. */
            if (is_sub && node->heard_primary_sync) break;

            /* 1-hop joined nodes ignore SUB_SYNC; 2-hop edge nodes only lock to SUB_SYNC */
            if (node->role != ON_ROLE_UNJOINED) {
                if (!node->is_sub_node && is_sub) break;
                if (node->is_sub_node && !is_sub) break;
            }

            on_sync_payload_t sp;
            if (on_sync_payload_unpack(payload, plen, &sp) != ON_OK) break;

            if (yielding_primary && hdr.src_id == node->short_id &&
                sp.session_salt >= node->sec.session_salt) {
                break; /* Leaderless split-brain tie-breaker: lower session_salt wins */
            }

            /* v3.0 Verify 32-bit AES-CBC-MAC anti-spoofing tag before trusting beacon */
            if (node->sec.enabled) {
                if (!on_sec_verify_beacon_mac(&node->sec, frame, payload, sp.sf_seq, sp.session_salt)) {
                    emit_event(node, "BEACON_SPOOF_DROP", hdr.src_id, (int64_t)sp.sf_seq);
                    break;
                }
            }

            /* Change role or assignment only after accepting the beacon. A same-salt key
             * epoch change preserves the coordinator's schedule and does not require JOIN. */
            const bool new_primary_session = !is_sub && node->sec.enabled && node->sync_rx_total > 0u &&
                node->short_id != ON_SHORT_ID_UNASSIGNED && sp.session_salt != node->sec.session_salt;
            if (node->act_lookahead_enabled && (is_sub || yielding_primary ||
                hdr.src_id != node->parent_id ||
                sp.session_salt != node->act_lookahead.session_salt)) {
                reset_act_lookahead(node);
            }
            if (yielding_primary) {
                if (hdr.src_id == node->short_id && !new_primary_session) {
                    rejoin_primary(node, true);
                } else {
                    node->role = ON_ROLE_T;
                    sync_mesh_identity(node);
                }
                emit_event(node, "G_YIELD_TO_PRIMARY", hdr.src_id, (int64_t)node->current_sf_seq);
            }
            if (new_primary_session) {
                rejoin_primary(node, true);
                emit_event(node, "PRIMARY_SESSION_REJOIN", hdr.src_id, (int64_t)sp.sf_seq);
            } else if (!is_sub && !yielding_primary && node->primary_slot_leased &&
                       hdr.src_id == node->parent_id && node->my_slot < 16u) {
                const uint32_t since_join = sp.sf_seq - node->primary_join_sf;
                /* JOIN_ACK follows the map in its superframe. A later map from the
                 * same parent must still advertise a dynamically assigned slot. */
                if (since_join != 0u && since_join <= UINT32_MAX / 2u &&
                    (sp.tdma_bitmap & (uint16_t)(1u << node->my_slot)) == 0u) {
                    rejoin_primary(node, false);
                    emit_event(node, "PRIMARY_LEASE_REJOIN", hdr.src_id, (int64_t)sp.sf_seq);
                }
            }
            if (!is_sub) node->heard_primary_sync = true;
            if (node->sec.enabled) {
                on_sec_set_session(&node->sec, sp.session_salt, sp.key_epoch);
            }

            /* v3.1 IEEE 1588 PTP Two-Step Follow-Up: apply previous beacon's TX departure error */
            if (sp.prev_tx_err_ns != 0) {
                on_sync_apply_followup(&node->sync, sp.prev_tx_err_ns);
            }

            /* Step 1 + Step 3: Hardware RX capture feeds TC-PLL / NCO lock */
            on_sync_set_queued_tx_mode(&node->sync, node->hal.tx_timing_best_effort);
            on_sync_on_beacon(&node->sync, sp.t_tx_net_ns, rx_local_ns);
            node->sync_rx_total++;
            if (node->backup_min_sync_silence_ns > 0) {
                node->last_sync_accept_local_ns = node->hal.now_ns
                    ? node->hal.now_ns(node->hal.user_ctx) : rx_local_ns;
            }
            node->sync_rx_this_sf   = true;
            node->missed_sync_count = 0u;
            /* v6.8: Detect Primary G failover (parent_id changed on joined 1-hop node) -> re-report MAC48s */
            if (!is_sub && node->role != ON_ROLE_UNJOINED && !node->is_sub_node &&
                node->parent_id != ON_SHORT_ID_UNASSIGNED && hdr.src_id != node->parent_id) {
                node->self_mac_reported_to_g = false;
                if (node->role == ON_ROLE_SUB_G) {
                    for (uint32_t pi = 0u; pi < ON_MAX_NODES_TABLE; ++pi) {
                        if (node->peers[pi].active && node->peers[pi].is_sub_slot) {
                            node->peers[pi].mac_reported_to_g = false;
                        }
                    }
                }
            }
            node->parent_id         = hdr.src_id;
            node->current_sf_seq    = sp.sf_seq;
            uint8_t rx_phy = (uint8_t)(sp.flags & 0x07u);
            if (rx_phy == (uint8_t)ON_PHY_LR_SPREAD_250K) {
                node->lr_mode = 2u;
            } else if (rx_phy == (uint8_t)ON_PHY_LR_DSSS_1M || sp.t_sf_us >= 20000u) {
                node->lr_mode = 1u;
            } else {
                node->lr_mode = 0u;
            }
            if (sp.t_sf_us != node->sf_cfg.t_sf_us) {
                on_sf_gear_t gear = (sp.t_sf_us >= 20000u) ? ON_SF_GEAR_LONGRANGE_20MS
                                  : (sp.t_sf_us == 1000u   ? ON_SF_GEAR_AUDIO_1MS
                                  : (sp.t_sf_us == 500u    ? ON_SF_GEAR_FAST_500US
                                                           : ON_SF_GEAR_MESH_4MS));
                (void)on_sf_cfg_set_gear(&node->sf_cfg, gear);
            }
            node->sf_cfg.t_sf_us         = sp.t_sf_us;
            node->sf_cfg.sub_g_start_us  = sp.sub_g_win_start_us;
            node->sf_cfg.sub_g_len_us    = sp.sub_g_win_len_us;
            node->sf_cfg.act_fire_off_us = sp.act_fire_off_us;
            if (node->act_lookahead_enabled && !is_sub) {
                receive_act_lookahead(node, &sp, act_rx_local_ns);
            } else {
                node->act_cmd = sp.act_cmd;
            }

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
                node->sub_g_id_2  = sp.sub_g_id_2;
                if (node->role == ON_ROLE_UNJOINED) {
                    node->is_sub_node = false;
                }
                /* Promote to Zone-0 or Zone-1 Sub-G proxy if designated by primary G */
                if (node->role == ON_ROLE_T &&
                    node->short_id != ON_SHORT_ID_UNASSIGNED &&
                    (node->short_id == sp.sub_g_id || node->short_id == sp.sub_g_id_2)) {
                    node->role       = ON_ROLE_SUB_G;
                    node->sub_g_zone = (node->short_id == sp.sub_g_id_2) ? 1u : 0u;
                    emit_event(node, "PROMOTED_SUB_G", node->short_id, (int64_t)node->sub_g_zone);
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
            node->service_tx_done_this_sf  = false;
            node->sub_dl_tx_done_this_sf   = false;
            node->service_dl_reserved_this_sf = ((sp.flags & 0x08u) != 0u);
            if (node->sub_dl_listen_sfs > 0u) node->sub_dl_listen_sfs--;
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
                node->primary_slot_leased = !node->is_sub_node;
                node->primary_join_sf = node->current_sf_seq;
                node->self_mac_reported_to_g = true; /* Primary G already learned MAC48 from JOIN_REQ */
                node->rtt_next_sf = node->current_sf_seq + 1u;
                node->sync.rtt_calibrated = false; /* Fresh RTT lock to new parent */
                sync_mesh_identity(node);
                emit_event(node, node->is_sub_node ? "JOINED_VIA_SUBG" : "JOINED_PRIMARY",
                           node->short_id, node->my_slot);
            }
            break;
        }

        case ON_FRAME_RTT_REQ: {
            if (node->hal.tx_timing_best_effort || hdr.dst_id != node->short_id) break;
            on_rtt_req_t req;
            if (on_rtt_req_unpack(payload, plen, &req) != ON_OK) break;

            int64_t now_loc = node->hal.now_ns ? node->hal.now_ns(node->hal.user_ctx) : rx_local_ns;
            int64_t t2_net  = on_sync_local_to_net(&node->sync, rx_local_ns);
            /* Schedule hardware TX strictly AFTER RX completion so T3 matches physical TX start */
            int64_t tx_local = (now_loc > rx_local_ns ? now_loc : rx_local_ns) + 25000LL;
            int64_t t3_net   = on_sync_local_to_net(&node->sync, tx_local);

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
            if (node->hal.tx_timing_best_effort || hdr.dst_id != node->short_id || !node->rtt_in_flight) break;
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
                cluster_record_tdma(node, &hdr, &dp, pe);
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
                /* Fair per-origin coalescing: update queued stale sample from same child so no 2-hop node starves */
                bool updated_in_place = false;
                for (uint8_t qi = 0u; qi < node->relay_count; ++qi) {
                    uint8_t idx = (uint8_t)((node->relay_head + qi) % ON_DEFAULT_SUBG_MAX_SLOTS);
                    if (node->relay_q[idx].origin_id == rdp.origin_id) {
                        node->relay_q[idx] = rdp;
                        updated_in_place = true;
                        break;
                    }
                }
                if (!updated_in_place && node->relay_count < ON_DEFAULT_SUBG_MAX_SLOTS) {
                    node->relay_q[node->relay_tail] = rdp;
                    node->relay_tail = (uint8_t)((node->relay_tail + 1u) % ON_DEFAULT_SUBG_MAX_SLOTS);
                    node->relay_count++;
                }
            } else if ((node->short_id == node->backup_id || (node->capabilities & 0x04u) != 0u) && !hdr.is_sub_g) {
                /* Hot-standby Backup_ID & all Standby Gateways passively mirror active 1-hop peer table & cluster_tbl */
                uint8_t peer_caps = (uint8_t)((dp.flags >> 6) & 0x03u);
                if (peer_caps == 0u) peer_caps = 0x01u;
                on_peer_entry_t *pe = find_peer_by_id(node, hdr.src_id);
                if (!pe) {
                    for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
                        if (!node->peers[i].active) {
                            pe = &node->peers[i];
                            pe->active       = true;
                            pe->short_id     = hdr.src_id;
                            pe->slot_idx     = hdr.slot_idx;
                            pe->is_sub_slot  = false;
                            pe->capabilities = peer_caps;
                            pe->last_seen_sf = node->current_sf_seq;
                            break;
                        }
                    }
                } else {
                    pe->capabilities = peer_caps;
                    pe->last_seen_sf = node->current_sf_seq;
                }
                cluster_record_tdma(node, &hdr, &dp, pe);
            }
            break;
        }

        case ON_FRAME_OTA_META: {
            on_ota_meta_t meta;
            if (on_ota_meta_unpack(payload, plen, &meta) != ON_OK) break;
            node->ota_rx_total++;
            if (node->ota && node->role != ON_ROLE_G) {
                if (on_ota_rx_on_meta(&node->ota->rx, &meta, node->short_id) == ON_OK) {
                    emit_event(node, "OTA_META_RX", meta.ota_id, (int64_t)meta.fw_version);
                }
            }
            /* Sub-G transparently relays broadcast or 2-hop-targeted OTA_META into Sub-G window */
            if (node->role == ON_ROLE_SUB_G && !hdr.is_sub_g &&
                (meta.target_id == ON_SHORT_ID_BROADCAST || meta.target_id >= 0x100u) &&
                plen <= sizeof(node->sub_dl_payload)) {
                node->sub_dl_pending = true;
                node->sub_dl_type    = (uint8_t)ON_FRAME_OTA_META;
                node->sub_dl_dst_id  = meta.target_id;
                node->sub_dl_len     = (uint8_t)plen;
                memcpy(node->sub_dl_payload, payload, plen);
            }
            break;
        }

        case ON_FRAME_OTA_CHUNK: {
            on_ota_chunk_t chunk;
            if (on_ota_chunk_unpack(payload, plen, &chunk) != ON_OK) break;
            node->ota_rx_total++;
            if (node->ota && node->role != ON_ROLE_G) {
                const uint8_t *auth_key = node->sec.enabled ? node->sec.master_key : NULL;
                on_ota_rx_on_chunk(&node->ota->rx, &chunk, auth_key);
                if (node->ota->rx.state == ON_OTA_STATE_READY) {
                    emit_event(node, "OTA_IMAGE_READY", chunk.ota_id, (int64_t)node->ota->rx.meta.fw_version);
                } else if (node->ota->rx.state == ON_OTA_STATE_FAILED) {
                    emit_event(node, "OTA_VERIFY_FAIL", chunk.ota_id, (int64_t)node->ota->rx.last_err);
                }
            }
            /* Sub-G relays broadcast or 2-hop OTA_CHUNK into Sub-G window */
            if (node->role == ON_ROLE_SUB_G && !hdr.is_sub_g &&
                (hdr.dst_id == ON_SHORT_ID_BROADCAST || hdr.dst_id == node->short_id || hdr.dst_id >= 0x100u) &&
                plen <= sizeof(node->sub_dl_payload)) {
                node->sub_dl_pending = true;
                node->sub_dl_type    = (uint8_t)ON_FRAME_OTA_CHUNK;
                node->sub_dl_dst_id  = ON_SHORT_ID_BROADCAST;
                node->sub_dl_len     = (uint8_t)plen;
                memcpy(node->sub_dl_payload, payload, plen);
            }
            break;
        }

        case ON_FRAME_OTA_STATUS: {
            on_ota_status_t st;
            if (on_ota_status_unpack(payload, plen, &st) != ON_OK) break;
            node->ota_rx_total++;
            if (node->role == ON_ROLE_G && node->ota) {
                on_ota_tx_on_status(&node->ota->tx, &st, node->current_sf_seq);
                cluster_record_ota(node, &st);
                emit_event(node, "OTA_STATUS_RX", st.node_id, (int64_t)st.rx_chunks);
            } else if (node->role == ON_ROLE_SUB_G && hdr.is_sub_g && plen <= sizeof(node->sub_ul_payload)) {
                /* Sub-G forwards 2-hop child OTA_STATUS back to Primary G */
                node->sub_ul_pending = true;
                node->sub_ul_type    = (uint8_t)ON_FRAME_OTA_STATUS;
                node->sub_ul_len     = (uint8_t)plen;
                memcpy(node->sub_ul_payload, payload, plen);
            }
            break;
        }

        case ON_FRAME_OTA_COMMIT: {
            on_ota_commit_t cm;
            if (on_ota_commit_unpack(payload, plen, &cm) != ON_OK) break;
            node->ota_rx_total++;
            if (node->ota && node->role != ON_ROLE_G) {
                if (on_ota_rx_on_commit(&node->ota->rx, &cm, node->short_id) == ON_OK) {
                    emit_event(node, "OTA_COMMIT_ARMED", cm.ota_id, (int64_t)cm.commit_sf_seq);
                }
            }
            if (node->role == ON_ROLE_SUB_G && !hdr.is_sub_g &&
                (cm.target_id == ON_SHORT_ID_BROADCAST || cm.target_id >= 0x100u) &&
                plen <= sizeof(node->sub_dl_payload)) {
                node->sub_dl_pending = true;
                node->sub_dl_type    = (uint8_t)ON_FRAME_OTA_COMMIT;
                node->sub_dl_dst_id  = cm.target_id;
                node->sub_dl_len     = (uint8_t)plen;
                memcpy(node->sub_dl_payload, payload, plen);
            }
            break;
        }

        case ON_FRAME_GW_BRIDGE: {
            on_gw_bridge_msg_t msg;
            if (on_gw_bridge_unpack(payload, plen, &msg) != ON_OK) break;
            node->gw_rx_total++;
            bool is_uplink = (msg.bridge_type == ON_GW_MSG_TELEMETRY_UP ||
                              msg.bridge_type == ON_GW_MSG_BLE_NOTIFY_UP ||
                              msg.bridge_type == ON_GW_MSG_TOPO_REQ ||
                              msg.bridge_type == ON_GW_MSG_MAC_REPORT ||
                              msg.bridge_type == ON_GW_MSG_RSSI_REPORT ||
                              msg.bridge_type == ON_GW_MSG_BLE_BEACON_REPORT ||
                              msg.bridge_type == ON_GW_MSG_CSI_REPORT);
            if (is_uplink) {
                bool is_pri_g = (node->role == ON_ROLE_G);
                bool is_bk_mirror = (!is_pri_g && node->short_id != ON_SHORT_ID_UNASSIGNED &&
                                     (node->short_id == node->backup_id || (node->capabilities & 0x04u) != 0u) &&
                                     !hdr.is_sub_g);
                if (is_pri_g || is_bk_mirror) {
                    if (msg.bridge_type == ON_GW_MSG_TOPO_REQ) {
                        if (is_pri_g) {
                            topo_queue_resp_pages(node, msg.origin_id);
                            emit_event(node, "TOPO_REQ_RX", msg.origin_id, (int64_t)node->current_sf_seq);
                        }
                    } else if (msg.bridge_type == ON_GW_MSG_RSSI_REPORT && msg.payload_len >= 3u) {
                        if (is_pri_g) node->rx_slot_mask_this_sf |= (uint16_t)(1u << (hdr.slot_idx & 0x0Fu));
                        on_peer_entry_t *pe_an = find_peer_by_id(node, hdr.src_id);
                        if (pe_an) pe_an->last_seen_sf = node->current_sf_seq;
                        uint16_t anchor_id = (msg.origin_id != ON_SHORT_ID_UNASSIGNED) ? msg.origin_id : hdr.src_id;
                        if (anchor_id != hdr.src_id) {
                            on_cluster_node_t *cn_an = cluster_find_or_alloc(node, anchor_id);
                            if (cn_an) cn_an->last_seen_sf = node->current_sf_seq;
                        }
                        if (node->on_rssi_observed) {
                            for (uint8_t k = 0u; (uint8_t)(k + 3u) <= msg.payload_len; k = (uint8_t)(k + 3u)) {
                                uint16_t tid = (uint16_t)(((uint16_t)msg.payload[k] |
                                                          ((uint16_t)msg.payload[k + 1u] << 8)) & ON_SHORT_ID_MASK);
                                int8_t rdbm = (int8_t)msg.payload[k + 2u];
                                node->on_rssi_observed(node->rssi_obs_ctx, anchor_id, tid, rdbm);
                            }
                        }
                    } else if (msg.bridge_type == ON_GW_MSG_BLE_BEACON_REPORT && msg.payload_len >= 9u) {
                        if (is_pri_g) node->rx_slot_mask_this_sf |= (uint16_t)(1u << (hdr.slot_idx & 0x0Fu));
                        on_peer_entry_t *pe_bn = find_peer_by_id(node, hdr.src_id);
                        if (pe_bn) pe_bn->last_seen_sf = node->current_sf_seq;
                        uint16_t anchor_id = (msg.origin_id != ON_SHORT_ID_UNASSIGNED) ? msg.origin_id : hdr.src_id;
                        if (anchor_id != hdr.src_id) {
                            on_cluster_node_t *cn_bn = cluster_find_or_alloc(node, anchor_id);
                            if (cn_bn) cn_bn->last_seen_sf = node->current_sf_seq;
                        }
                        if (node->on_ble_beacon_observed) {
                            for (uint8_t k = 0u; (uint8_t)(k + 9u) <= msg.payload_len; k = (uint8_t)(k + 9u)) {
                                node->on_ble_beacon_observed(node->ble_obs_ctx, anchor_id,
                                                             &msg.payload[k],
                                                             msg.payload[k + 6u],
                                                             (int8_t)msg.payload[k + 7u],
                                                             (int8_t)msg.payload[k + 8u]);
                            }
                        }
                    } else if (msg.bridge_type == ON_GW_MSG_CSI_REPORT && msg.payload_len >= 17u) {
                        if (is_pri_g) node->rx_slot_mask_this_sf |= (uint16_t)(1u << (hdr.slot_idx & 0x0Fu));
                        on_peer_entry_t *pe_cn = find_peer_by_id(node, hdr.src_id);
                        if (pe_cn) pe_cn->last_seen_sf = node->current_sf_seq;
                        uint16_t anchor_id = (msg.origin_id != ON_SHORT_ID_UNASSIGNED) ? msg.origin_id : hdr.src_id;
                        if (anchor_id != hdr.src_id) {
                            on_cluster_node_t *cn_cs = cluster_find_or_alloc(node, anchor_id);
                            if (cn_cs) cn_cs->last_seen_sf = node->current_sf_seq;
                        }
                        if (node->on_csi_observed) {
                            node->on_csi_observed(node->csi_obs_ctx, anchor_id,
                                                  msg.payload[0], &msg.payload[1]);
                        }
                    } else if (msg.bridge_type == ON_GW_MSG_MAC_REPORT && msg.payload_len >= 6u) {
                        if (is_pri_g) node->rx_slot_mask_this_sf |= (uint16_t)(1u << (hdr.slot_idx & 0x0Fu));
                        on_peer_entry_t *pe_sg = find_peer_by_id(node, hdr.src_id);
                        if (pe_sg) {
                            pe_sg->last_seen_sf = node->current_sf_seq;
                            if (msg.origin_id == hdr.src_id) {
                                memcpy(pe_sg->mac48, msg.payload, 6u);
                            }
                        }
                        on_cluster_node_t *cn = cluster_find_or_alloc(node, msg.origin_id);
                        if (cn) {
                            memcpy(cn->mac48, msg.payload, 6u);
                            if (msg.origin_id == hdr.src_id) {
                                cn->parent_id = is_pri_g ? node->short_id : hdr.dst_id;
                                cn->hop_depth = 1u;
                                cn->slot_idx  = pe_sg ? pe_sg->slot_idx : hdr.slot_idx;
                                cn->role      = (hdr.src_id == node->sub_g_id || hdr.src_id == node->sub_g_id_2)
                                                ? (uint8_t)ON_ROLE_SUB_G : (uint8_t)ON_ROLE_T;
                            } else {
                                if (cn->parent_id == ON_SHORT_ID_UNASSIGNED) cn->parent_id = hdr.src_id;
                                if (cn->hop_depth == 0u) cn->hop_depth = 2u;
                                if (cn->role == 0u) cn->role = (uint8_t)ON_ROLE_T;
                            }
                            cn->last_seen_sf = node->current_sf_seq;
                        }
                    } else if (is_pri_g && node->gw) {
                        on_gw_push_uplink(node->gw, &msg);
                        emit_event(node, "GW_UPLINK_RX", msg.origin_id, (int64_t)msg.seq);
                    }
                } else if (node->role == ON_ROLE_SUB_G && hdr.is_sub_g) {
                    node->sub_rx_slot_mask_this_sf |= (uint16_t)(1u << (hdr.slot_idx & 0x0Fu));
                    on_peer_entry_t *pe_ch = find_peer_by_id(node, hdr.src_id);
                    if (pe_ch) pe_ch->last_seen_sf = node->current_sf_seq;
                    msg.flags |= ON_GW_FLAG_RELAYED_2HOP;
                    size_t rlen = 0u;
                    if (on_gw_bridge_pack(&msg, node->sub_ul_payload, sizeof(node->sub_ul_payload), &rlen) == ON_OK) {
                        node->sub_ul_pending    = true;
                        node->sub_ul_type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                        node->sub_ul_len        = (uint8_t)rlen;
                        node->sub_dl_listen_sfs = 48u; /* Stay awake in C-RA to catch all paginated responses for 2-hop child */
                    }
                }
            } else {
                /* Downlink command, Dual-Gateway encrypted config sync, or cluster topology snapshot response */
                bool for_self = (msg.origin_id == node->short_id || msg.origin_id == ON_SHORT_ID_BROADCAST);
                if (for_self) {
                    if (msg.bridge_type == ON_GW_MSG_TOPO_RESP) {
                        topo_ingest_resp_page(node, &msg);
                    } else if (msg.bridge_type == ON_GW_MSG_WIFI_PROV) {
                        if (node->gw && msg.payload_len > 0u) {
                            char saved_tok[32];
                            memcpy(saved_tok, node->gw->cfg.admin_token, sizeof(saved_tok));
                            node->gw->cfg.admin_token[0] = '\0';
                            (void)on_gw_handle_control_text(node->gw,
                                                            (const char *)msg.payload,
                                                            msg.payload_len, false);
                            if (!node->gw->cfg.admin_token_updated) {
                                memcpy(node->gw->cfg.admin_token, saved_tok, sizeof(saved_tok));
                            }
                        }
                        if (node->gw && node->gw->ops.on_downlink_delivered) {
                            node->gw->ops.on_downlink_delivered(node->gw->ops.user_ctx, &msg);
                        }
                        emit_event(node, "GW_PROV_SYNC_RX", msg.origin_id, (int64_t)msg.seq);
                    } else {
                        if (node->gw && node->gw->ops.on_downlink_delivered) {
                            node->gw->ops.on_downlink_delivered(node->gw->ops.user_ctx, &msg);
                        }
                        emit_event(node, "GW_DOWNLINK_RX", msg.origin_id, (int64_t)msg.seq);
                    }
                }
                if (node->role == ON_ROLE_SUB_G && !hdr.is_sub_g &&
                    (msg.origin_id == ON_SHORT_ID_BROADCAST || msg.origin_id >= 0x100u) &&
                    plen <= sizeof(node->sub_dl_payload)) {
                    node->sub_dl_pending = true;
                    node->sub_dl_type    = (uint8_t)ON_FRAME_GW_BRIDGE;
                    node->sub_dl_dst_id  = msg.origin_id;
                    node->sub_dl_len     = (uint8_t)plen;
                    memcpy(node->sub_dl_payload, payload, plen);
                }
            }
            break;
        }

        case ON_FRAME_MESH: {
            if (!node->mesh) break;
            on_mesh_pkt_t mpkt, fwd;
            if (on_mesh_pkt_unpack(payload, plen, &mpkt) != ON_OK) break;
            node->mesh_rx_total++;
            on_mesh_action_t act = on_mesh_process_rx(node->mesh, hdr.src_id, &mpkt,
                                                      node->current_sf_seq, &fwd);
            if (act == ON_MESH_ACT_DELIVER || act == ON_MESH_ACT_DELIV_FWD) {
                if (node->role == ON_ROLE_G &&
                    (mpkt.mesh_type == ON_MESH_DATA_UP || mpkt.mesh_type == ON_MESH_TOPO_REPORT)) {
                    /* Auto-register 3~6-hop Mesh node into Primary G's cluster_tbl & Gateway uplink! */
                    on_cluster_node_t *cn = cluster_find_or_alloc(node, mpkt.src_id);
                    if (cn) {
                        cn->parent_id     = hdr.src_id;
                        cn->hop_depth     = mpkt.hop_count;
                        cn->role          = (uint8_t)ON_ROLE_T;
                        cn->last_seen_sf  = node->current_sf_seq;
                        cn->rx_packets++;
                        cn->last_seq      = mpkt.msg_id;
                        if (mpkt.mesh_type == ON_MESH_TOPO_REPORT && mpkt.payload_len >= 6u) {
                            memcpy(cn->mac48, mpkt.payload, 6u);
                            uint8_t rem = (uint8_t)(mpkt.payload_len - 6u);
                            if (rem > 0u) {
                                cn->last_data_len = (rem <= 8u) ? rem : 8u;
                                memcpy(cn->last_data, mpkt.payload + 6u, cn->last_data_len);
                            }
                        } else {
                            cn->last_data_len = (mpkt.payload_len <= 8u) ? mpkt.payload_len : 8u;
                            if (cn->last_data_len > 0u) {
                                memcpy(cn->last_data, mpkt.payload, cn->last_data_len);
                            }
                        }
                    }
                    if (node->gw) {
                        on_gw_bridge_msg_t umsg;
                        memset(&umsg, 0, sizeof(umsg));
                        umsg.bridge_type = (uint8_t)ON_GW_MSG_TELEMETRY_UP;
                        umsg.flags       = ON_GW_FLAG_RELAYED_2HOP;
                        umsg.origin_id   = mpkt.src_id;
                        umsg.seq         = mpkt.msg_id;
                        umsg.payload_len = (mpkt.payload_len <= ON_GW_MAX_PAYLOAD_LEN)
                                           ? mpkt.payload_len : ON_GW_MAX_PAYLOAD_LEN;
                        if (umsg.payload_len > 0u) memcpy(umsg.payload, mpkt.payload, umsg.payload_len);
                        on_gw_push_uplink(node->gw, &umsg);
                    }
                    emit_event(node, "MESH_DATA_UP_RX", mpkt.src_id, (int64_t)mpkt.hop_count);
                } else if (mpkt.mesh_type == ON_MESH_CMD_DOWN) {
                    if (node->gw && node->gw->ops.on_downlink_delivered) {
                        on_gw_bridge_msg_t dmsg;
                        memset(&dmsg, 0, sizeof(dmsg));
                        dmsg.bridge_type = (uint8_t)ON_GW_MSG_CLOUD_CMD_DOWN;
                        dmsg.origin_id   = mpkt.dst_id;
                        dmsg.seq         = mpkt.msg_id;
                        dmsg.payload_len = (mpkt.payload_len <= ON_GW_MAX_PAYLOAD_LEN)
                                           ? mpkt.payload_len : ON_GW_MAX_PAYLOAD_LEN;
                        if (dmsg.payload_len > 0u) memcpy(dmsg.payload, mpkt.payload, dmsg.payload_len);
                        node->gw->ops.on_downlink_delivered(node->gw->ops.user_ctx, &dmsg);
                    }
                    emit_event(node, "MESH_CMD_DOWN_RX", mpkt.src_id, (int64_t)mpkt.hop_count);
                }
            }
            if (act == ON_MESH_ACT_FORWARD || act == ON_MESH_ACT_DELIV_FWD) {
                node->mesh_tx_pkt     = fwd;
                node->mesh_tx_pending = true;
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
                node->act_cmd = demo_act_level(node->current_sf_seq);

                /* Atomic key epoch rotation at exact 0 us superframe boundary */
                if (node->pending_key_rotation && node->sec.enabled) {
                    node->pending_key_rotation = false;
                    uint8_t next_ep = (uint8_t)(node->sec.key_epoch + 1u);
                    on_sec_set_session(&node->sec, node->sec.session_salt, next_ep);
                    emit_event(node, "KEY_EPOCH_ROTATED", next_ep, (int64_t)node->current_sf_seq);
                }

                /* v7.3 Atomic 4ms <-> 20ms Long-Range gear switch at exact 0 us superframe boundary */
                if (node->has_pending_lr_mode) {
                    node->has_pending_lr_mode = false;
                    node->lr_mode = node->pending_lr_mode;
                    (void)on_sf_cfg_set_gear(&node->sf_cfg,
                        (node->lr_mode > 0u) ? ON_SF_GEAR_LONGRANGE_20MS : ON_SF_GEAR_MESH_4MS);
                    emit_event(node, "LR_MODE_SWITCHED", node->lr_mode, (int64_t)node->current_sf_seq);
                }

                /* v3.0 Lease Expiry GC & fast automatic Sub-G / Backup_ID re-election */
                for (uint32_t i = 0u; i < ON_MAX_NODES_TABLE; ++i) {
                    if (!node->peers[i].active || node->peers[i].is_sub_slot) continue;
                    bool is_active_subg = (!node->hal.tx_timing_best_effort &&
                                           (node->peers[i].short_id == node->sub_g_id ||
                                            node->peers[i].short_id == node->sub_g_id_2));
                    uint32_t timeout_sfs = is_active_subg ? 12u : node->peer_lease_timeout_sf;
                    if (node->current_sf_seq - node->peers[i].last_seen_sf > timeout_sfs) {
                        uint16_t dead_id = node->peers[i].short_id;
                        uint8_t  dead_slot = node->peers[i].slot_idx;
                        node->tdma_bitmap &= (uint16_t)(~(1u << dead_slot));
                        node->peers[i].active = false;
                        cluster_mark_inactive(node, dead_id);
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

                /* v6.6 Multi-hop (2~6 hop) cluster table lease expiry GC (prevents ghost nodes) */
                uint32_t multi_hop_timeout = (node->peer_lease_timeout_sf >= 100u)
                                             ? (node->peer_lease_timeout_sf * 2u) : 200u;
                for (uint32_t ci = 0u; ci < ON_MAX_CLUSTER_NODES; ++ci) {
                    on_cluster_node_t *cn = &node->cluster_tbl[ci];
                    if (!cn->active || cn->hop_depth < 2u) continue;
                    if (node->current_sf_seq > cn->last_seen_sf &&
                        (node->current_sf_seq - cn->last_seen_sf) > multi_hop_timeout) {
                        cn->active = false;
                        node->lease_reclaim_count++;
                        emit_event(node, "CLUSTER_NODE_EXPIRED", cn->short_id, (int64_t)cn->hop_depth);
                    }
                }
            }
            /* Tune to this superframe's TSCH channel before SYNC/MAP TX */
            if (node->afh.enabled) {
                tune_channel(node, on_afh_channel_for(&node->afh, node->current_sf_seq, 0u));
            }
            node->sync_rx_this_sf         = true;
            node->act_fired_this_sf       = false;
            node->service_tx_done_this_sf = false;
            send_sync_map(node, false);
            elapsed_ns = now_net_ns - node->current_sf_start_net_ns;
        }

        /* v5.0 Even-SF Downlink Service Slot (3200 us): Authenticated OTA & GW Bridge Downlink */
        int64_t cra0_ns = (int64_t)node->sf_cfg.cra_start_us * 1000LL;
        if (!node->service_tx_done_this_sf &&
            ((node->current_sf_seq & 1u) == 0u) &&
            elapsed_ns >= cra0_ns && elapsed_ns < cra0_ns + 80000LL) {
            if (node->ota && node->ota->tx.active) {
                node->service_tx_done_this_sf = true;
                uint8_t pbuf[ON_MAX_PAYLOAD_LEN];
                size_t plen = 0u;
                on_mac_hdr_t hdr;
                memset(&hdr, 0, sizeof(hdr));
                hdr.version    = ON_PROTO_VERSION;
                hdr.src_id     = node->short_id;
                hdr.dst_id     = node->ota->tx.meta.target_id;
                hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                hdr.slot_idx   = 14u;

                if (!node->ota->tx.meta_sent || (node->ota->tx.meta_repeat_cnt > 0u && (node->current_sf_seq & 0x0Fu) == 0u)) {
                    hdr.type = (uint8_t)ON_FRAME_OTA_META;
                    if (on_ota_meta_pack(&node->ota->tx.meta, pbuf, sizeof(pbuf), &plen) == ON_OK) {
                        send_frame(node, &hdr, pbuf, plen, now_local_ns);
                        node->ota->tx.meta_sent = true;
                        if (node->ota->tx.meta_repeat_cnt > 0u) node->ota->tx.meta_repeat_cnt--;
                        node->ota_tx_total++;
                    }
                } else if (node->ota->tx.commit_sf_seq > 0u && !node->ota->tx.commit_sent) {
                    on_ota_commit_t cm;
                    cm.ota_id        = node->ota->tx.meta.ota_id;
                    cm.commit_sf_seq = node->ota->tx.commit_sf_seq;
                    cm.crc32         = node->ota->tx.meta.crc32;
                    cm.target_id     = node->ota->tx.meta.target_id;
                    cm.flags         = node->ota->tx.meta.flags;
                    hdr.type = (uint8_t)ON_FRAME_OTA_COMMIT;
                    if (on_ota_commit_pack(&cm, pbuf, sizeof(pbuf), &plen) == ON_OK) {
                        send_frame(node, &hdr, pbuf, plen, now_local_ns);
                        node->ota->tx.commit_sent = true;
                        node->ota_tx_total++;
                    }
                } else {
                    on_ota_chunk_t chunk;
                    if (on_ota_tx_next_chunk(&node->ota->tx, &chunk) &&
                        on_ota_chunk_pack(&chunk, pbuf, sizeof(pbuf), &plen) == ON_OK) {
                        hdr.type = (uint8_t)ON_FRAME_OTA_CHUNK;
                        /* If targeting a 2-hop node (>=0x100), route via Sub-G proxy */
                        if (hdr.dst_id >= 0x100u && hdr.dst_id != ON_SHORT_ID_BROADCAST &&
                            node->sub_g_id != ON_SHORT_ID_UNASSIGNED) {
                            hdr.dst_id = node->sub_g_id;
                        }
                        send_frame(node, &hdr, pbuf, plen, now_local_ns);
                        node->ota_tx_total++;
                    }
                }
            } else if (on_gw_ring_count(&node->topo_resp_q) > 0u) {
                on_gw_bridge_msg_t dl = node->topo_resp_q.items[node->topo_resp_q.head];
                node->topo_resp_q.head = (uint8_t)((node->topo_resp_q.head + 1u) & (ON_GW_RING_DEPTH - 1u));
                node->service_tx_done_this_sf = true;
                uint8_t pbuf[ON_MAX_PAYLOAD_LEN];
                size_t plen = 0u;
                if (on_gw_bridge_pack(&dl, pbuf, sizeof(pbuf), &plen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = (dl.origin_id >= 0x100u && dl.origin_id != ON_SHORT_ID_BROADCAST &&
                                      node->sub_g_id != ON_SHORT_ID_UNASSIGNED)
                                     ? node->sub_g_id : dl.origin_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = 14u;
                    send_frame(node, &hdr, pbuf, plen, now_local_ns);
                    node->gw_tx_total++;
                }
            } else if (node->mesh_tx_pending) {
                node->service_tx_done_this_sf = true;
                node->mesh_tx_pending         = false;
                uint8_t mbuf[ON_MAX_PAYLOAD_LEN];
                size_t mlen = 0u;
                if (on_mesh_pkt_pack(&node->mesh_tx_pkt, mbuf, sizeof(mbuf), &mlen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_MESH;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->mesh_tx_pkt.next_hop_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = 14u;
                    send_frame(node, &hdr, mbuf, mlen, now_local_ns);
                    node->mesh_tx_total++;
                }
            } else if (node->gw && on_gw_ring_count(&node->gw->downlink_q) > 0u) {
                on_gw_bridge_msg_t dl;
                if (on_gw_pop_downlink(node->gw, &dl)) {
                    node->service_tx_done_this_sf = true;
                    uint8_t pbuf[ON_MAX_PAYLOAD_LEN];
                    size_t plen = 0u;
                    if (on_gw_bridge_pack(&dl, pbuf, sizeof(pbuf), &plen) == ON_OK) {
                        on_mac_hdr_t hdr;
                        memset(&hdr, 0, sizeof(hdr));
                        hdr.version    = ON_PROTO_VERSION;
                        hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                        hdr.src_id     = node->short_id;
                        hdr.dst_id     = (dl.origin_id >= 0x100u && dl.origin_id != ON_SHORT_ID_BROADCAST &&
                                          node->sub_g_id != ON_SHORT_ID_UNASSIGNED)
                                         ? node->sub_g_id : dl.origin_id;
                        hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                        hdr.slot_idx   = 14u;
                        send_frame(node, &hdr, pbuf, plen, now_local_ns);
                        node->gw->downlink_forwarded++;
                        node->gw_tx_total++;
                    }
                }
            }
        }

        /* Synchronized ACT/QUIET trigger */
        int64_t act_off_ns = (int64_t)node->sf_cfg.act_fire_off_us * 1000LL;
        if (!node->act_fired_this_sf && elapsed_ns >= act_off_ns) {
            node->act_fired_this_sf = true;
            fire_act(node, node->current_sf_seq, now_local_ns, false);
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
            if (node->sync_rx_total > 0u) {
                /* Re-attaching node (e.g. after Sub-G failover) already has NCO net time & AFH map */
                int64_t est_net_ns = on_sync_local_to_net(&node->sync, now_local_ns);
                uint32_t est_sf = (est_net_ns > 0) ? (uint32_t)(est_net_ns / t_sf_ns) : node->current_sf_seq;
                tune_channel(node, on_afh_channel_for(&node->afh, est_sf, 0u));
            } else if (node->scan_last_hop_local_ns == 0) {
                node->scan_last_hop_local_ns = now_local_ns;
            } else if ((now_local_ns - node->scan_last_hop_local_ns) >= 16000000LL) {
                /* Dwell 16 ms (4 superframes) per channel then sweep across all 16 channels */
                node->scan_last_hop_local_ns = now_local_ns;
                node->scan_ch_idx = (uint8_t)((node->scan_ch_idx + 1u) % ON_AFH_NUM_CHANNELS);
                tune_channel(node, node->scan_ch_idx);
            }
        }
        /* v6.0 Leaderless Cold-Boot Auto-Election (lowest MAC48 promotes to Primary G) */
        if (node->leaderless_boot_enabled && node->role == ON_ROLE_UNJOINED) {
            if (node->boot_listen_start_ns == 0) {
                node->boot_listen_start_ns = now_local_ns;
            } else {
                int64_t mac_backoff_ns = (int64_t)node->mac48[5] * 4000000LL;
                int64_t wait_ns = node->leaderless_base_timeout_ns + mac_backoff_ns;
                if ((now_local_ns - node->boot_listen_start_ns) >= wait_ns) {
                    node->role                    = ON_ROLE_G;
                    node->short_id                = ON_SHORT_ID_G_DEFAULT;
                    node->sync.locked             = true;
                    node->sync.ref_local_ns       = now_local_ns;
                    node->sync.ref_net_ns         = now_local_ns;
                    node->current_sf_start_net_ns = now_local_ns;
                    node->current_sf_seq          = 0u;
                    node->sync_rx_this_sf         = true;
                    node->act_fired_this_sf       = false;
                    node->act_cmd                 = 1u;
                    if (node->sec.enabled) {
                        uint32_t mac_salt = ((uint32_t)node->mac48[2] << 24) |
                                            ((uint32_t)node->mac48[3] << 16) |
                                            ((uint32_t)node->mac48[4] << 8)  |
                                            (uint32_t)node->mac48[5];
                        on_sec_set_session(&node->sec, mac_salt, node->sec.key_epoch);
                    }
                    sync_mesh_identity(node);
                    send_sync_map(node, false);
                    emit_event(node, "LEADERLESS_ELECTED_G", node->short_id, (int64_t)node->mac48[5]);
                }
            }
        }
        return;
    }

    int64_t now_net_ns = on_sync_local_to_net(&node->sync, now_local_ns);
    int64_t elapsed_ns = now_net_ns - node->current_sf_start_net_ns;

    /*
     * ZERO-VOTE HOT-STANDBY FAILOVER (Backup_ID):
     * After the configured number of consecutive missed SYNC frames (default 3),
     * Backup_ID takes the role at the next boundary, inheriting the schedule and clock.
     * Queued radios can additionally require a minimum silence interval on the local clock.
     */
    if (node->role == ON_ROLE_T &&
        !node->is_sub_node &&
        node->epoch_period <= 1u &&
        node->short_id != ON_SHORT_ID_UNASSIGNED &&
        node->short_id == node->backup_id &&
        node->missed_sync_count >= node->backup_miss_threshold &&
        (node->backup_min_sync_silence_ns <= 0 ||
         now_local_ns - node->last_sync_accept_local_ns >= node->backup_min_sync_silence_ns) &&
        elapsed_ns >= t_sf_ns) {
        node->current_sf_seq++;
        node->current_sf_start_net_ns += t_sf_ns;
        node->role = ON_ROLE_G;
        node->failover_count++;
        sync_mesh_identity(node);
        /* Anchor G clock smoothly to our locked NCO network time without tick-overshoot jump */
        node->sync.ref_local_ns = on_sync_net_to_local(&node->sync, node->current_sf_start_net_ns);
        node->sync.ref_net_ns   = node->current_sf_start_net_ns;
        /* Refresh mirrored peer lease timestamps at takeover epoch so inherited peers get a full lease window */
        for (uint32_t i = 0u; i < ON_MAX_NODES_TABLE; ++i) {
            if (node->peers[i].active) {
                node->peers[i].last_seen_sf = node->current_sf_seq;
            }
        }
        node->backup_id         = select_next_backup_id(node);
        node->sync_rx_this_sf   = true;
        node->act_fired_this_sf = false;
        node->act_cmd           = demo_act_level(node->current_sf_seq);
        reset_act_lookahead(node);
        if (node->afh.enabled) {
            tune_channel(node, on_afh_channel_for(&node->afh, node->current_sf_seq, 0u));
        }

        send_sync_map(node, false);
        emit_event(node, "BACKUP_TAKEOVER", node->short_id, (int64_t)node->current_sf_seq);
        return;
    }

    /* Superframe rollover on missed SYNC / SUB_SYNC (280 us grace past expected beacon) */
    int64_t expected_next_sync_ns = t_sf_ns +
        (node->is_sub_node ? (int64_t)node->sf_cfg.sub_g_start_us * 1000LL : 0LL);
    if (elapsed_ns >= expected_next_sync_ns + 280000LL) {
        bool planned_epoch_sleep = (node->epoch_period > 1u && node->sync.lock_count >= 4u &&
                                    node->role == ON_ROLE_T && !node->is_sub_node &&
                                    !on_epoch_is_my_sf(node->current_sf_seq + 1u,
                                                       node->epoch_phase, node->epoch_period));
        if (!planned_epoch_sleep) {
            node->missed_sync_count++;
        }
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
            if (node->missed_sync_count > 6u && (node->missed_sync_count & 1u) != 0u) {
                node->scan_ch_idx = (uint8_t)((node->scan_ch_idx + 1u) % ON_AFH_NUM_CHANNELS);
                tune_channel(node, node->scan_ch_idx);
            } else {
                tune_channel(node, on_afh_channel_for(&node->afh, node->current_sf_seq, 0u));
            }
        }

        /* v3.0 2-hop edge node self-healing: if Sub-G proxy dies, detach & re-join new Sub-G */
        if (node->is_sub_node && node->missed_sync_count >= ON_SUB_NODE_DETACH_MISS) {
            node->role                = ON_ROLE_UNJOINED;
            node->short_id            = ON_SHORT_ID_UNASSIGNED;
            node->sync.locked         = false;
            node->sync.rtt_calibrated = false;
            node->missed_sync_count   = 0u;
            node->awaiting_ack        = false;
            node->harq_retry_pending  = false;
            sync_mesh_identity(node);
            emit_event(node, "SUB_NODE_DETACHED", node->parent_id, (int64_t)node->current_sf_seq);
            return;
        }
        elapsed_ns = now_net_ns - node->current_sf_start_net_ns;
    }

    /* Phase 2a: Primary D-TDMA private slot TX (with Piggyback Group-ACK Fast HARQ & v6.0 Epoch-TDMA) */
    if ((node->role == ON_ROLE_T || node->role == ON_ROLE_SUB_G) &&
        !node->is_sub_node && !node->tdma_tx_done_this_sf &&
        !(node->afh.enabled && node->missed_sync_count > 6u) &&
        on_epoch_is_my_sf(node->current_sf_seq, node->epoch_phase, node->epoch_period)) {
        int64_t slot_ns = (int64_t)on_sf_slot_start_us(&node->sf_cfg, node->my_slot) * 1000LL;
        if (elapsed_ns >= slot_ns && elapsed_ns < slot_ns + 80000LL) {
            node->tdma_tx_done_this_sf = true;
            /* v6.8: After Backup_ID failover, 1-hop nodes re-report their own MAC48 once in their private D-TDMA slot */
            if (!node->self_mac_reported_to_g && !node->harq_retry_pending &&
                node->tdma_tx_total > 0u && mac48_nonzero(node->mac48)) {
                on_gw_bridge_msg_t mrep;
                memset(&mrep, 0, sizeof(mrep));
                mrep.bridge_type = (uint8_t)ON_GW_MSG_MAC_REPORT;
                mrep.flags       = 0u;
                mrep.origin_id   = node->short_id;
                mrep.seq         = ++node->data_tx_seq;
                mrep.payload_len = 6u;
                memcpy(mrep.payload, node->mac48, 6u);
                uint8_t mbuf[ON_GW_BRIDGE_HDR_LEN + 6u];
                size_t mlen = 0u;
                if (on_gw_bridge_pack(&mrep, mbuf, sizeof(mbuf), &mlen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                    hdr.is_sub_g   = false;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->parent_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = node->my_slot;
                    send_frame(node, &hdr, mbuf, mlen, now_local_ns);
                    node->self_mac_reported_to_g = true;
                    node->gw_tx_total++;
                    goto tdma_slot_done;
                }
            }
            /* v6.5: Sub-G reports each newly-joined 2-hop child's MAC48 in its own private D-TDMA slot (guaranteed 0 collision!) */
            if (node->role == ON_ROLE_SUB_G && !node->harq_retry_pending && node->tdma_tx_total > 0u) {
                for (uint32_t pi = 0u; pi < ON_MAX_NODES_TABLE; ++pi) {
                    on_peer_entry_t *pe = &node->peers[pi];
                    if (pe->active && pe->is_sub_slot && !pe->mac_reported_to_g &&
                        mac48_nonzero(pe->mac48)) {
                        on_gw_bridge_msg_t mrep;
                        memset(&mrep, 0, sizeof(mrep));
                        mrep.bridge_type = (uint8_t)ON_GW_MSG_MAC_REPORT;
                        mrep.flags       = ON_GW_FLAG_RELAYED_2HOP;
                        mrep.origin_id   = pe->short_id;
                        mrep.seq         = ++node->data_tx_seq;
                        mrep.payload_len = 6u;
                        memcpy(mrep.payload, pe->mac48, 6u);
                        uint8_t mbuf[ON_GW_BRIDGE_HDR_LEN + 6u];
                        size_t mlen = 0u;
                        if (on_gw_bridge_pack(&mrep, mbuf, sizeof(mbuf), &mlen) == ON_OK) {
                            on_mac_hdr_t hdr;
                            memset(&hdr, 0, sizeof(hdr));
                            hdr.version    = ON_PROTO_VERSION;
                            hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                            hdr.is_sub_g   = false;
                            hdr.src_id     = node->short_id;
                            hdr.dst_id     = node->parent_id;
                            hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                            hdr.slot_idx   = node->my_slot;
                            send_frame(node, &hdr, mbuf, mlen, now_local_ns);
                            pe->mac_reported_to_g = true;
                            node->gw_tx_total++;
                            goto tdma_slot_done;
                        }
                    }
                }
            }
            /* v7.0: Anchor periodically flushes overheard peer RSSI batch in its own private D-TDMA slot (0 collisions!) */
            if (node->rssi_report_enabled && !node->harq_retry_pending &&
                node->tdma_tx_total > 0u && node->rssi_obs_count > 0u &&
                (node->current_sf_seq - node->last_rssi_rep_sf) >= 64u) {
                on_gw_bridge_msg_t rrep;
                memset(&rrep, 0, sizeof(rrep));
                rrep.bridge_type = (uint8_t)ON_GW_MSG_RSSI_REPORT;
                rrep.origin_id   = node->short_id;
                rrep.seq         = ++node->data_tx_seq;
                uint8_t out_n = 0u;
                for (uint8_t ri = 0u; ri < node->rssi_obs_count && out_n + 3u <= ON_GW_MAX_PAYLOAD_LEN; ++ri) {
                    rrep.payload[out_n++] = (uint8_t)(node->rssi_obs[ri].short_id & 0xFFu);
                    rrep.payload[out_n++] = (uint8_t)((node->rssi_obs[ri].short_id >> 8) & 0x0Fu);
                    rrep.payload[out_n++] = (uint8_t)node->rssi_obs[ri].rssi_dbm;
                }
                rrep.payload_len = out_n;
                uint8_t rbuf[ON_GW_BRIDGE_HDR_LEN + 24u];
                size_t rlen = 0u;
                if (out_n >= 3u && on_gw_bridge_pack(&rrep, rbuf, sizeof(rbuf), &rlen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                    hdr.is_sub_g   = false;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->parent_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = node->my_slot;
                    send_frame(node, &hdr, rbuf, rlen, now_local_ns);
                    node->last_rssi_rep_sf = node->current_sf_seq;
                    node->rssi_obs_count   = 0u;
                    node->gw_tx_total++;
                    goto tdma_slot_done;
                }
            }
            /* v7.1: Anchor periodically flushes passive BLE iBeacon/Eddystone batch in private D-TDMA slot */
            if (node->rssi_report_enabled && !node->harq_retry_pending &&
                node->tdma_tx_total > 0u && node->ble_obs_count > 0u &&
                (node->current_sf_seq - node->last_ble_rep_sf) >= 64u) {
                on_gw_bridge_msg_t brep;
                memset(&brep, 0, sizeof(brep));
                brep.bridge_type = (uint8_t)ON_GW_MSG_BLE_BEACON_REPORT;
                brep.origin_id   = node->short_id;
                brep.seq         = ++node->data_tx_seq;
                uint8_t out_n = 0u;
                for (uint8_t bi = 0u; bi < node->ble_obs_count && out_n + 9u <= ON_GW_MAX_PAYLOAD_LEN; ++bi) {
                    memcpy(&brep.payload[out_n], node->ble_obs[bi].tag_mac48, 6u);
                    brep.payload[out_n + 6u] = node->ble_obs[bi].source;
                    brep.payload[out_n + 7u] = (uint8_t)node->ble_obs[bi].tx_power_1m_dbm;
                    brep.payload[out_n + 8u] = (uint8_t)node->ble_obs[bi].rssi_dbm;
                    out_n = (uint8_t)(out_n + 9u);
                }
                brep.payload_len = out_n;
                uint8_t bbuf[ON_GW_BRIDGE_HDR_LEN + 40u];
                size_t blen = 0u;
                if (out_n >= 9u && on_gw_bridge_pack(&brep, bbuf, sizeof(bbuf), &blen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                    hdr.is_sub_g   = false;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->parent_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = node->my_slot;
                    send_frame(node, &hdr, bbuf, blen, now_local_ns);
                    node->last_ble_rep_sf = node->current_sf_seq;
                    node->ble_obs_count   = 0u;
                    node->gw_tx_total++;
                    goto tdma_slot_done;
                }
            }
            /* v7.2: Anchor periodically flushes IEEE 802.11bf Wi-Fi CSI motion & spectrum in private D-TDMA slot */
            if (node->rssi_report_enabled && !node->harq_retry_pending &&
                node->tdma_tx_total > 0u && node->csi_obs_pending &&
                (node->current_sf_seq - node->last_csi_rep_sf) >= 64u) {
                on_gw_bridge_msg_t crep;
                memset(&crep, 0, sizeof(crep));
                crep.bridge_type = (uint8_t)ON_GW_MSG_CSI_REPORT;
                crep.origin_id   = node->short_id;
                crep.seq         = ++node->data_tx_seq;
                crep.payload[0]  = node->csi_obs_score;
                memcpy(&crep.payload[1], node->csi_obs_bins, 16u);
                crep.payload_len = 17u;
                uint8_t cbuf[ON_GW_BRIDGE_HDR_LEN + 20u];
                size_t clen = 0u;
                if (on_gw_bridge_pack(&crep, cbuf, sizeof(cbuf), &clen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                    hdr.is_sub_g   = false;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->parent_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = node->my_slot;
                    send_frame(node, &hdr, cbuf, clen, now_local_ns);
                    node->last_csi_rep_sf = node->current_sf_seq;
                    node->csi_obs_pending = false;
                    node->gw_tx_total++;
                    goto tdma_slot_done;
                }
            }
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
                if (node->app_tx_data_len > 0u) {
                    uint8_t n = (node->app_tx_data_len <= 8u) ? node->app_tx_data_len : 8u;
                    memcpy(dp.data, node->app_tx_data, n);
                    dp.data_len = n;
                } else {
                    dp.data[0]  = (uint8_t)(node->current_sf_seq & 0xFFu);
                    dp.data_len = 1u;
                }
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
tdma_slot_done: ;
        }
    }

    /* Phase 2b: Sub-G Staggered Sub-Superframe (Zone 0 + Zone 1 orthogonal FDMA concurrency!) */
    if (node->role == ON_ROLE_SUB_G) {
        int64_t sub_start_ns = (int64_t)on_sf_subg_sync_start_us(&node->sf_cfg) * 1000LL;
        int64_t sub_end_ns   = sub_start_ns + (int64_t)node->sf_cfg.sub_g_len_us * 1000LL;

        if (!node->sub_sync_tx_done_this_sf && elapsed_ns >= sub_start_ns && elapsed_ns < sub_start_ns + 80000LL) {
            node->sub_sync_tx_done_this_sf = true;
            node->last_sub_ack_bitmap      = node->sub_rx_slot_mask_this_sf;
            node->sub_rx_slot_mask_this_sf = 0u;

            /* Zone-1 Sub-G hops onto orthogonal FDMA channel (+8 offset) during Sub-G window */
            if (node->sub_g_zone == 1u && node->hal.radio_set_channel) {
                uint8_t base_ch = on_afh_channel_for(&node->afh, node->current_sf_seq, 0u);
                uint8_t z1_ch   = (uint8_t)((base_ch + 8u) & 0x0Fu);
                node->hal.radio_set_channel(node->hal.user_ctx, z1_ch);
                node->in_subg_z1_ch = true;
            }

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
        } else if (node->in_subg_z1_ch && elapsed_ns >= sub_end_ns) {
            /* Return Zone-1 Sub-G to primary superframe channel after Sub-G window closes */
            uint8_t base_ch = on_afh_channel_for(&node->afh, node->current_sf_seq, 0u);
            node->hal.radio_set_channel(node->hal.user_ctx, base_ch);
            node->in_subg_z1_ch = false;
        }
        if (node->role == ON_ROLE_SUB_G && !node->sub_dl_tx_done_this_sf &&
            node->sub_dl_pending && ((node->current_sf_seq & 1u) != 0u)) {
            int64_t sub_cra_ns = (int64_t)on_sf_subg_cra_start_us(&node->sf_cfg) * 1000LL;
            if (elapsed_ns >= sub_cra_ns && elapsed_ns < sub_cra_ns + 60000LL) {
                node->sub_dl_tx_done_this_sf = true;
                node->sub_dl_pending         = false;
                on_mac_hdr_t hdr;
                memset(&hdr, 0, sizeof(hdr));
                hdr.version    = ON_PROTO_VERSION;
                hdr.type       = node->sub_dl_type;
                hdr.is_sub_g   = true;
                hdr.src_id     = node->short_id;
                hdr.dst_id     = node->sub_dl_dst_id;
                hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                hdr.slot_idx   = 13u;
                send_frame(node, &hdr, node->sub_dl_payload, node->sub_dl_len, now_local_ns);
                if (node->sub_dl_type == ON_FRAME_GW_BRIDGE) node->gw_tx_total++;
                else node->ota_tx_total++;
            }
        }
    }

    /* Phase 2c: 2-hop Edge Node private Sub-TDMA TX inside Sub-G window (with Fast HARQ) */
    if (node->role == ON_ROLE_T && node->is_sub_node && !node->tdma_tx_done_this_sf) {
        int64_t sub_slot_ns = (int64_t)on_sf_subg_slot_start_us(&node->sf_cfg, node->my_slot) * 1000LL;
        if (elapsed_ns >= sub_slot_ns && elapsed_ns < sub_slot_ns + 80000LL) {
            node->tdma_tx_done_this_sf = true;
            /* v7.1: 2-hop Edge Anchor zero-collision overheard RSSI & BLE beacon batch reporting via Sub-G */
            if (node->rssi_report_enabled && !node->harq_retry_pending &&
                node->tdma_tx_total > 0u && node->rssi_obs_count > 0u &&
                (node->current_sf_seq - node->last_rssi_rep_sf) >= 64u) {
                on_gw_bridge_msg_t rrep;
                memset(&rrep, 0, sizeof(rrep));
                rrep.bridge_type = (uint8_t)ON_GW_MSG_RSSI_REPORT;
                rrep.flags       = ON_GW_FLAG_RELAYED_2HOP;
                rrep.origin_id   = node->short_id;
                rrep.seq         = ++node->data_tx_seq;
                uint8_t out_n = 0u;
                for (uint8_t ri = 0u; ri < node->rssi_obs_count && out_n + 3u <= ON_GW_MAX_PAYLOAD_LEN; ++ri) {
                    rrep.payload[out_n++] = (uint8_t)(node->rssi_obs[ri].short_id & 0xFFu);
                    rrep.payload[out_n++] = (uint8_t)((node->rssi_obs[ri].short_id >> 8) & 0x0Fu);
                    rrep.payload[out_n++] = (uint8_t)node->rssi_obs[ri].rssi_dbm;
                }
                rrep.payload_len = out_n;
                uint8_t rbuf[ON_GW_BRIDGE_HDR_LEN + 24u];
                size_t rlen = 0u;
                if (out_n >= 3u && on_gw_bridge_pack(&rrep, rbuf, sizeof(rbuf), &rlen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                    hdr.is_sub_g   = true;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->parent_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = node->my_slot;
                    send_frame(node, &hdr, rbuf, rlen, now_local_ns);
                    node->last_rssi_rep_sf = node->current_sf_seq;
                    node->rssi_obs_count   = 0u;
                    node->gw_tx_total++;
                    goto sub_tdma_slot_done;
                }
            }
            if (node->rssi_report_enabled && !node->harq_retry_pending &&
                node->tdma_tx_total > 0u && node->ble_obs_count > 0u &&
                (node->current_sf_seq - node->last_ble_rep_sf) >= 64u) {
                on_gw_bridge_msg_t brep;
                memset(&brep, 0, sizeof(brep));
                brep.bridge_type = (uint8_t)ON_GW_MSG_BLE_BEACON_REPORT;
                brep.flags       = ON_GW_FLAG_RELAYED_2HOP;
                brep.origin_id   = node->short_id;
                brep.seq         = ++node->data_tx_seq;
                uint8_t out_n = 0u;
                for (uint8_t bi = 0u; bi < node->ble_obs_count && out_n + 9u <= ON_GW_MAX_PAYLOAD_LEN; ++bi) {
                    memcpy(&brep.payload[out_n], node->ble_obs[bi].tag_mac48, 6u);
                    brep.payload[out_n + 6u] = node->ble_obs[bi].source;
                    brep.payload[out_n + 7u] = (uint8_t)node->ble_obs[bi].tx_power_1m_dbm;
                    brep.payload[out_n + 8u] = (uint8_t)node->ble_obs[bi].rssi_dbm;
                    out_n = (uint8_t)(out_n + 9u);
                }
                brep.payload_len = out_n;
                uint8_t bbuf[ON_GW_BRIDGE_HDR_LEN + 40u];
                size_t blen = 0u;
                if (out_n >= 9u && on_gw_bridge_pack(&brep, bbuf, sizeof(bbuf), &blen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                    hdr.is_sub_g   = true;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->parent_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = node->my_slot;
                    send_frame(node, &hdr, bbuf, blen, now_local_ns);
                    node->last_ble_rep_sf = node->current_sf_seq;
                    node->ble_obs_count   = 0u;
                    node->gw_tx_total++;
                    goto sub_tdma_slot_done;
                }
            }
            if (node->rssi_report_enabled && !node->harq_retry_pending &&
                node->tdma_tx_total > 0u && node->csi_obs_pending &&
                (node->current_sf_seq - node->last_csi_rep_sf) >= 64u) {
                on_gw_bridge_msg_t crep;
                memset(&crep, 0, sizeof(crep));
                crep.bridge_type = (uint8_t)ON_GW_MSG_CSI_REPORT;
                crep.flags       = ON_GW_FLAG_RELAYED_2HOP;
                crep.origin_id   = node->short_id;
                crep.seq         = ++node->data_tx_seq;
                crep.payload[0]  = node->csi_obs_score;
                memcpy(&crep.payload[1], node->csi_obs_bins, 16u);
                crep.payload_len = 17u;
                uint8_t cbuf[ON_GW_BRIDGE_HDR_LEN + 20u];
                size_t clen = 0u;
                if (on_gw_bridge_pack(&crep, cbuf, sizeof(cbuf), &clen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                    hdr.is_sub_g   = true;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->parent_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = node->my_slot;
                    send_frame(node, &hdr, cbuf, clen, now_local_ns);
                    node->last_csi_rep_sf = node->current_sf_seq;
                    node->csi_obs_pending = false;
                    node->gw_tx_total++;
                    goto sub_tdma_slot_done;
                }
            }
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
                if (node->app_tx_data_len > 0u) {
                    uint8_t n = (node->app_tx_data_len <= 8u) ? node->app_tx_data_len : 8u;
                    memcpy(dp.data, node->app_tx_data, n);
                    dp.data_len = n;
                } else {
                    dp.data[0]  = 0xE8u; /* edge sensor marker */
                    dp.data_len = 1u;
                }
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
sub_tdma_slot_done: ;
        }
    }

    /* Phase 3: C-RA Contention Micro-Slot (JOIN_REQ, RTT_REQ, or Emergency Alert) */
    if (!node->cra_tx_done_this_sf && !node->service_dl_reserved_this_sf) {
        /* Deterministic stagger by MAC byte[5] (mod 2 for Sub-CRA 2-hop, mod 7 for 1-hop C-RA) */
        uint32_t turn_mod = node->is_sub_node ? 2u : 7u;
        uint32_t my_turn  = node->is_sub_node ? ((node->mac48[5] + 1u) % 2u) : (node->mac48[5] % 7u);
        int64_t cra_ns = node->is_sub_node
            ? (int64_t)on_sf_subg_cra_start_us(&node->sf_cfg) * 1000LL
            : (int64_t)on_sf_cra_slot_start_us(&node->sf_cfg, (uint8_t)(node->mac48[5] & 1u)) * 1000LL;

        if (elapsed_ns >= cra_ns && elapsed_ns < cra_ns + 60000LL) {
            if (node->role == ON_ROLE_UNJOINED && ((node->current_sf_seq % turn_mod) == my_turn)) {
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
                       !node->hal.tx_timing_best_effort &&
                       !(node->ota && node->ota->rx.state != ON_OTA_STATE_IDLE) &&
                       node->current_sf_seq >= node->rtt_next_sf &&
                       ((node->current_sf_seq % turn_mod) == my_turn)) {
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

    /* v5.0 Service Uplink Slot: OTA Status/NACK & Public Wi-Fi/BLE Gateway Uplink
     * - 1-hop / Sub-G: Odd SF at C-RA slot 1 (3420 us) -> zero collision with G's Even-SF Downlink
     * - 2-hop Edge:    Even SF at Sub-CRA (2940 us)    -> zero collision with Sub-G's Odd-SF Relay */
    if (!node->service_tx_done_this_sf && !node->cra_tx_done_this_sf &&
        (node->role == ON_ROLE_T || node->role == ON_ROLE_SUB_G) &&
        node->short_id != ON_SHORT_ID_UNASSIGNED) {
        bool my_sf_parity = node->is_sub_node ? ((node->current_sf_seq & 1u) == 0u)
                                              : ((node->current_sf_seq & 1u) != 0u);
        int64_t ul_ns = node->is_sub_node
            ? (int64_t)on_sf_subg_cra_start_us(&node->sf_cfg) * 1000LL
            : (int64_t)(node->sf_cfg.cra_start_us + 300u) * 1000LL;
        if (my_sf_parity && elapsed_ns >= ul_ns && elapsed_ns < ul_ns + 60000LL) {
            uint32_t half_sf = node->current_sf_seq >> 1u;
            uint32_t ul_mod  = node->is_sub_node ? 4u : 12u;
            uint32_t ul_turn = (uint32_t)(node->my_slot % ul_mod);

            if (node->role == ON_ROLE_SUB_G && node->sub_ul_pending &&
                ((half_sf % 12u) == 11u)) {
                node->service_tx_done_this_sf = true;
                node->sub_ul_pending          = false;
                on_mac_hdr_t hdr;
                memset(&hdr, 0, sizeof(hdr));
                hdr.version    = ON_PROTO_VERSION;
                hdr.type       = node->sub_ul_type;
                hdr.is_sub_g   = false;
                hdr.src_id     = node->short_id;
                hdr.dst_id     = node->parent_id;
                hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                hdr.slot_idx   = 15u;
                send_frame(node, &hdr, node->sub_ul_payload, node->sub_ul_len, now_local_ns);
                if (node->sub_ul_type == ON_FRAME_GW_BRIDGE) node->gw_tx_total++;
                else node->ota_tx_total++;
            } else if (node->mesh_tx_pending && ((half_sf % ul_mod) == ul_turn)) {
                node->service_tx_done_this_sf = true;
                node->mesh_tx_pending         = false;
                uint8_t mbuf[ON_MAX_PAYLOAD_LEN];
                size_t mlen = 0u;
                if (on_mesh_pkt_pack(&node->mesh_tx_pkt, mbuf, sizeof(mbuf), &mlen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_MESH;
                    hdr.is_sub_g   = node->is_sub_node;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->mesh_tx_pkt.next_hop_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = 15u;
                    send_frame(node, &hdr, mbuf, mlen, now_local_ns);
                    node->mesh_tx_total++;
                }
            } else if (node->gw_ul_pending && ((half_sf % ul_mod) == ul_turn)) {
                node->service_tx_done_this_sf = true;
                node->gw_ul_pending           = false;
                uint8_t pbuf[ON_MAX_PAYLOAD_LEN];
                size_t plen = 0u;
                if (on_gw_bridge_pack(&node->gw_ul_msg, pbuf, sizeof(pbuf), &plen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_GW_BRIDGE;
                    hdr.is_sub_g   = node->is_sub_node;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->parent_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = 15u;
                    send_frame(node, &hdr, pbuf, plen, now_local_ns);
                    node->gw_tx_total++;
                }
            } else if (node->ota &&
                       (node->ota->rx.status_dirty ||
                        (node->ota->rx.state == ON_OTA_STATE_RECEIVING &&
                         node->current_sf_seq >= node->ota->rx.last_status_sf + 8u)) &&
                       ((half_sf % ul_mod) == ul_turn)) {
                node->service_tx_done_this_sf = true;
                node->ota->rx.status_dirty    = false;
                node->ota->rx.last_status_sf  = node->current_sf_seq;
                on_ota_status_t st;
                on_ota_rx_build_status(&node->ota->rx, node->short_id, &st);
                uint8_t sbuf[ON_OTA_STATUS_WIRE_LEN];
                size_t slen = 0u;
                if (on_ota_status_pack(&st, sbuf, sizeof(sbuf), &slen) == ON_OK) {
                    on_mac_hdr_t hdr;
                    memset(&hdr, 0, sizeof(hdr));
                    hdr.version    = ON_PROTO_VERSION;
                    hdr.type       = (uint8_t)ON_FRAME_OTA_STATUS;
                    hdr.is_sub_g   = node->is_sub_node;
                    hdr.src_id     = node->short_id;
                    hdr.dst_id     = node->parent_id;
                    hdr.sf_seq_low = (uint8_t)(node->current_sf_seq & 0x0Fu);
                    hdr.slot_idx   = 15u;
                    send_frame(node, &hdr, sbuf, slen, now_local_ns);
                    node->ota_tx_total++;
                }
            }
        }
    }

    /* v5.0 Synchronized Superframe-Aligned OTA Activation */
    if (node->ota && on_ota_rx_step_commit(&node->ota->rx, node->current_sf_seq)) {
        emit_event(node, "OTA_APPLIED", node->ota->rx.meta.ota_id, (int64_t)node->ota->rx.applied_version);
    }

    /* Phase 4: ACT / QUIET Silent Guard Band -> Synchronized Hardware Action Trigger */
    int64_t act_off_ns = (int64_t)node->sf_cfg.act_fire_off_us * 1000LL;
    if (node->act_lookahead.pending && now_local_ns >= node->act_lookahead.deadline_local_ns) {
        node->act_lookahead.pending = false;
        node->act_cmd = node->act_lookahead.next_level;
        node->act_lookahead.level_sf = node->act_lookahead.target_sf;
        if (node->current_sf_seq == node->act_lookahead.target_sf) node->act_fired_this_sf = true;
        fire_act(node, node->act_lookahead.target_sf, now_local_ns, true);
    } else if (!node->act_fired_this_sf && elapsed_ns >= act_off_ns) {
        node->act_fired_this_sf = true;
        fire_act(node, node->current_sf_seq, now_local_ns, false);
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
        if (node->missed_sync_count > 6u && (node->missed_sync_count & 1u) != 0u) {
            tune_channel(node, node->scan_ch_idx);
        } else {
            tune_channel(node, on_afh_channel_for(&node->afh, node->current_sf_seq + 1u, 0u));
        }
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
    int64_t best_local = on_sync_net_to_local(&node->sync, best_net);
    if (node->act_lookahead.pending && node->act_lookahead.deadline_local_ns < best_local) {
        best_local = node->act_lookahead.deadline_local_ns;
    }
    return best_local;
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
        /* 2-hop edge node wakes RX during Sub-G SYNC window (with 40 us pre-guard for clock jitter) */
        uint32_t sub0 = node->sf_cfg.sub_g_start_us;
        uint32_t pre  = (sub0 >= 40u) ? (sub0 - 40u) : 0u;
        if (off_us >= pre && off_us < sub0 + 270u) {
            return true;
        }
        /* v5.0/v5.1 OTA / Gateway / Topology-query 2-hop node also listens in Sub-CRA downlink window */
        if (node->service_dl_reserved_this_sf || node->ota != NULL ||
            node->gw != NULL || node->topo_query_in_flight) {
            uint32_t sub_cra = on_sf_subg_cra_start_us(&node->sf_cfg);
            uint32_t cra_pre = (sub_cra >= 40u) ? (sub_cra - 40u) : 0u;
            if (off_us >= cra_pre && off_us < sub0 + node->sf_cfg.sub_g_len_us + 250u) {
                return true;
            }
        }
        return false;
    }

    /* 1-hop T node & Sub-G wake RX during primary SYNC/MAP (0..270 us).
     * Sleeping Epoch-TDMA sensors (epoch_period > 1) only wake on their scheduled epoch
     * once NCO frequency is locked (lock_count >= 4), achieving < 0.5% radio duty cycle! */
    if (off_us < 270u || off_us >= (uint32_t)(node->sf_cfg.t_sf_us - 40u)) {
        if (node->epoch_period > 1u && node->sync.lock_count >= 4u &&
            node->role == ON_ROLE_T && !node->is_sub_node) {
            uint32_t target_sf = (off_us >= (uint32_t)(node->sf_cfg.t_sf_us - 40u))
                                 ? (node->current_sf_seq + 1u) : node->current_sf_seq;
            return on_epoch_is_my_sf(target_sf, node->epoch_phase, node->epoch_period);
        }
        return true;
    }

    /* v5.0/v5.1/v6.0 OTA / Gateway / Topology / Mesh 1-hop node listens in C-RA service window */
    if ((node->service_dl_reserved_this_sf || node->ota != NULL || node->gw != NULL ||
         node->mesh != NULL || node->topo_query_in_flight || node->sub_dl_listen_sfs > 0u) &&
        off_us + 40u >= node->sf_cfg.cra_start_us &&
        off_us < node->sf_cfg.act_start_us + 100u) {
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

/* ========================================================================== */
/* v5.0 Node OTA & Public Wi-Fi/BLE Gateway Bridge Public Helpers             */
/* ========================================================================== */

void on_node_attach_ota(on_node_t *node, on_ota_ctx_t *ota) {
    if (!node) return;
    node->ota = ota;
}

void on_node_attach_gateway(on_node_t *node, on_gw_ctx_t *gw) {
    if (!node) return;
    node->gw = gw;
}

on_err_t on_node_ota_start_mem(on_node_t *node, uint16_t ota_id, uint32_t fw_version,
                               uint16_t target_id, uint8_t hw_compat, uint8_t flags,
                               const uint8_t *image_buf, uint32_t image_size,
                               uint16_t chunk_size) {
    if (!node || !node->ota || node->role != ON_ROLE_G) return ON_ERR_NOT_READY;
    const uint8_t *auth_key = node->sec.enabled ? node->sec.master_key : NULL;
    on_err_t rc = on_ota_tx_start_mem(&node->ota->tx, ota_id, fw_version,
                                      target_id, hw_compat, flags,
                                      image_buf, image_size, chunk_size, auth_key);
    if (rc == ON_OK) {
        node->ota->tx.meta_repeat_cnt = 2u;
        emit_event(node, "OTA_TX_STARTED", ota_id, (int64_t)image_size);
    }
    return rc;
}

on_err_t on_node_ota_commit(on_node_t *node, uint32_t lead_sfs, uint8_t flags) {
    if (!node || !node->ota || node->role != ON_ROLE_G || !node->ota->tx.active) {
        return ON_ERR_NOT_READY;
    }
    if (lead_sfs < 2u) lead_sfs = 2u;
    node->ota->tx.commit_sf_seq = node->current_sf_seq + lead_sfs;
    node->ota->tx.commit_sent   = false;
    node->ota->tx.meta.flags   |= flags;
    emit_event(node, "OTA_COMMIT_SCHED", node->ota->tx.meta.ota_id,
               (int64_t)node->ota->tx.commit_sf_seq);
    return ON_OK;
}

on_err_t on_node_send_gw_uplink(on_node_t *node, on_gw_msg_type_t type,
                                const uint8_t *payload, uint8_t len) {
    if (!node || (!payload && len > 0u) || len > ON_GW_MAX_PAYLOAD_LEN) {
        return ON_ERR_INVALID_ARG;
    }
    if (node->role == ON_ROLE_UNJOINED || node->short_id == ON_SHORT_ID_UNASSIGNED) {
        return ON_ERR_NOT_READY;
    }
    memset(&node->gw_ul_msg, 0, sizeof(node->gw_ul_msg));
    node->gw_ul_msg.bridge_type = (uint8_t)type;
    node->gw_ul_msg.flags       = node->is_sub_node ? ON_GW_FLAG_RELAYED_2HOP : 0u;
    node->gw_ul_msg.origin_id   = node->short_id;
    node->gw_ul_msg.seq         = ++node->data_tx_seq;
    node->gw_ul_msg.payload_len = len;
    if (len > 0u) memcpy(node->gw_ul_msg.payload, payload, len);

    if (node->role == ON_ROLE_G && node->gw) {
        return on_gw_push_uplink(node->gw, &node->gw_ul_msg) ? ON_OK : ON_ERR_NO_MEM;
    }
    node->gw_ul_pending = true;
    return ON_OK;
}

on_err_t on_node_send_gw_downlink(on_node_t *node, uint16_t target_id,
                                  on_gw_msg_type_t type, const uint8_t *payload, uint8_t len) {
    if (!node || !node->gw || node->role != ON_ROLE_G ||
        (!payload && len > 0u) || len > ON_GW_MAX_PAYLOAD_LEN) {
        return ON_ERR_INVALID_ARG;
    }
    on_gw_bridge_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.bridge_type = (uint8_t)type;
    msg.origin_id   = target_id & ON_SHORT_ID_MASK;
    msg.seq         = ++node->gw->tx_seq;
    msg.payload_len = len;
    if (len > 0u) memcpy(msg.payload, payload, len);
    return on_gw_push_downlink(node->gw, &msg) ? ON_OK : ON_ERR_NO_MEM;
}

/* ========================================================================== */
/* v5.1 Application Sensor Payload & Cluster Topology / Live Data Export      */
/* ========================================================================== */

void on_node_set_tx_payload(on_node_t *node, const uint8_t *data, uint8_t len) {
    if (!node) return;
    if (!data || len == 0u) {
        node->app_tx_data_len = 0u;
        return;
    }
    uint8_t n = (len <= 8u) ? len : 8u;
    memcpy(node->app_tx_data, data, n);
    node->app_tx_data_len = n;
}

on_err_t on_node_request_cluster_snapshot(on_node_t *node) {
    if (!node) return ON_ERR_INVALID_ARG;
    if (node->role == ON_ROLE_G) {
        node->cluster_snapshot_valid = true;
        return ON_OK;
    }
    if (node->role == ON_ROLE_UNJOINED || node->short_id == ON_SHORT_ID_UNASSIGNED) {
        return ON_ERR_NOT_READY;
    }
    node->topo_query_in_flight   = true;
    node->topo_rx_pages_mask     = 0u;
    node->topo_rx_total_pages    = 0u;
    node->cluster_snapshot_valid = false;
    return on_node_send_gw_uplink(node, ON_GW_MSG_TOPO_REQ, NULL, 0u);
}

size_t on_node_get_cluster_snapshot(const on_node_t *node, on_cluster_snapshot_t *out) {
    if (!node || !out) return 0u;
    memset(out, 0, sizeof(*out));
    out->sf_seq      = node->current_sf_seq;
    out->backup_id   = node->backup_id;
    out->sub_g_id    = node->sub_g_id;
    out->sub_g_id_2  = node->sub_g_id_2;
    out->tdma_bitmap = node->tdma_bitmap;
    out->channel_map = node->afh.channel_map;

    uint8_t count = 0u;

    if (node->role == ON_ROLE_G) {
        out->g_id             = node->short_id;
        out->is_remote_cached = false;

        /* Entry 0: Primary G itself */
        on_cluster_node_t *g = &out->nodes[count++];
        g->active        = true;
        memcpy(g->mac48, node->mac48, 6u);
        g->short_id      = node->short_id;
        g->parent_id     = ON_SHORT_ID_UNASSIGNED;
        g->role          = (uint8_t)ON_ROLE_G;
        g->hop_depth     = 0u;
        g->slot_idx      = 0u;
        g->capabilities  = node->capabilities;
        g->last_seen_sf  = node->current_sf_seq;
        g->last_skew_ns  = 0;
        g->rx_packets    = node->tdma_rx_total;
        g->last_data_len = node->app_tx_data_len;
        if (g->last_data_len > 0u) memcpy(g->last_data, node->app_tx_data, g->last_data_len);
        if (node->ota) g->ota_state = (uint8_t)node->ota->rx.state;

        /* Copy all tracked 1-hop & 2-hop cluster nodes (skip stale 2-hop > 2*lease) */
        uint32_t max_age = (node->peer_lease_timeout_sf > 0u) ? (node->peer_lease_timeout_sf * 2u) : 60u;
        for (uint32_t i = 0u; i < ON_MAX_CLUSTER_NODES && count < ON_MAX_CLUSTER_NODES; ++i) {
            const on_cluster_node_t *c = &node->cluster_tbl[i];
            if (!c->active || c->short_id == node->short_id) continue;
            if (node->current_sf_seq > c->last_seen_sf + max_age) continue;
            out->nodes[count]           = *c;
            out->nodes[count].is_backup = (c->short_id == node->backup_id);
            if (c->hop_depth == 1u) {
                out->nodes[count].role = (c->short_id == node->sub_g_id || c->short_id == node->sub_g_id_2)
                                         ? (uint8_t)ON_ROLE_SUB_G : (uint8_t)ON_ROLE_T;
            }
            count++;
        }
        /* Include any joined 1-hop peer that hasn't sent its first TDMA data packet yet */
        for (uint32_t i = 0u; i < ON_MAX_NODES_TABLE && count < ON_MAX_CLUSTER_NODES; ++i) {
            const on_peer_entry_t *pe = &node->peers[i];
            if (!pe->active || pe->is_sub_slot) continue;
            bool exists = false;
            for (uint8_t j = 0u; j < count; ++j) {
                if (out->nodes[j].short_id == pe->short_id) { exists = true; break; }
            }
            if (!exists) {
                on_cluster_node_t *dst = &out->nodes[count++];
                dst->active       = true;
                memcpy(dst->mac48, pe->mac48, 6u);
                dst->short_id     = pe->short_id;
                dst->parent_id    = node->short_id;
                dst->hop_depth    = 1u;
                dst->role         = (pe->short_id == node->sub_g_id || pe->short_id == node->sub_g_id_2)
                                    ? (uint8_t)ON_ROLE_SUB_G : (uint8_t)ON_ROLE_T;
                dst->slot_idx     = pe->slot_idx;
                dst->capabilities = pe->capabilities;
                dst->is_backup    = (pe->short_id == node->backup_id);
                dst->last_seen_sf = pe->last_seen_sf;
                dst->last_skew_ns = pe->last_skew_ns;
                dst->rx_packets   = pe->rx_packets;
            }
        }
    } else if (node->cluster_snapshot_valid) {
        /* Cached full-cluster snapshot fetched from Primary G over OpenNear air */
        out->g_id             = node->cluster_g_id;
        out->is_remote_cached = true;
        for (uint32_t i = 0u; i < ON_MAX_CLUSTER_NODES && count < ON_MAX_CLUSTER_NODES; ++i) {
            if (node->cluster_tbl[i].active) {
                out->nodes[count++] = node->cluster_tbl[i];
            }
        }
    } else {
        /* Local-only neighborhood view (self + Sub-G children if Sub-G) */
        out->g_id             = node->is_sub_node ? ON_SHORT_ID_G_DEFAULT : node->parent_id;
        out->is_remote_cached = false;
        if (node->short_id != ON_SHORT_ID_UNASSIGNED) {
            on_cluster_node_t *me = &out->nodes[count++];
            me->active        = true;
            memcpy(me->mac48, node->mac48, 6u);
            me->short_id      = node->short_id;
            me->parent_id     = node->parent_id;
            me->role          = (uint8_t)node->role;
            me->hop_depth     = node->is_sub_node ? 2u : 1u;
            me->slot_idx      = node->my_slot;
            me->capabilities  = node->capabilities;
            me->is_backup     = (node->short_id == node->backup_id);
            me->last_seen_sf  = node->current_sf_seq;
            me->last_skew_ns  = (int16_t)(node->sync.last_err_ns > 32000 ? 32000 :
                                         (node->sync.last_err_ns < -32000 ? -32000 : node->sync.last_err_ns));
            me->last_data_len = node->app_tx_data_len;
            if (me->last_data_len > 0u) memcpy(me->last_data, node->app_tx_data, me->last_data_len);
            if (node->ota) me->ota_state = (uint8_t)node->ota->rx.state;
        }
        if (node->role == ON_ROLE_SUB_G) {
            for (uint32_t i = 0u; i < ON_MAX_NODES_TABLE && count < ON_MAX_CLUSTER_NODES; ++i) {
                const on_peer_entry_t *pe = &node->peers[i];
                if (!pe->active || !pe->is_sub_slot) continue;
                on_cluster_node_t *ch = &out->nodes[count++];
                ch->active       = true;
                memcpy(ch->mac48, pe->mac48, 6u);
                ch->short_id     = pe->short_id;
                ch->parent_id    = node->short_id;
                ch->role         = (uint8_t)ON_ROLE_T;
                ch->hop_depth    = 2u;
                ch->slot_idx     = pe->slot_idx;
                ch->last_seen_sf = pe->last_seen_sf;
                ch->last_skew_ns = pe->last_skew_ns;
                ch->rx_packets   = pe->rx_packets;
            }
        }
    }

    out->active_count = count;
    return (size_t)count;
}

static const char *role_str(uint8_t role) {
    switch ((on_role_t)role) {
        case ON_ROLE_G:     return "G";
        case ON_ROLE_SUB_G: return "SUB_G";
        case ON_ROLE_T:     return "T";
        default:            return "UNJOINED";
    }
}

size_t on_node_format_cluster_json(const on_node_t *node, bool include_data,
                                   char *out_buf, size_t out_cap) {
    if (!node || !out_buf || out_cap < 128u) return 0u;
    on_cluster_snapshot_t snap;
    on_node_get_cluster_snapshot(node, &snap);

    static const char kHex[] = "0123456789ABCDEF";
    int pos = snprintf(out_buf, out_cap,
                       "{\"sf\":%u,\"g\":\"0x%03X\",\"backup\":\"0x%03X\",\"sub_g\":\"0x%03X\","
                       "\"ch_map\":\"0x%04X\",\"cached\":%u,\"count\":%u,\"nodes\":[",
                       (unsigned)snap.sf_seq,
                       (unsigned)snap.g_id,
                       (unsigned)snap.backup_id,
                       (unsigned)snap.sub_g_id,
                       (unsigned)snap.channel_map,
                       snap.is_remote_cached ? 1u : 0u,
                       (unsigned)snap.active_count);
    if (pos <= 0 || (size_t)pos >= out_cap) return 0u;

    for (uint8_t i = 0u; i < snap.active_count; ++i) {
        const on_cluster_node_t *c = &snap.nodes[i];
        char dhex[17];
        uint8_t dlen = (c->last_data_len <= 8u) ? c->last_data_len : 8u;
        for (uint8_t k = 0u; k < dlen; ++k) {
            dhex[k * 2u]      = kHex[(c->last_data[k] >> 4) & 0x0Fu];
            dhex[k * 2u + 1u] = kHex[c->last_data[k] & 0x0Fu];
        }
        dhex[dlen * 2u] = '\0';

        char mac_s[18];
        format_mac48(c->mac48, mac_s);

        int n;
        if (include_data) {
            n = snprintf(out_buf + pos, out_cap - (size_t)pos,
                         "%s{\"id\":\"0x%03X\",\"mac\":\"%s\",\"parent\":\"0x%03X\",\"hop\":%u,\"role\":\"%s\","
                         "\"slot\":%u,\"skew_ns\":%d,\"rx\":%u,\"seq\":%u,\"ota\":%u,\"data\":\"%s\"}",
                         (i > 0u) ? "," : "",
                         (unsigned)c->short_id, mac_s, (unsigned)c->parent_id,
                         (unsigned)c->hop_depth, role_str(c->role),
                         (unsigned)c->slot_idx, (int)c->last_skew_ns,
                         (unsigned)c->rx_packets, (unsigned)c->last_seq,
                         (unsigned)c->ota_state, dhex);
        } else {
            n = snprintf(out_buf + pos, out_cap - (size_t)pos,
                         "%s{\"id\":\"0x%03X\",\"mac\":\"%s\",\"parent\":\"0x%03X\",\"hop\":%u,\"role\":\"%s\","
                         "\"slot\":%u,\"skew_ns\":%d,\"ota\":%u}",
                         (i > 0u) ? "," : "",
                         (unsigned)c->short_id, mac_s, (unsigned)c->parent_id,
                         (unsigned)c->hop_depth, role_str(c->role),
                         (unsigned)c->slot_idx, (int)c->last_skew_ns,
                         (unsigned)c->ota_state);
        }
        if (n <= 0 || (size_t)(pos + n + 4) >= out_cap) break;
        pos += n;
    }

    int tail = snprintf(out_buf + pos, out_cap - (size_t)pos, "]}");
    if (tail <= 0 || (size_t)(pos + tail) >= out_cap) return 0u;
    return (size_t)(pos + tail);
}

uint16_t on_node_find_short_id_by_mac(const on_node_t *node, const uint8_t mac48[6]) {
    if (!node || !mac48 || !mac48_nonzero(mac48)) return ON_SHORT_ID_UNASSIGNED;
    if (memcmp(node->mac48, mac48, 6u) == 0) {
        return node->short_id & ON_SHORT_ID_MASK;
    }
    for (uint32_t i = 0u; i < ON_MAX_CLUSTER_NODES; ++i) {
        if (node->cluster_tbl[i].active &&
            memcmp(node->cluster_tbl[i].mac48, mac48, 6u) == 0) {
            return node->cluster_tbl[i].short_id & ON_SHORT_ID_MASK;
        }
    }
    for (uint32_t i = 0u; i < ON_MAX_NODES_TABLE; ++i) {
        if (node->peers[i].active &&
            memcmp(node->peers[i].mac48, mac48, 6u) == 0) {
            return node->peers[i].short_id & ON_SHORT_ID_MASK;
        }
    }
    return ON_SHORT_ID_UNASSIGNED;
}

bool on_node_find_mac_by_short_id(const on_node_t *node, uint16_t short_id, uint8_t mac_out[6]) {
    if (!node || !mac_out || short_id == ON_SHORT_ID_UNASSIGNED) return false;
    uint16_t sid = short_id & ON_SHORT_ID_MASK;
    if ((node->short_id & ON_SHORT_ID_MASK) == sid && mac48_nonzero(node->mac48)) {
        memcpy(mac_out, node->mac48, 6u);
        return true;
    }
    for (uint32_t i = 0u; i < ON_MAX_CLUSTER_NODES; ++i) {
        if (node->cluster_tbl[i].active &&
            (node->cluster_tbl[i].short_id & ON_SHORT_ID_MASK) == sid &&
            mac48_nonzero(node->cluster_tbl[i].mac48)) {
            memcpy(mac_out, node->cluster_tbl[i].mac48, 6u);
            return true;
        }
    }
    for (uint32_t i = 0u; i < ON_MAX_NODES_TABLE; ++i) {
        if (node->peers[i].active &&
            (node->peers[i].short_id & ON_SHORT_ID_MASK) == sid &&
            mac48_nonzero(node->peers[i].mac48)) {
            memcpy(mac_out, node->peers[i].mac48, 6u);
            return true;
        }
    }
    return false;
}

void on_node_enable_rssi_anchor_report(on_node_t *node, bool enable) {
    if (!node) return;
    node->rssi_report_enabled = enable;
    if (!enable) node->rssi_obs_count = 0u;
}

void on_node_set_rssi_observer(on_node_t *node,
                               void (*cb)(void *ctx, uint16_t anchor_id,
                                          uint16_t target_id, int8_t rssi_dbm),
                               void *user_ctx) {
    if (!node) return;
    node->on_rssi_observed = cb;
    node->rssi_obs_ctx     = user_ctx;
}

/* ========================================================================== */
/* v6.0 Leaderless Auto-Boot, 256-Node Epoch-TDMA & 3~6-Hop Hybrid Mesh       */
/* ========================================================================== */

void on_node_enable_leaderless_boot(on_node_t *node, uint32_t base_timeout_ms) {
    if (!node) return;
    node->leaderless_boot_enabled    = (base_timeout_ms > 0u);
    node->leaderless_base_timeout_ns = (int64_t)base_timeout_ms * 1000000LL;
    node->boot_listen_start_ns       = 0;
}

void on_node_set_epoch_schedule(on_node_t *node, uint8_t epoch_phase, uint8_t epoch_period) {
    if (!node) return;
    node->epoch_phase  = epoch_phase;
    node->epoch_period = epoch_period;
    if (epoch_period > 1u) {
        /* Sleeping Epoch-TDMA nodes must not be elected as always-on Backup_ID */
        node->capabilities &= (uint8_t)(~0x01u);
        uint32_t min_lease = (uint32_t)epoch_period * 3u;
        if (node->peer_lease_timeout_sf < min_lease) {
            node->peer_lease_timeout_sf = min_lease;
        }
    }
}

void on_node_attach_mesh(on_node_t *node, on_mesh_ctx_t *mesh) {
    if (!node) return;
    node->mesh = mesh;
    sync_mesh_identity(node);
}

on_err_t on_node_send_mesh_uplink(on_node_t *node, const uint8_t *payload, uint8_t len) {
    if (!node || !node->mesh) return ON_ERR_NOT_READY;
    on_err_t rc = on_mesh_build_uplink(node->mesh, ON_MESH_DATA_UP, payload, len, &node->mesh_tx_pkt);
    if (rc == ON_OK) {
        node->mesh_tx_pending = true;
    }
    return rc;
}

on_err_t on_node_send_mesh_topo_mac(on_node_t *node) {
    if (!node || !node->mesh) return ON_ERR_NOT_READY;
    on_err_t rc = on_mesh_build_uplink(node->mesh, ON_MESH_TOPO_REPORT, node->mac48, 6u, &node->mesh_tx_pkt);
    if (rc == ON_OK) {
        node->mesh_tx_pending = true;
    }
    return rc;
}

on_err_t on_node_send_mesh_downlink(on_node_t *node, uint16_t target_id,
                                    const uint8_t *payload, uint8_t len) {
    if (!node || !node->mesh) return ON_ERR_NOT_READY;
    on_err_t rc = on_mesh_build_downlink(node->mesh, target_id, payload, len, &node->mesh_tx_pkt);
    if (rc == ON_OK) {
        node->mesh_tx_pending = true;
    }
    return rc;
}

void on_node_set_ble_beacon_observer(on_node_t *node,
                                     void (*cb)(void *ctx, uint16_t anchor_id,
                                                const uint8_t tag_mac48[6], uint8_t source,
                                                int8_t tx_power_1m_dbm, int8_t rssi_dbm),
                                     void *user_ctx) {
    if (!node) return;
    node->on_ble_beacon_observed = cb;
    node->ble_obs_ctx            = user_ctx;
}

void on_node_report_ble_beacon(on_node_t *node, const uint8_t tag_mac48[6],
                               uint8_t source, int8_t tx_power_1m_dbm, int8_t rssi_dbm) {
    if (!node || !tag_mac48 || rssi_dbm >= 0 || rssi_dbm < -120) return;
    if ((tag_mac48[0] | tag_mac48[1] | tag_mac48[2] |
         tag_mac48[3] | tag_mac48[4] | tag_mac48[5]) == 0u) {
        return;
    }
    if (node->role == ON_ROLE_G) {
        if (node->on_ble_beacon_observed) {
            node->on_ble_beacon_observed(node->ble_obs_ctx, node->short_id,
                                         tag_mac48, source, tx_power_1m_dbm, rssi_dbm);
        }
        return;
    }
    if (!node->rssi_report_enabled) return;
    for (uint8_t i = 0u; i < node->ble_obs_count; ++i) {
        if (memcmp(node->ble_obs[i].tag_mac48, tag_mac48, 6u) == 0) {
            node->ble_obs[i].source          = source;
            node->ble_obs[i].tx_power_1m_dbm = tx_power_1m_dbm;
            node->ble_obs[i].rssi_dbm        = rssi_dbm;
            return;
        }
    }
    uint8_t idx = (node->ble_obs_count < 4u) ? node->ble_obs_count++ : 0u;
    memcpy(node->ble_obs[idx].tag_mac48, tag_mac48, 6u);
    node->ble_obs[idx].source          = source;
    node->ble_obs[idx].tx_power_1m_dbm = tx_power_1m_dbm;
    node->ble_obs[idx].rssi_dbm        = rssi_dbm;
}

void on_node_set_csi_observer(on_node_t *node,
                              void (*cb)(void *ctx, uint16_t anchor_id,
                                         uint8_t motion_score, const uint8_t bins16[16]),
                              void *user_ctx) {
    if (!node) return;
    node->on_csi_observed = cb;
    node->csi_obs_ctx     = user_ctx;
}

void on_node_report_csi(on_node_t *node, uint8_t motion_score, const uint8_t bins16[16]) {
    if (!node) return;
    if (motion_score > 100u) motion_score = 100u;
    uint8_t synth[16];
    const uint8_t *use_bins = bins16;
    if (!use_bins) {
        for (uint8_t i = 0u; i < 16u; ++i) {
            int32_t v = (int32_t)motion_score + (int32_t)((i & 3u) * 4u) - 6;
            if (v < 2) v = 2;
            if (v > 100) v = 100;
            synth[i] = (uint8_t)v;
        }
        use_bins = synth;
    }
    if (node->role == ON_ROLE_G) {
        if (node->on_csi_observed) {
            node->on_csi_observed(node->csi_obs_ctx, node->short_id, motion_score, use_bins);
        }
        return;
    }
    node->rssi_report_enabled = true;
    node->csi_obs_score = motion_score;
    memcpy(node->csi_obs_bins, use_bins, 16u);
    node->csi_obs_pending = true;
}
