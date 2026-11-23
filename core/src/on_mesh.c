#include "on_mesh.h"
#include <string.h>

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* ========================================================================== */
/* ON_FRAME_MESH Wire Codec                                                   */
/* ========================================================================== */

on_err_t on_mesh_pkt_pack(const on_mesh_pkt_t *p, uint8_t *out, size_t cap, size_t *len) {
    if (!p || !out || !len) return ON_ERR_INVALID_ARG;
    if (p->payload_len > ON_MESH_MAX_PAYLOAD_LEN) return ON_ERR_TOO_LONG;
    size_t total = ON_MESH_HDR_LEN + 1u + (size_t)p->payload_len;
    if (cap < total) return ON_ERR_TOO_LONG;

    out[0] = p->mesh_type;
    out[1] = p->ttl;
    out[2] = p->hop_count;
    out[3] = p->flags;
    put_u16(out + 4,  p->src_id & ON_SHORT_ID_MASK);
    put_u16(out + 6,  p->dst_id & ON_SHORT_ID_MASK);
    put_u16(out + 8,  p->next_hop_id & ON_SHORT_ID_MASK);
    put_u16(out + 10, p->msg_id);
    out[12] = p->payload_len;
    if (p->payload_len > 0u) {
        memcpy(out + 13, p->payload, p->payload_len);
    }
    *len = total;
    return ON_OK;
}

on_err_t on_mesh_pkt_unpack(const uint8_t *in, size_t len, on_mesh_pkt_t *p) {
    if (!in || !p) return ON_ERR_INVALID_ARG;
    if (len < ON_MESH_HDR_LEN + 1u) return ON_ERR_TOO_SHORT;

    p->mesh_type   = in[0];
    p->ttl         = in[1];
    p->hop_count   = in[2];
    p->flags       = in[3];
    p->src_id      = get_u16(in + 4)  & ON_SHORT_ID_MASK;
    p->dst_id      = get_u16(in + 6)  & ON_SHORT_ID_MASK;
    p->next_hop_id = get_u16(in + 8)  & ON_SHORT_ID_MASK;
    p->msg_id      = get_u16(in + 10);
    p->payload_len = in[12];
    if (p->payload_len > ON_MESH_MAX_PAYLOAD_LEN ||
        len < ON_MESH_HDR_LEN + 1u + (size_t)p->payload_len) {
        return ON_ERR_TOO_SHORT;
    }
    if (p->payload_len > 0u) {
        memcpy(p->payload, in + 13, p->payload_len);
    }
    return ON_OK;
}

/* ========================================================================== */
/* Hybrid 3~6 Hop Mesh Router                                                 */
/* ========================================================================== */

void on_mesh_init(on_mesh_ctx_t *m, uint16_t my_id, uint8_t initial_hop_depth,
                  uint16_t parent_id) {
    if (!m) return;
    memset(m, 0, sizeof(*m));
    m->my_id             = my_id & ON_SHORT_ID_MASK;
    m->root_id           = ON_SHORT_ID_G_DEFAULT;
    m->my_hop_depth      = initial_hop_depth;
    m->parent_id         = parent_id & ON_SHORT_ID_MASK;
    m->parent_timeout_sf = 30u;
}

static bool dedup_seen_or_insert(on_mesh_ctx_t *m, uint16_t src_id, uint16_t msg_id) {
    for (uint32_t i = 0u; i < ON_MESH_DEDUP_SIZE; ++i) {
        if (m->dedup[i].used && m->dedup[i].src_id == src_id && m->dedup[i].msg_id == msg_id) {
            return true;
        }
    }
    m->dedup[m->dedup_cursor].used   = true;
    m->dedup[m->dedup_cursor].src_id = src_id;
    m->dedup[m->dedup_cursor].msg_id = msg_id;
    m->dedup_cursor = (uint8_t)((m->dedup_cursor + 1u) % ON_MESH_DEDUP_SIZE);
    return false;
}

static void learn_reverse_route(on_mesh_ctx_t *m, uint16_t dest_id, uint16_t next_hop_id,
                                uint8_t hop_dist, uint32_t current_sf) {
    if (dest_id == ON_SHORT_ID_UNASSIGNED || dest_id == m->my_id ||
        next_hop_id == ON_SHORT_ID_UNASSIGNED) {
        return;
    }
    on_mesh_route_t *slot = NULL;
    on_mesh_route_t *oldest = &m->routes[0];
    for (uint32_t i = 0u; i < ON_MESH_MAX_ROUTES; ++i) {
        if (m->routes[i].active && m->routes[i].dest_id == dest_id) {
            slot = &m->routes[i];
            break;
        }
        if (!m->routes[i].active && !slot) {
            slot = &m->routes[i];
        }
        if (m->routes[i].last_seen_sf < oldest->last_seen_sf) {
            oldest = &m->routes[i];
        }
    }
    if (!slot) slot = oldest;
    slot->active       = true;
    slot->dest_id      = dest_id;
    slot->next_hop_id  = next_hop_id;
    slot->hop_dist     = hop_dist;
    slot->last_seen_sf = current_sf;
}

uint16_t on_mesh_lookup_next_hop(const on_mesh_ctx_t *m, uint16_t dest_id) {
    if (!m || dest_id == ON_SHORT_ID_UNASSIGNED) return ON_SHORT_ID_UNASSIGNED;
    if (dest_id == ON_SHORT_ID_BROADCAST) return ON_SHORT_ID_BROADCAST;
    for (uint32_t i = 0u; i < ON_MESH_MAX_ROUTES; ++i) {
        if (m->routes[i].active && m->routes[i].dest_id == dest_id) {
            return m->routes[i].next_hop_id;
        }
    }
    return ON_SHORT_ID_UNASSIGNED;
}

static void elect_best_parent(on_mesh_ctx_t *m, uint32_t current_sf) {
    if (m->my_hop_depth == 0u) return; /* Root G never elects a parent */
    int best_idx = -1;
    for (uint32_t i = 0u; i < ON_MESH_MAX_NEIGHBORS; ++i) {
        on_mesh_neighbor_t *nb = &m->neighbors[i];
        if (!nb->active) continue;
        if (current_sf > nb->last_seen_sf + m->parent_timeout_sf) {
            nb->active = false;
            continue;
        }
        if (nb->hop_to_root >= ON_MESH_MAX_HOPS) continue;
        if (best_idx < 0) {
            best_idx = (int)i;
        } else {
            const on_mesh_neighbor_t *cur = &m->neighbors[best_idx];
            if (nb->hop_to_root < cur->hop_to_root ||
                (nb->hop_to_root == cur->hop_to_root && nb->lqi > cur->lqi)) {
                best_idx = (int)i;
            }
        }
    }
    if (best_idx >= 0) {
        uint16_t new_parent = m->neighbors[best_idx].neighbor_id;
        uint8_t  new_hop    = (uint8_t)(m->neighbors[best_idx].hop_to_root + 1u);
        if (m->parent_id != ON_SHORT_ID_UNASSIGNED && m->parent_id != new_parent) {
            m->parent_switches++;
        }
        m->parent_id    = new_parent;
        m->my_hop_depth = new_hop;
    } else {
        m->parent_id    = ON_SHORT_ID_UNASSIGNED;
        m->my_hop_depth = 0xFFu;
    }
}

void on_mesh_build_beacon(on_mesh_ctx_t *m, on_mesh_pkt_t *out) {
    if (!m || !out) return;
    memset(out, 0, sizeof(*out));
    out->mesh_type   = (uint8_t)ON_MESH_BEACON;
    out->ttl         = 1u;
    out->hop_count   = m->my_hop_depth;
    out->src_id      = m->my_id;
    out->dst_id      = ON_SHORT_ID_BROADCAST;
    out->next_hop_id = ON_SHORT_ID_BROADCAST;
    out->msg_id      = ++m->tx_msg_seq;
    out->payload_len = 2u;
    put_u16(out->payload, m->root_id);
}

void on_mesh_on_beacon(on_mesh_ctx_t *m, uint16_t sender_id, uint8_t sender_hop,
                       uint8_t lqi, uint32_t current_sf) {
    if (!m || sender_id == ON_SHORT_ID_UNASSIGNED || sender_id == m->my_id) return;
    if (sender_hop >= ON_MESH_MAX_HOPS || m->my_hop_depth == 0u) return;

    /* Avoid routing loops: never adopt a neighbor deeper than or equal to ourselves */
    on_mesh_neighbor_t *slot = NULL;
    for (uint32_t i = 0u; i < ON_MESH_MAX_NEIGHBORS; ++i) {
        if (m->neighbors[i].active && m->neighbors[i].neighbor_id == sender_id) {
            slot = &m->neighbors[i];
            break;
        }
        if (!m->neighbors[i].active && !slot) {
            slot = &m->neighbors[i];
        }
    }
    if (!slot) slot = &m->neighbors[0];
    slot->active       = true;
    slot->neighbor_id  = sender_id;
    slot->hop_to_root  = sender_hop;
    slot->lqi          = lqi;
    slot->last_seen_sf = current_sf;

    elect_best_parent(m, current_sf);
}

bool on_mesh_check_parent_health(on_mesh_ctx_t *m, uint32_t current_sf) {
    if (!m || m->my_hop_depth == 0u) return false;
    uint16_t old_parent = m->parent_id;
    elect_best_parent(m, current_sf);
    return (m->parent_id != old_parent);
}

on_err_t on_mesh_build_uplink(on_mesh_ctx_t *m, on_mesh_type_t type,
                              const uint8_t *payload, uint8_t len,
                              on_mesh_pkt_t *out) {
    if (!m || !out || (!payload && len > 0u) || len > ON_MESH_MAX_PAYLOAD_LEN) {
        return ON_ERR_INVALID_ARG;
    }
    if (m->parent_id == ON_SHORT_ID_UNASSIGNED || m->my_hop_depth == 0xFFu) {
        return ON_ERR_NOT_READY;
    }
    memset(out, 0, sizeof(*out));
    out->mesh_type   = (uint8_t)type;
    out->ttl         = ON_MESH_MAX_HOPS;
    out->hop_count   = 1u;
    out->src_id      = m->my_id;
    out->dst_id      = m->root_id;
    out->next_hop_id = m->parent_id;
    out->msg_id      = ++m->tx_msg_seq;
    out->payload_len = len;
    if (len > 0u) memcpy(out->payload, payload, len);
    dedup_seen_or_insert(m, out->src_id, out->msg_id);
    return ON_OK;
}

on_err_t on_mesh_build_downlink(on_mesh_ctx_t *m, uint16_t target_id,
                                const uint8_t *payload, uint8_t len,
                                on_mesh_pkt_t *out) {
    if (!m || !out || (!payload && len > 0u) || len > ON_MESH_MAX_PAYLOAD_LEN) {
        return ON_ERR_INVALID_ARG;
    }
    uint16_t nh = on_mesh_lookup_next_hop(m, target_id);
    memset(out, 0, sizeof(*out));
    out->mesh_type   = (uint8_t)ON_MESH_CMD_DOWN;
    out->ttl         = ON_MESH_MAX_HOPS;
    out->hop_count   = 1u;
    out->src_id      = m->my_id;
    out->dst_id      = target_id & ON_SHORT_ID_MASK;
    if (nh != ON_SHORT_ID_UNASSIGNED) {
        out->next_hop_id = nh;
    } else {
        out->next_hop_id = ON_SHORT_ID_BROADCAST;
        out->flags      |= ON_MESH_FLAG_FLOOD_FALLBACK;
    }
    out->msg_id      = ++m->tx_msg_seq;
    out->payload_len = len;
    if (len > 0u) memcpy(out->payload, payload, len);
    dedup_seen_or_insert(m, out->src_id, out->msg_id);
    return ON_OK;
}

on_mesh_action_t on_mesh_process_rx(on_mesh_ctx_t *m, uint16_t prev_hop_id,
                                    const on_mesh_pkt_t *in, uint32_t current_sf,
                                    on_mesh_pkt_t *out_fwd) {
    if (!m || !in) return ON_MESH_ACT_DROP;

    /* Handle neighbor discovery beacons */
    if (in->mesh_type == ON_MESH_BEACON) {
        on_mesh_on_beacon(m, in->src_id, in->hop_count, 200u, current_sf);
        return ON_MESH_ACT_DROP;
    }

    if (in->src_id == m->my_id) return ON_MESH_ACT_DROP;
    if (dedup_seen_or_insert(m, in->src_id, in->msg_id)) {
        m->dedup_drops++;
        return ON_MESH_ACT_DROP;
    }

    /* Learn reverse route toward src_id via prev_hop_id for future downlink routing! */
    if (in->mesh_type == ON_MESH_DATA_UP || in->mesh_type == ON_MESH_TOPO_REPORT) {
        learn_reverse_route(m, in->src_id, prev_hop_id, in->hop_count, current_sf);
    }

    bool addressed_to_me = (in->dst_id == m->my_id);
    bool is_bcast        = (in->dst_id == ON_SHORT_ID_BROADCAST);
    bool should_relay    = (in->next_hop_id == m->my_id || in->next_hop_id == ON_SHORT_ID_BROADCAST);

    if (addressed_to_me) {
        return ON_MESH_ACT_DELIVER;
    }

    if (!should_relay || !out_fwd) {
        return is_bcast ? ON_MESH_ACT_DELIVER : ON_MESH_ACT_DROP;
    }

    if (in->ttl <= 1u || in->hop_count >= ON_MESH_MAX_HOPS) {
        m->ttl_drops++;
        return is_bcast ? ON_MESH_ACT_DELIVER : ON_MESH_ACT_DROP;
    }

    *out_fwd = *in;
    out_fwd->ttl--;
    out_fwd->hop_count++;

    if (in->mesh_type == ON_MESH_DATA_UP || in->mesh_type == ON_MESH_TOPO_REPORT) {
        if (m->parent_id == ON_SHORT_ID_UNASSIGNED) {
            return ON_MESH_ACT_DROP;
        }
        out_fwd->next_hop_id = m->parent_id;
        m->relayed_up_total++;
        return ON_MESH_ACT_FORWARD;
    }

    /* Downlink tree-routed or controlled-flood forwarding */
    uint16_t nh = on_mesh_lookup_next_hop(m, in->dst_id);
    if (nh != ON_SHORT_ID_UNASSIGNED) {
        out_fwd->next_hop_id = nh;
        out_fwd->flags      &= (uint8_t)(~ON_MESH_FLAG_FLOOD_FALLBACK);
    } else if ((in->flags & ON_MESH_FLAG_FLOOD_FALLBACK) != 0u || is_bcast) {
        out_fwd->next_hop_id = ON_SHORT_ID_BROADCAST;
    } else {
        return ON_MESH_ACT_DROP;
    }
    m->relayed_down_total++;
    return is_bcast ? ON_MESH_ACT_DELIV_FWD : ON_MESH_ACT_FORWARD;
}

/* ========================================================================== */
/* 256-Node Multi-Epoch TDMA Slot Scheduler                                   */
/* ========================================================================== */

static uint8_t normalize_period(uint8_t p) {
    if (p <= 1u)  return 1u;
    if (p <= 2u)  return 2u;
    if (p <= 4u)  return 4u;
    if (p <= 8u)  return 8u;
    if (p <= 16u) return 16u;
    return 32u;
}

static uint32_t build_epoch_pattern(uint8_t phase, uint8_t period) {
    uint32_t mask = 0u;
    for (uint8_t e = phase; e < ON_EPOCH_MAX_PERIOD; e = (uint8_t)(e + period)) {
        mask |= (1UL << e);
    }
    return mask;
}

void on_epoch_sched_init(on_epoch_sched_t *s) {
    if (!s) return;
    memset(s, 0, sizeof(*s));
}

on_err_t on_epoch_alloc(on_epoch_sched_t *s, uint16_t short_id, uint8_t requested_period,
                        uint32_t current_sf, on_epoch_grant_t *out_grant) {
    if (!s || short_id == ON_SHORT_ID_UNASSIGNED || !out_grant) return ON_ERR_INVALID_ARG;
    uint8_t period = normalize_period(requested_period);

    /* If already allocated, return existing grant cleanly */
    for (uint32_t i = 0u; i < ON_EPOCH_MAX_DEVICES; ++i) {
        if (s->grants[i].active && s->grants[i].short_id == short_id) {
            s->grants[i].last_seen_sf = current_sf;
            *out_grant = s->grants[i];
            return ON_OK;
        }
    }

    /* Find first free (slot, phase) whose periodic mask has zero bit overlap */
    for (uint8_t slot = 0u; slot < ON_EPOCH_MAX_SLOTS; ++slot) {
        for (uint8_t phase = 0u; phase < period; ++phase) {
            uint32_t pat = build_epoch_pattern(phase, period);
            if ((s->slot_epoch_mask[slot] & pat) == 0u) {
                for (uint32_t i = 0u; i < ON_EPOCH_MAX_DEVICES; ++i) {
                    if (!s->grants[i].active) {
                        s->slot_epoch_mask[slot] |= pat;
                        s->grants[i].active       = true;
                        s->grants[i].short_id     = short_id & ON_SHORT_ID_MASK;
                        s->grants[i].slot_idx     = slot;
                        s->grants[i].epoch_phase  = phase;
                        s->grants[i].epoch_period = period;
                        s->grants[i].last_seen_sf = current_sf;
                        s->active_count++;
                        *out_grant = s->grants[i];
                        return ON_OK;
                    }
                }
            }
        }
    }
    return ON_ERR_NO_MEM;
}

bool on_epoch_release(on_epoch_sched_t *s, uint16_t short_id) {
    if (!s) return false;
    for (uint32_t i = 0u; i < ON_EPOCH_MAX_DEVICES; ++i) {
        if (s->grants[i].active && s->grants[i].short_id == short_id) {
            uint32_t pat = build_epoch_pattern(s->grants[i].epoch_phase, s->grants[i].epoch_period);
            s->slot_epoch_mask[s->grants[i].slot_idx] &= ~pat;
            s->grants[i].active = false;
            if (s->active_count > 0u) s->active_count--;
            return true;
        }
    }
    return false;
}

bool on_epoch_is_my_sf(uint32_t current_sf, uint8_t epoch_phase, uint8_t epoch_period) {
    if (epoch_period <= 1u) return true;
    return ((current_sf % epoch_period) == (epoch_phase % epoch_period));
}

uint32_t on_epoch_next_tx_sf(uint32_t current_sf, uint8_t epoch_phase, uint8_t epoch_period) {
    if (epoch_period <= 1u) return current_sf;
    uint32_t rem = current_sf % epoch_period;
    uint32_t target = epoch_phase % epoch_period;
    if (rem <= target) return current_sf + (target - rem);
    return current_sf + (epoch_period - rem) + target;
}

double on_epoch_duty_cycle_pct(uint8_t epoch_period) {
    /* 160 us active slot + 240 us SYNC RX every epoch_period * 4000 us */
    uint8_t p = normalize_period(epoch_period);
    return 100.0 * (400.0 / (4000.0 * (double)p));
}
