#include "sim_world.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    sim_world_t *world;
    uint32_t     node_idx;
} sim_hal_ctx_t;

static sim_hal_ctx_t s_hal_ctxs[SIM_MAX_NODES];

static uint64_t xorshift64(uint64_t *s) {
    uint64_t x = *s;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *s = x;
    return x;
}

static int64_t sim_rand_jitter(sim_world_t *w) {
    if (w->hw_jitter_bound_ns <= 0) return 0;
    uint64_t r = xorshift64(&w->rng_state);
    int64_t span = 2 * w->hw_jitter_bound_ns + 1;
    return (int64_t)(r % (uint64_t)span) - w->hw_jitter_bound_ns;
}

int64_t sim_phys_to_local_ns(const sim_world_t *w, uint32_t idx, int64_t phys_ns) {
    double scale = 1.0 + w->drift_ppm[idx] * 1e-6;
    return w->boot_offset_ns[idx] + (int64_t)llround((double)phys_ns * scale);
}

int64_t sim_local_to_phys_ns(const sim_world_t *w, uint32_t idx, int64_t local_ns) {
    double scale = 1.0 + w->drift_ppm[idx] * 1e-6;
    return (int64_t)llround((double)(local_ns - w->boot_offset_ns[idx]) / scale);
}

static int64_t hal_now_ns(void *ctx) {
    sim_hal_ctx_t *sc = (sim_hal_ctx_t *)ctx;
    return sim_phys_to_local_ns(sc->world, sc->node_idx, sc->world->phys_now_ns);
}

static void pcap_write_u32(FILE *fp, uint32_t v) {
    uint8_t b[4] = { (uint8_t)(v & 0xFFu), (uint8_t)((v >> 8) & 0xFFu),
                     (uint8_t)((v >> 16) & 0xFFu), (uint8_t)((v >> 24) & 0xFFu) };
    fwrite(b, 1u, 4u, fp);
}

static void pcap_write_u16(FILE *fp, uint16_t v) {
    uint8_t b[2] = { (uint8_t)(v & 0xFFu), (uint8_t)((v >> 8) & 0xFFu) };
    fwrite(b, 1u, 2u, fp);
}

bool sim_world_start_pcap(sim_world_t *w, const char *pcap_path) {
    if (!w || !pcap_path) return false;
    w->pcap_fp = fopen(pcap_path, "wb");
    if (!w->pcap_fp) return false;
    /* Standard libpcap global header (little-endian, LINKTYPE_USER0 = 147) */
    pcap_write_u32(w->pcap_fp, 0xA1B2C3D4UL);
    pcap_write_u16(w->pcap_fp, 2u);
    pcap_write_u16(w->pcap_fp, 4u);
    pcap_write_u32(w->pcap_fp, 0u);
    pcap_write_u32(w->pcap_fp, 0u);
    pcap_write_u32(w->pcap_fp, 256u);
    pcap_write_u32(w->pcap_fp, 147u); /* DLT_USER0 for opennear_dissector.lua */
    w->pcap_pkts = 0u;
    return true;
}

void sim_world_stop_pcap(sim_world_t *w) {
    if (w && w->pcap_fp) {
        fclose(w->pcap_fp);
        w->pcap_fp = NULL;
    }
}

static void pcap_log_packet(sim_world_t *w, uint32_t src_idx, uint8_t ch_idx,
                            const uint8_t *frame, size_t len, int64_t phys_ns) {
    if (!w || !w->pcap_fp) return;
    uint64_t us_total = (phys_ns > 0) ? (uint64_t)(phys_ns / 1000LL) : 0ULL;
    uint32_t ts_sec  = (uint32_t)(us_total / 1000000ULL);
    uint32_t ts_usec = (uint32_t)(us_total % 1000000ULL);
    uint32_t rec_len = (uint32_t)(4u + len);
    pcap_write_u32(w->pcap_fp, ts_sec);
    pcap_write_u32(w->pcap_fp, ts_usec);
    pcap_write_u32(w->pcap_fp, rec_len);
    pcap_write_u32(w->pcap_fp, rec_len);
    uint8_t meta[4] = { ch_idx, (uint8_t)src_idx, 0u, 0u };
    fwrite(meta, 1u, 4u, w->pcap_fp);
    fwrite(frame, 1u, len, w->pcap_fp);
    w->pcap_pkts++;
}

static on_err_t hal_tx_raw(void *ctx, const uint8_t *frame, size_t len, int64_t tx_at_local_ns) {
    sim_hal_ctx_t *sc = (sim_hal_ctx_t *)ctx;
    sim_world_t *w = sc->world;
    uint32_t src = sc->node_idx;
    if (!w->alive[src] || len > ON_MAX_FRAME_LEN) return ON_ERR_INVALID_ARG;

    on_mac_hdr_t hdr;
    if (on_hdr_unpack(frame, &hdr) != ON_OK) return ON_ERR_HDR_CRC;

    /* Optional interference injection: drop primary SYNC_MAP in configured SF window (all-node mode) */
    if (hdr.type == ON_FRAME_SYNC_MAP &&
        w->blackout_node_idx < 0 &&
        w->drop_sync_start_sf > 0u &&
        w->nodes[src].current_sf_seq >= w->drop_sync_start_sf &&
        w->nodes[src].current_sf_seq <= w->drop_sync_end_sf) {
        w->sync_drops_injected++;
        return ON_OK;
    }

    /* Optional single-packet impulse drop injection (for Fast HARQ verification) */
    if (hdr.type == ON_FRAME_TDMA_DATA &&
        w->impulse_drop_node_idx >= 0 &&
        src == (uint32_t)w->impulse_drop_node_idx &&
        w->nodes[src].current_sf_seq == w->impulse_drop_sf) {
        w->impulse_drop_node_idx = -1; /* fire once */
        return ON_OK;
    }

    /* Keep capturing the latest encrypted 1-hop TDMA_DATA wire packet for live replay/tamper tests */
    if (hdr.sec_en && hdr.type == ON_FRAME_TDMA_DATA && !hdr.is_sub_g && hdr.dst_id == w->nodes[0].short_id) {
        w->captured_sec_pkt = true;
        memcpy(w->cap_frame, frame, len);
        w->cap_len = len;
        w->cap_src_idx = src;
    }

    /* Check if this TX carries a relayed edge packet to G (plaintext path) */
    if (!hdr.sec_en && hdr.type == ON_FRAME_TDMA_DATA && len > ON_MAC_HDR_LEN + ON_PAYLOAD_CRC_LEN) {
        on_data_payload_t dp;
        if (on_data_payload_unpack(frame + ON_MAC_HDR_LEN, len - ON_MAC_HDR_LEN - ON_PAYLOAD_CRC_LEN, &dp) == ON_OK) {
            if ((dp.flags & 0x02u) && hdr.dst_id == w->nodes[0].short_id) {
                w->edge_relay_received_by_g = true;
            }
        }
    }

    int64_t tx_phys = (tx_at_local_ns > 0)
                    ? sim_local_to_phys_ns(w, src, tx_at_local_ns)
                    : w->phys_now_ns;
    /* Honor hardware PPI scheduled TX instant within +/- 100 us of current sim tick */
    if (tx_phys < w->phys_now_ns - 100000LL) {
        tx_phys = w->phys_now_ns;
    }
    /* LE 2 Mbps short-preamble air duration: 8 us preamble + 4 us per byte */
    int64_t air_dur_ns = 8000LL + (int64_t)len * 4000LL;

    for (uint32_t i = 0; i < SIM_MAX_IN_FLIGHT; ++i) {
        if (!w->air[i].active) {
            w->air[i].active           = true;
            w->air[i].src_idx          = src;
            w->air[i].ch_idx           = w->cur_ch[src];
            memcpy(w->air[i].frame, frame, len);
            w->air[i].len              = len;
            w->air[i].tx_start_phys_ns = tx_phys;
            w->air[i].tx_end_phys_ns   = tx_phys + air_dur_ns;
            w->air[i].frame_type       = hdr.type;
            w->air[i].is_sub_g         = hdr.is_sub_g;
            w->total_tx_packets++;
            if (hdr.type == ON_FRAME_SYNC_MAP && w->nodes[src].current_sf_seq < SIM_MAX_SF_HISTORY) {
                w->sf_ch_hist[w->nodes[src].current_sf_seq]  = w->cur_ch[src];
                w->sf_map_hist[w->nodes[src].current_sf_seq] = w->nodes[src].afh.channel_map;
            }
            pcap_log_packet(w, src, w->cur_ch[src], frame, len, tx_phys);
            return ON_OK;
        }
    }
    return ON_ERR_NO_MEM;
}

static void hal_radio_set_rx(void *ctx, bool enable) {
    (void)ctx;
    (void)enable;
}

static void hal_radio_sleep(void *ctx) {
    (void)ctx;
}

static void hal_radio_set_channel(void *ctx, uint8_t ch_idx) {
    sim_hal_ctx_t *sc = (sim_hal_ctx_t *)ctx;
    sc->world->cur_ch[sc->node_idx] = ch_idx & 0x0Fu;
}

static void hal_get_mac48(void *ctx, uint8_t mac_out[6]) {
    sim_hal_ctx_t *sc = (sim_hal_ctx_t *)ctx;
    mac_out[0] = 0x02u; /* Locally administered OUI */
    mac_out[1] = 0x4Fu; /* 'O' */
    mac_out[2] = 0x4Eu; /* 'N' */
    mac_out[3] = 0x00u;
    mac_out[4] = 0x01u;
    mac_out[5] = (uint8_t)(sc->node_idx + 1u);
}

static void hal_on_act(void *ctx, uint32_t sf_seq, uint8_t act_cmd, int64_t local_ns) {
    sim_hal_ctx_t *sc = (sim_hal_ctx_t *)ctx;
    sim_world_t *w = sc->world;
    uint32_t idx = sc->node_idx;

    /* Compute exact true physical trigger time from node's hardware compare target */
    int64_t exact_net_ns = w->nodes[idx].current_sf_start_net_ns +
                           (int64_t)w->nodes[idx].sf_cfg.act_fire_off_us * 1000LL;
    int64_t exact_loc_ns = on_sync_net_to_local(&w->nodes[idx].sync, exact_net_ns);
    (void)local_ns;
    int64_t exact_phys_ns = sim_local_to_phys_ns(w, idx, exact_loc_ns);

    sim_act_record_t *rec = NULL;
    for (uint32_t i = 0; i < w->act_hist_count; ++i) {
        if (w->act_hist[i].sf_seq == sf_seq) {
            rec = &w->act_hist[i];
            break;
        }
    }
    if (!rec && w->act_hist_count < SIM_MAX_SF_HISTORY) {
        rec = &w->act_hist[w->act_hist_count++];
        memset(rec, 0, sizeof(*rec));
        rec->sf_seq = sf_seq;
        rec->cmd_consistent = true;
    }
    if (!rec) return;

    rec->fired_mask |= (1u << idx);
    rec->fire_phys_ns[idx] = exact_phys_ns;
    rec->fire_cmd[idx]     = act_cmd;

    /* Recompute max skew across all fired nodes in this superframe */
    int64_t min_t = INT64_MAX;
    int64_t max_t = INT64_MIN;
    int first_cmd = -1;
    rec->cmd_consistent = true;
    for (uint32_t i = 0; i < w->num_nodes; ++i) {
        if ((rec->fired_mask & (1u << i)) != 0u) {
            if (rec->fire_phys_ns[i] < min_t) min_t = rec->fire_phys_ns[i];
            if (rec->fire_phys_ns[i] > max_t) max_t = rec->fire_phys_ns[i];
            if (first_cmd < 0) {
                first_cmd = rec->fire_cmd[i];
            } else if (rec->fire_cmd[i] != (uint8_t)first_cmd) {
                rec->cmd_consistent = false;
            }
        }
    }
    rec->max_skew_ns = (max_t > min_t) ? (max_t - min_t) : 0;
}

static void hal_on_event(void *ctx, const char *ev, uint32_t a1, int64_t a2) {
    sim_hal_ctx_t *sc = (sim_hal_ctx_t *)ctx;
    sim_world_t *w = sc->world;
    if (strcmp(ev, "BACKUP_TAKEOVER") == 0) {
        w->failover_sf_seq   = (uint32_t)a2;
        w->failover_new_g_id = (uint16_t)a1;
    } else if (strcmp(ev, "CRA_ALERT_RX") == 0) {
        w->alert_received_by_g = true;
    }
    if (w->verbose) {
        printf("[t=%7.3f ms] %-4s EVENT %-18s arg1=0x%03X arg2=%lld\n",
               (double)w->phys_now_ns * 1e-6,
               w->names[sc->node_idx], ev, a1, (long long)a2);
    }
}

void sim_world_init(sim_world_t *w, uint64_t seed) {
    memset(w, 0, sizeof(*w));
    w->rng_state             = seed ? seed : 0x5EED1234CAFEBEEFULL;
    w->hw_jitter_bound_ns    = 35; /* +/- 35 ns hardware correlator capture noise (< 0.1 us spec) */
    w->blackout_node_idx     = -1;
    w->kill_node_idx         = -1;
    w->impulse_drop_node_idx = -1;
}

void sim_world_setup_10node_demo(sim_world_t *w, bool pre_registered_fast) {
    static const char *kNames[10] = {
        "G", "T1_BK", "T2_SG", "T3", "T4", "T5", "T6", "T7", "T8_E1", "T9_E2"
    };
    static const double kDrifts[10] = {
        0.0, +18.5, -15.2, +24.0, -21.5, +11.8, -9.4, +19.7, -23.1, +16.4
    };
    static const int64_t kOffsets[10] = {
        0, 1234567, 8910111, 3322110, 9988776, 4455667, 7766554, 2109876, 6543210, 5151515
    };

    w->num_nodes = 10u;

    /* Build RF reachability & propagation/frontend delay matrix */
    for (uint32_t i = 0; i < 10u; ++i) {
        for (uint32_t j = 0; j < 10u; ++j) {
            if (i == j) continue;
            bool i_edge = (i >= 8u);
            bool j_edge = (j >= 8u);
            if (!i_edge && !j_edge) {
                /* 1-hop cluster: G + T1..T7 all hear each other */
                w->can_hear[i][j] = true;
                w->link_delay_ns[i][j] = 180 + (int64_t)((i + j) * 25);
            } else if ((i == 2u && j_edge) || (j == 2u && i_edge)) {
                /* Wall-blocked 2-hop edge nodes T8 & T9 ONLY hear Sub-G proxy T2! */
                w->can_hear[i][j] = true;
                w->link_delay_ns[i][j] = 320 + (int64_t)((i + j) * 20);
            } else {
                w->can_hear[i][j] = false;
                w->link_delay_ns[i][j] = 0;
            }
        }
    }

    for (uint32_t i = 0; i < 10u; ++i) {
        strncpy(w->names[i], kNames[i], sizeof(w->names[i]) - 1u);
        w->alive[i]          = true;
        w->drift_ppm[i]      = kDrifts[i];
        w->boot_offset_ns[i] = kOffsets[i];

        s_hal_ctxs[i].world    = w;
        s_hal_ctxs[i].node_idx = i;

        on_hal_ops_t hal;
        memset(&hal, 0, sizeof(hal));
        hal.user_ctx       = &s_hal_ctxs[i];
        hal.now_ns         = hal_now_ns;
        hal.tx_raw         = hal_tx_raw;
        hal.radio_set_rx      = hal_radio_set_rx;
        hal.radio_sleep       = hal_radio_sleep;
        hal.radio_set_channel = hal_radio_set_channel;
        hal.on_act_trigger = hal_on_act;
        hal.get_mac48      = hal_get_mac48;
        hal.on_event       = hal_on_event;

        uint8_t caps = 0x00u;
        if (i == 1u) caps = 0x01u; /* T1_BK: designated hot-standby Backup_ID capable */
        if (i == 2u) caps = 0x02u; /* T2_SG: designated Sub-G relay proxy capable */

        on_node_init(&w->nodes[i], &hal, (i == 0u) ? ON_ROLE_G : ON_ROLE_UNJOINED, caps);

        if (pre_registered_fast && i >= 1u && i <= 7u) {
            uint8_t mac[6];
            hal_get_mac48(&s_hal_ctxs[i], mac);
            uint16_t sid = (uint16_t)(0x001u + i);
            on_node_register_static_peer(&w->nodes[0], mac, sid, (uint8_t)(i - 1u), caps, false);
            w->nodes[i].short_id    = sid;
            w->nodes[i].my_slot     = (uint8_t)(i - 1u);
            w->nodes[i].is_sub_node = false;
            w->nodes[i].parent_id   = ON_SHORT_ID_G_DEFAULT;
            w->nodes[i].role        = (i == 2u) ? ON_ROLE_SUB_G : ON_ROLE_T;
            w->nodes[i].rtt_next_sf = 2u + i;
        }
    }

    if (pre_registered_fast) {
        /* Designate T1 (0x002) as Backup_ID and T2 (0x003) as Sub-G proxy */
        on_node_set_backup_and_subg(&w->nodes[0], 0x002u, 0x003u);
    }
}

/* Deliver completed in-flight RF packets & check real overlapping RF collisions */
static void deliver_air_packets(sim_world_t *w) {
    for (uint32_t i = 0; i < SIM_MAX_IN_FLIGHT; ++i) {
        sim_air_pkt_t *p = &w->air[i];
        if (!p->active) continue;
        if (w->phys_now_ns < p->tx_end_phys_ns) continue;

        /* Check if this logical channel is wiped out by a Wi-Fi co-channel jammer */
        if ((w->jammed_ch_mask & (uint16_t)(1u << (p->ch_idx & 0x0Fu))) != 0u) {
            w->jammed_drops++;
            p->active = false;
            continue;
        }

        /* Deliver packet to every alive node on the SAME channel that can hear p->src_idx */
        for (uint32_t dst = 0; dst < w->num_nodes; ++dst) {
            if (!w->alive[dst] || dst == p->src_idx) continue;
            if (!w->can_hear[p->src_idx][dst]) continue;
            if (w->cur_ch[dst] != p->ch_idx) continue;

            /* Check whether ANY other active air packet on the SAME channel overlapped at receiver dst */
            bool collided = false;
            for (uint32_t j = 0; j < SIM_MAX_IN_FLIGHT; ++j) {
                if (j == i || !w->air[j].active) continue;
                sim_air_pkt_t *q = &w->air[j];
                if (q->ch_idx != p->ch_idx) continue;
                if (!w->can_hear[q->src_idx][dst]) continue;
                if (q->tx_start_phys_ns < p->tx_end_phys_ns &&
                    p->tx_start_phys_ns < q->tx_end_phys_ns) {
                    collided = true;
                    break;
                }
            }

            if (collided) {
                if (p->frame_type == ON_FRAME_JOIN_REQ ||
                    p->frame_type == ON_FRAME_RTT_REQ ||
                    p->frame_type == ON_FRAME_CRA_ALERT) {
                    w->cra_collisions++;
                } else {
                    w->tdma_subg_collisions++;
                }
                continue;
            }

            /* Hardware preamble correlator captures rx timestamp at tx_start + link_delay + hw_jitter */
            int64_t rx_phys_ns = p->tx_start_phys_ns +
                                 w->link_delay_ns[p->src_idx][dst] +
                                 sim_rand_jitter(w);
            int64_t rx_local_ns = sim_phys_to_local_ns(w, dst, rx_phys_ns);

            /* Per-node interference/shielding blackout injection */
            if (w->blackout_node_idx >= 0 &&
                dst == (uint32_t)w->blackout_node_idx &&
                p->frame_type == ON_FRAME_SYNC_MAP &&
                w->nodes[p->src_idx].current_sf_seq >= w->drop_sync_start_sf &&
                w->nodes[p->src_idx].current_sf_seq <= w->drop_sync_end_sf) {
                w->sync_drops_injected++;
                continue;
            }

            /* Respect low-power TDMA radio sleep gating */
            if (!on_node_is_rx_open(&w->nodes[dst], rx_local_ns)) {
                continue;
            }

            on_node_on_rx_raw(&w->nodes[dst], p->frame, p->len, rx_local_ns);
            w->total_rx_packets++;
        }

        p->active = false;
    }
}

void sim_world_run_us(sim_world_t *w, int64_t duration_us, int64_t step_us) {
    int64_t end_ns  = w->phys_now_ns + duration_us * 1000LL;
    int64_t step_ns = step_us * 1000LL;

    while (w->phys_now_ns <= end_ns) {
        /* Optional Primary G kill injection */
        if (!w->kill_executed && w->kill_node_idx >= 0 &&
            (uint32_t)w->kill_node_idx < w->num_nodes &&
            w->nodes[w->kill_node_idx].current_sf_seq >= w->kill_at_sf) {
            w->alive[w->kill_node_idx] = false;
            w->kill_executed = true;
            if (w->verbose) {
                printf("[t=%7.3f ms] !!! INJECTED POWER LOSS on %s at SF=%u !!!\n",
                       (double)w->phys_now_ns * 1e-6,
                       w->names[w->kill_node_idx], w->kill_at_sf);
            }
        }

        deliver_air_packets(w);

        for (uint32_t i = 0; i < w->num_nodes; ++i) {
            if (!w->alive[i]) continue;
            int64_t loc_ns = sim_phys_to_local_ns(w, i, w->phys_now_ns);
            on_node_step(&w->nodes[i], loc_ns);
        }

        w->phys_now_ns += step_ns;
    }
}

static const char *role_str(on_role_t r) {
    switch (r) {
        case ON_ROLE_G:     return "G-MASTER";
        case ON_ROLE_SUB_G: return "SUB-G";
        case ON_ROLE_T:     return "T-NODE";
        default:            return "UNJOINED";
    }
}

void sim_world_print_report(const sim_world_t *w) {
    printf("\n==================== OpenNear 10-Node Simulation Report ====================\n");
    printf("%-7s %-8s %-6s %-4s %-4s %-8s %-10s %-8s %-7s %-6s\n",
           "Node", "Role", "ShortID", "Hop", "Slot", "DriftPpm", "NcoPpm", "RTT(ns)", "Duty%", "ACTs");
    printf("----------------------------------------------------------------------------\n");
    for (uint32_t i = 0; i < w->num_nodes; ++i) {
        const on_node_t *n = &w->nodes[i];
        printf("%-7s %-8s 0x%03X  %-4s %-4u %+7.2f  %+9.2f  %-8lld %5.2f%%  %-6u%s\n",
               w->names[i],
               w->alive[i] ? role_str(n->role) : "DEAD",
               n->short_id,
               n->is_sub_node ? "2-H" : (n->role == ON_ROLE_G ? "0-H" : "1-H"),
               n->my_slot,
               w->drift_ppm[i],
               n->sync.rate_ppb * 1e-3,
               (long long)n->sync.link_delay_ns,
               on_node_duty_cycle_pct(n),
               n->act_fire_total,
               (i == 0u && !w->alive[i]) ? " [KILLED]" : "");
    }
    printf("----------------------------------------------------------------------------\n");
    printf("Total TX=%u | Total RX=%u | TDMA/Sub-G Collisions=%u | C-RA Collisions=%u\n",
           w->total_tx_packets, w->total_rx_packets,
           w->tdma_subg_collisions, w->cra_collisions);
    if (w->failover_new_g_id != 0u) {
        printf("Backup_ID Failover: New G = 0x%03X at SF=%u (zero-vote hot-standby)\n",
               w->failover_new_g_id, w->failover_sf_seq);
    }
    printf("============================================================================\n");
}

bool sim_world_export_csv(const sim_world_t *w, const char *csv_path) {
    FILE *fp = fopen(csv_path, "w");
    if (!fp) return false;
    fprintf(fp, "sf_seq,fired_nodes,max_skew_ns,cmd_consistent\n");
    for (uint32_t i = 0; i < w->act_hist_count; ++i) {
        const sim_act_record_t *r = &w->act_hist[i];
        uint32_t cnt = 0u;
        for (uint32_t b = 0; b < w->num_nodes; ++b) {
            if (r->fired_mask & (1u << b)) cnt++;
        }
        fprintf(fp, "%u,%u,%lld,%d\n",
                r->sf_seq, cnt, (long long)r->max_skew_ns, r->cmd_consistent ? 1 : 0);
    }
    fclose(fp);
    return true;
}

bool sim_world_export_dashboard_js(const sim_world_t *w, const char *js_path) {
    FILE *fp = fopen(js_path, "w");
    if (!fp) return false;
    fprintf(fp, "window.OPENNEAR_DATA = {\n");
    fprintf(fp, "  \"meta\": {\"total_tx\": %u, \"total_rx\": %u, \"tdma_collisions\": %u, \"cra_collisions\": %u, \"failover_sf\": %u, \"failover_new_g\": \"0x%03X\", \"pcap_pkts\": %u},\n",
            w->total_tx_packets, w->total_rx_packets, w->tdma_subg_collisions, w->cra_collisions,
            w->failover_sf_seq, w->failover_new_g_id, w->pcap_pkts);
    fprintf(fp, "  \"nodes\": [\n");
    for (uint32_t i = 0; i < w->num_nodes; ++i) {
        const on_node_t *n = &w->nodes[i];
        fprintf(fp, "    {\"name\":\"%s\",\"alive\":%s,\"role\":\"%s\",\"short_id\":\"0x%03X\",\"hop\":%d,\"slot\":%u,\"drift_ppm\":%.2f,\"nco_ppm\":%.2f,\"rtt_ns\":%lld,\"duty_pct\":%.2f,\"acts\":%u}%s\n",
                w->names[i], w->alive[i] ? "true" : "false",
                w->alive[i] ? role_str(n->role) : "DEAD",
                n->short_id,
                n->is_sub_node ? 2 : (n->role == ON_ROLE_G ? 0 : 1),
                n->my_slot, w->drift_ppm[i], n->sync.rate_ppb * 1e-3,
                (long long)n->sync.link_delay_ns, on_node_duty_cycle_pct(n), n->act_fire_total,
                (i + 1u < w->num_nodes) ? "," : "");
    }
    fprintf(fp, "  ],\n  \"timeline\": [\n");
    for (uint32_t i = 0; i < w->act_hist_count; ++i) {
        const sim_act_record_t *r = &w->act_hist[i];
        uint32_t cnt = 0u;
        for (uint32_t b = 0; b < w->num_nodes; ++b) {
            if (r->fired_mask & (1u << b)) cnt++;
        }
        uint8_t ch = (r->sf_seq < SIM_MAX_SF_HISTORY) ? w->sf_ch_hist[r->sf_seq] : 0u;
        uint16_t cmap = (r->sf_seq < SIM_MAX_SF_HISTORY) ? w->sf_map_hist[r->sf_seq] : 0xFFFFu;
        fprintf(fp, "    {\"sf\":%u,\"nodes\":%u,\"skew_ns\":%lld,\"ch\":%u,\"map\":%u}%s\n",
                r->sf_seq, cnt, (long long)r->max_skew_ns, ch, cmap,
                (i + 1u < w->act_hist_count) ? "," : "");
    }
    fprintf(fp, "  ]\n};\n");
    fclose(fp);
    return true;
}
