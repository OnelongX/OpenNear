#include "on_locate.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static bool loc_mac_nonzero(const uint8_t mac[6]) {
    return mac != NULL && ((mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5]) != 0u);
}

static int8_t median3_i8(int8_t a, int8_t b, int8_t c) {
    if (a > b) { int8_t t = a; a = b; b = t; }
    if (b > c) { int8_t t = b; b = c; c = t; }
    if (a > b) { int8_t t = a; a = b; b = t; }
    return b;
}

static float rssi_to_dist_cm(float rssi_dbm, int8_t tx_1m_dbm, float n_exp) {
    if (n_exp < 1.2f) n_exp = 1.2f;
    if (n_exp > 5.0f) n_exp = 5.0f;
    float exp_arg = ((float)tx_1m_dbm - rssi_dbm) / (10.0f * n_exp);
    float d_m = powf(10.0f, exp_arg);
    if (d_m < 0.2f) d_m = 0.2f;
    if (d_m > 30.0f) d_m = 30.0f;
    return d_m * 100.0f;
}

static void copy_sanitized(char *dst, size_t cap, const char *src) {
    if (!dst || cap == 0u) return;
    size_t j = 0u;
    if (src) {
        for (size_t i = 0u; src[i] != '\0' && j + 1u < cap; ++i) {
            unsigned char c = (unsigned char)src[i];
            if (c >= 0x20u && c != '"' && c != '\\') {
                dst[j++] = (char)c;
            }
        }
    }
    dst[j] = '\0';
}

static void copy_room(char dst[ON_LOC_ROOM_MAX_LEN], const char *src) {
    copy_sanitized(dst, ON_LOC_ROOM_MAX_LEN, src);
}

static uint8_t fnv1a_8(const uint8_t *data, size_t len) {
    uint32_t h = 2166136261u;
    for (size_t i = 0u; i < len; ++i) {
        h ^= (uint32_t)data[i];
        h *= 16777619u;
    }
    return (uint8_t)((h ^ (h >> 8) ^ (h >> 16) ^ (h >> 24)) & 0xFFu);
}

static int8_t clamp_tx1m(int32_t v) {
    if (v < -100) return -100;
    if (v > -20)  return -20;
    return (int8_t)v;
}

bool on_ble_beacon_parse(const uint8_t *ad_data, size_t ad_len, on_ble_beacon_t *out) {
    if (!ad_data || ad_len < 4u || !out) return false;
    memset(out, 0, sizeof(*out));

    size_t pos = 0u;
    while (pos < ad_len) {
        uint8_t flen = ad_data[pos];
        if (flen == 0u) break;
        if (pos + 1u + (size_t)flen > ad_len) break;
        uint8_t ftype = ad_data[pos + 1u];
        const uint8_t *val = ad_data + pos + 2u;
        size_t vlen = (size_t)flen - 1u;

        /* 1. Apple iBeacon: AD Type 0xFF (Manufacturer Specific), Company=0x004C, 0x02, 0x15 */
        if (ftype == 0xFFu && vlen >= 25u &&
            val[0] == 0x4Cu && val[1] == 0x00u &&
            val[2] == 0x02u && val[3] == 0x15u) {
            out->valid           = true;
            out->source          = ON_LOC_SRC_IBEACON;
            memcpy(out->uuid16, val + 4u, 16u);
            out->major           = (uint16_t)(((uint16_t)val[20] << 8) | (uint16_t)val[21]);
            out->minor           = (uint16_t)(((uint16_t)val[22] << 8) | (uint16_t)val[23]);
            out->tx_power_1m_dbm = clamp_tx1m((int8_t)val[24]);
            snprintf(out->label, sizeof(out->label), "iB:%u/%u",
                     (unsigned)out->major, (unsigned)out->minor);
            return true;
        }

        /* 2. Google Eddystone: AD Type 0x16 (16-bit Service Data), UUID=0xFEAA (LE: AA FE) */
        if (ftype == 0x16u && vlen >= 4u &&
            val[0] == 0xAAu && val[1] == 0xFEu) {
            uint8_t frame_type = val[2];
            if (frame_type == 0x00u && vlen >= 20u) {
                /* Eddystone-UID: [2]=0x00, [3]=tx_0m, [4..13]=namespace(10), [14..19]=instance(6) */
                out->valid           = true;
                out->source          = ON_LOC_SRC_EDDYSTONE;
                out->eddy_frame_type = 0x00u;
                out->tx_power_1m_dbm = clamp_tx1m((int32_t)(int8_t)val[3] - 41);
                memcpy(out->eddy_ns, val + 4u, 10u);
                memcpy(out->eddy_inst, val + 14u, 6u);
                snprintf(out->label, sizeof(out->label), "Ed:%02X%02X%02X%02X",
                         (unsigned)out->eddy_inst[2], (unsigned)out->eddy_inst[3],
                         (unsigned)out->eddy_inst[4], (unsigned)out->eddy_inst[5]);
                return true;
            }
            if (frame_type == 0x10u && vlen >= 5u) {
                /* Eddystone-URL: [2]=0x10, [3]=tx_0m, [4]=scheme, [5..]=encoded URL */
                out->valid           = true;
                out->source          = ON_LOC_SRC_EDDYSTONE;
                out->eddy_frame_type = 0x10u;
                out->tx_power_1m_dbm = clamp_tx1m((int32_t)(int8_t)val[3] - 41);
                snprintf(out->label, sizeof(out->label), "EdURL:%02X", (unsigned)val[4]);
                return true;
            }
            if (frame_type == 0x20u && vlen >= 16u && val[3] == 0x00u) {
                /* Eddystone-TLM (unencrypted v0): [4..5]=vbatt_mv, [6..7]=temp 8.8 signed */
                out->valid           = true;
                out->source          = ON_LOC_SRC_BLE_RAW;
                out->eddy_frame_type = 0x20u;
                out->tx_power_1m_dbm = 0; /* TLM carries no txPower -> use anchor/previous */
                out->tlm_batt_mv     = (uint16_t)(((uint16_t)val[4] << 8) | (uint16_t)val[5]);
                int8_t  t_int        = (int8_t)val[6];
                uint8_t t_frac       = val[7];
                out->tlm_temp_c_x100 = (int16_t)((int32_t)t_int * 100 + ((int32_t)t_frac * 100) / 256);
                snprintf(out->label, sizeof(out->label), "TLM:%umV", (unsigned)out->tlm_batt_mv);
                return true;
            }
        }

        pos += 1u + (size_t)flen;
    }
    return false;
}

void on_ble_beacon_derive_mac(const on_ble_beacon_t *b, const uint8_t fallback_ble_mac[6],
                              uint8_t out_mac[6]) {
    if (!out_mac) return;
    memset(out_mac, 0, 6u);
    if (!b || !b->valid) {
        if (fallback_ble_mac) memcpy(out_mac, fallback_ble_mac, 6u);
        return;
    }
    if (b->source == ON_LOC_SRC_IBEACON) {
        /* Deterministic locally-administered MAC48 immune to iOS/Android MAC rotation */
        out_mac[0] = 0x02u; /* LAA unicast marker for iBeacon */
        out_mac[1] = fnv1a_8(b->uuid16, 16u);
        out_mac[2] = (uint8_t)((b->major >> 8) & 0xFFu);
        out_mac[3] = (uint8_t)(b->major & 0xFFu);
        out_mac[4] = (uint8_t)((b->minor >> 8) & 0xFFu);
        out_mac[5] = (uint8_t)(b->minor & 0xFFu);
        return;
    }
    if (b->source == ON_LOC_SRC_EDDYSTONE && b->eddy_frame_type == 0x00u) {
        out_mac[0] = 0x06u; /* LAA unicast marker for Eddystone-UID */
        out_mac[1] = (uint8_t)(fnv1a_8(b->eddy_ns, 10u) ^ b->eddy_inst[0] ^ b->eddy_inst[1]);
        out_mac[2] = b->eddy_inst[2];
        out_mac[3] = b->eddy_inst[3];
        out_mac[4] = b->eddy_inst[4];
        out_mac[5] = b->eddy_inst[5];
        return;
    }
    if (fallback_ble_mac) {
        memcpy(out_mac, fallback_ble_mac, 6u);
    }
}

void on_loc_init(on_loc_ctx_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    for (uint32_t i = 0u; i < ON_LOC_MAX_TARGETS; ++i) {
        ctx->targets[i].cur_room_anchor_idx = -1;
    }
}

static int find_anchor_idx(const on_loc_ctx_t *ctx, const uint8_t mac48[6]) {
    if (!ctx || !loc_mac_nonzero(mac48)) return -1;
    for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
        if (ctx->anchors[i].active && memcmp(ctx->anchors[i].mac48, mac48, 6u) == 0) {
            return (int)i;
        }
    }
    return -1;
}

on_err_t on_loc_set_anchor(on_loc_ctx_t *ctx, const uint8_t mac48[6], uint16_t short_id,
                           const char *room, int32_t x_cm, int32_t y_cm) {
    if (!ctx || !loc_mac_nonzero(mac48)) return ON_ERR_INVALID_ARG;
    int idx = find_anchor_idx(ctx, mac48);
    bool removing = (!room || room[0] == '\0');
    if (removing) {
        if (idx >= 0) {
            memset(&ctx->anchors[idx], 0, sizeof(ctx->anchors[idx]));
        }
        return ON_OK;
    }
    if (idx < 0) {
        for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
            if (!ctx->anchors[i].active) {
                idx = (int)i;
                break;
            }
        }
    }
    if (idx < 0) return ON_ERR_NO_MEM;

    on_loc_anchor_t *a = &ctx->anchors[idx];
    bool was_active = a->active;
    a->active   = true;
    memcpy(a->mac48, mac48, 6u);
    if (short_id != ON_SHORT_ID_UNASSIGNED) a->short_id = short_id & ON_SHORT_ID_MASK;
    copy_room(a->room, room);
    a->x_cm = x_cm;
    a->y_cm = y_cm;
    if (!was_active) {
        a->tx_power_1m_dbm = -45;
        a->path_loss_n     = 2.5f;
    }
    return ON_OK;
}

on_err_t on_loc_calibrate_anchor(on_loc_ctx_t *ctx, const uint8_t mac48[6],
                                 int8_t tx_power_1m_dbm, float path_loss_n) {
    if (!ctx || !loc_mac_nonzero(mac48)) return ON_ERR_INVALID_ARG;
    int idx = find_anchor_idx(ctx, mac48);
    if (idx < 0) return ON_ERR_INVALID_ARG;
    ctx->anchors[idx].tx_power_1m_dbm = tx_power_1m_dbm;
    ctx->anchors[idx].path_loss_n     = path_loss_n;
    return ON_OK;
}

static on_loc_target_t *find_or_alloc_target(on_loc_ctx_t *ctx, const uint8_t mac48[6], uint32_t now_ms) {
    on_loc_target_t *empty  = NULL;
    on_loc_target_t *oldest = &ctx->targets[0];
    for (uint32_t i = 0u; i < ON_LOC_MAX_TARGETS; ++i) {
        on_loc_target_t *t = &ctx->targets[i];
        if (t->active && memcmp(t->mac48, mac48, 6u) == 0) {
            return t;
        }
        if (!t->active && !empty) empty = t;
        if (t->active && (!oldest->active || (now_ms - t->last_seen_ms) > (now_ms - oldest->last_seen_ms))) {
            oldest = t;
        }
    }
    on_loc_target_t *slot = empty ? empty : oldest;
    memset(slot, 0, sizeof(*slot));
    slot->active              = true;
    slot->cur_room_anchor_idx = -1;
    memcpy(slot->mac48, mac48, 6u);
    return slot;
}

static on_err_t observe_common(on_loc_ctx_t *ctx, const uint8_t anchor_mac48[6],
                               const uint8_t target_mac48[6], uint16_t target_short_id,
                               on_loc_source_t source, int8_t tx_power_1m_dbm,
                               const char *label, int8_t rssi_dbm, uint32_t now_ms) {
    if (!ctx || !loc_mac_nonzero(anchor_mac48) || !loc_mac_nonzero(target_mac48)) {
        return ON_ERR_INVALID_ARG;
    }
    if (rssi_dbm >= 0 || rssi_dbm < -120) return ON_ERR_INVALID_ARG;
    int aidx = find_anchor_idx(ctx, anchor_mac48);
    if (aidx < 0) return ON_ERR_NOT_READY;

    on_loc_target_t *t = find_or_alloc_target(ctx, target_mac48, now_ms);
    if (!t) return ON_ERR_NO_MEM;
    if (target_short_id != ON_SHORT_ID_UNASSIGNED) {
        t->short_id = target_short_id & ON_SHORT_ID_MASK;
    }
    t->source = (uint8_t)source;
    if (tx_power_1m_dbm < 0 && tx_power_1m_dbm >= -100) {
        t->tx_power_1m_dbm = tx_power_1m_dbm;
    }
    if (label && label[0] != '\0') {
        copy_sanitized(t->label, sizeof(t->label), label);
    }
    t->last_seen_ms = now_ms;

    on_loc_obs_t *slot = NULL;
    on_loc_obs_t *stale = &t->obs[0];
    for (uint32_t i = 0u; i < ON_LOC_MAX_OBS_PER_TARGET; ++i) {
        if (t->obs[i].used && t->obs[i].anchor_idx == (uint8_t)aidx) {
            slot = &t->obs[i];
            break;
        }
        if (!t->obs[i].used && !slot) {
            slot = &t->obs[i];
        }
        if (t->obs[i].used && (now_ms - t->obs[i].last_ms) > (now_ms - stale->last_ms)) {
            stale = &t->obs[i];
        }
    }
    if (!slot) slot = stale;

    if (!slot->used || slot->anchor_idx != (uint8_t)aidx) {
        memset(slot, 0, sizeof(*slot));
        slot->used        = true;
        slot->anchor_idx  = (uint8_t)aidx;
        slot->raw_hist[0] = rssi_dbm;
        slot->hist_cnt    = 1u;
        slot->hist_pos    = 1u;
        slot->smooth_rssi = (float)rssi_dbm;
        slot->last_ms     = now_ms;
        return ON_OK;
    }

    slot->raw_hist[slot->hist_pos] = rssi_dbm;
    slot->hist_pos = (uint8_t)((slot->hist_pos + 1u) % 3u);
    if (slot->hist_cnt < 3u) slot->hist_cnt++;

    int8_t filt = rssi_dbm;
    if (slot->hist_cnt >= 3u) {
        filt = median3_i8(slot->raw_hist[0], slot->raw_hist[1], slot->raw_hist[2]);
    }
    slot->smooth_rssi = 0.65f * slot->smooth_rssi + 0.35f * (float)filt;
    slot->last_ms     = now_ms;
    return ON_OK;
}

on_err_t on_loc_observe_rssi(on_loc_ctx_t *ctx, const uint8_t anchor_mac48[6],
                             const uint8_t target_mac48[6], uint16_t target_short_id,
                             int8_t rssi_dbm, uint32_t now_ms) {
    return observe_common(ctx, anchor_mac48, target_mac48, target_short_id,
                          ON_LOC_SRC_OPENNEAR, 0, NULL, rssi_dbm, now_ms);
}

on_err_t on_loc_observe_beacon(on_loc_ctx_t *ctx, const uint8_t anchor_mac48[6],
                               const uint8_t target_mac48[6], on_loc_source_t source,
                               int8_t tx_power_1m_dbm, int8_t rssi_dbm,
                               const char *label, uint32_t now_ms) {
    return observe_common(ctx, anchor_mac48, target_mac48, ON_SHORT_ID_UNASSIGNED,
                          source, tx_power_1m_dbm, label, rssi_dbm, now_ms);
}

bool on_loc_solve(on_loc_ctx_t *ctx, const uint8_t target_mac48[6],
                  uint32_t now_ms, on_loc_estimate_t *out_est) {
    if (!ctx || !out_est || !loc_mac_nonzero(target_mac48)) return false;
    memset(out_est, 0, sizeof(*out_est));

    on_loc_target_t *t = NULL;
    for (uint32_t i = 0u; i < ON_LOC_MAX_TARGETS; ++i) {
        if (ctx->targets[i].active && memcmp(ctx->targets[i].mac48, target_mac48, 6u) == 0) {
            t = &ctx->targets[i];
            break;
        }
    }
    if (!t) return false;

    /* Collect fresh anchor observations */
    float ax[ON_LOC_MAX_OBS_PER_TARGET];
    float ay[ON_LOC_MAX_OBS_PER_TARGET];
    float dist_cm[ON_LOC_MAX_OBS_PER_TARGET];
    float weight[ON_LOC_MAX_OBS_PER_TARGET];
    int   aidx_list[ON_LOC_MAX_OBS_PER_TARGET];
    uint8_t cnt = 0u;

    int best_i = -1;
    float best_rssi = -127.0f;
    float cur_room_rssi = -127.0f;
    bool  cur_room_alive = false;

    for (uint32_t i = 0u; i < ON_LOC_MAX_OBS_PER_TARGET; ++i) {
        const on_loc_obs_t *o = &t->obs[i];
        if (!o->used) continue;
        if (now_ms >= o->last_ms && (now_ms - o->last_ms) > ON_LOC_OBS_STALE_MS) continue;
        if (o->anchor_idx >= ON_LOC_MAX_ANCHORS || !ctx->anchors[o->anchor_idx].active) continue;

        const on_loc_anchor_t *a = &ctx->anchors[o->anchor_idx];
        int8_t tx_ref = (t->tx_power_1m_dbm < 0) ? t->tx_power_1m_dbm : a->tx_power_1m_dbm;
        float d = rssi_to_dist_cm(o->smooth_rssi, tx_ref, a->path_loss_n);
        ax[cnt]        = (float)a->x_cm;
        ay[cnt]        = (float)a->y_cm;
        dist_cm[cnt]   = d;
        weight[cnt]    = 1.0f / ((d * d) + 2500.0f); /* inverse-distance^2 weight (50cm floor) */
        aidx_list[cnt] = (int)o->anchor_idx;
        if (o->smooth_rssi > best_rssi) {
            best_rssi = o->smooth_rssi;
            best_i    = (int)cnt;
        }
        if ((int)o->anchor_idx == t->cur_room_anchor_idx) {
            cur_room_rssi  = o->smooth_rssi;
            cur_room_alive = true;
        }
        cnt++;
    }
    if (cnt == 0u || best_i < 0) {
        if (now_ms >= t->last_seen_ms && (now_ms - t->last_seen_ms) > (ON_LOC_OBS_STALE_MS * 2u)) {
            t->active = false;
        }
        return false;
    }

    /* 1. Room classification with +4 dB hysteresis anti-ping-pong */
    int chosen_aidx = aidx_list[best_i];
    if (cur_room_alive && t->cur_room_anchor_idx >= 0 &&
        t->cur_room_anchor_idx < (int)ON_LOC_MAX_ANCHORS &&
        ctx->anchors[t->cur_room_anchor_idx].active) {
        if (best_rssi < cur_room_rssi + (float)ON_LOC_HYSTERESIS_DB) {
            chosen_aidx = t->cur_room_anchor_idx;
        }
    }
    t->cur_room_anchor_idx = (int8_t)chosen_aidx;
    copy_room(t->cur_room, ctx->anchors[chosen_aidx].room);

    /* 2. Initial weighted-centroid estimate */
    float wsum = 0.0f, x = 0.0f, y = 0.0f;
    for (uint8_t i = 0u; i < cnt; ++i) {
        wsum += weight[i];
        x += weight[i] * ax[i];
        y += weight[i] * ay[i];
    }
    if (wsum > 0.0f) {
        x /= wsum;
        y /= wsum;
    }

    /* 3. For >= 3 anchors, run 6 iterations of damped Gauss-Newton trilateration refinement */
    if (cnt >= 3u) {
        for (int iter = 0; iter < 6; ++iter) {
            float h00 = 1e-4f, h01 = 0.0f, h11 = 1e-4f;
            float g0 = 0.0f, g1 = 0.0f;
            for (uint8_t i = 0u; i < cnt; ++i) {
                float dx = x - ax[i];
                float dy = y - ay[i];
                float r  = sqrtf(dx * dx + dy * dy);
                if (r < 1.0f) r = 1.0f;
                float jx = dx / r;
                float jy = dy / r;
                float err = r - dist_cm[i];
                float wi = 1.0f / (dist_cm[i] + 50.0f);
                h00 += wi * jx * jx;
                h01 += wi * jx * jy;
                h11 += wi * jy * jy;
                g0  += wi * jx * err;
                g1  += wi * jy * err;
            }
            float det = h00 * h11 - h01 * h01;
            if (fabsf(det) < 1e-9f) break;
            float step_x = -( h11 * g0 - h01 * g1) / det;
            float step_y = -(-h01 * g0 + h00 * g1) / det;
            /* Clamp single step to 2 m for numerical stability */
            float step_mag = sqrtf(step_x * step_x + step_y * step_y);
            if (step_mag > 200.0f) {
                step_x *= (200.0f / step_mag);
                step_y *= (200.0f / step_mag);
            }
            x += 0.7f * step_x;
            y += 0.7f * step_y;
        }
    }

    /* 4. 2D kinematic EMA smoothing */
    if (!t->pos_inited) {
        t->smooth_x_cm = x;
        t->smooth_y_cm = y;
        t->pos_inited  = true;
    } else {
        t->smooth_x_cm = 0.60f * t->smooth_x_cm + 0.40f * x;
        t->smooth_y_cm = 0.60f * t->smooth_y_cm + 0.40f * y;
    }

    /* Residual RMS distance error (cm) */
    float rms = 0.0f;
    for (uint8_t i = 0u; i < cnt; ++i) {
        float dx = t->smooth_x_cm - ax[i];
        float dy = t->smooth_y_cm - ay[i];
        float e  = sqrtf(dx * dx + dy * dy) - dist_cm[i];
        rms += e * e;
    }
    rms = sqrtf(rms / (float)cnt);
    if (cnt == 1u) rms = dist_cm[0];
    if (rms > 3000.0f) rms = 3000.0f;

    out_est->valid           = true;
    memcpy(out_est->mac48, t->mac48, 6u);
    out_est->short_id        = t->short_id;
    out_est->source          = t->source;
    out_est->tx_power_1m_dbm = t->tx_power_1m_dbm;
    copy_sanitized(out_est->label, sizeof(out_est->label), t->label);
    copy_room(out_est->room, t->cur_room);
    out_est->x_cm            = (int32_t)lroundf(t->smooth_x_cm);
    out_est->y_cm            = (int32_t)lroundf(t->smooth_y_cm);
    out_est->best_rssi_dbm   = (int8_t)lroundf(best_rssi);
    out_est->anchor_count    = cnt;
    out_est->est_err_cm      = (uint16_t)lroundf(rms);
    return true;
}

static const char *loc_src_name(uint8_t s) {
    switch (s) {
        case ON_LOC_SRC_IBEACON:   return "ibeacon";
        case ON_LOC_SRC_EDDYSTONE: return "eddystone";
        case ON_LOC_SRC_BLE_RAW:   return "ble";
        default:                   return "opennear";
    }
}

static void loc_fmt_mac(const uint8_t m[6], char out[18]) {
    snprintf(out, 18u, "%02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned)m[0], (unsigned)m[1], (unsigned)m[2],
             (unsigned)m[3], (unsigned)m[4], (unsigned)m[5]);
}

/* ========================================================================== */
/* v7.2 IEEE 802.11bf Wi-Fi CSI Subcarrier Analyzer & Passive Presence Radar  */
/* ========================================================================== */

uint8_t on_csi_extract_iq_score(const int8_t *iq_buf, uint16_t iq_len, bool skip_first_word,
                                float *inout_baseline, float *inout_var_ema,
                                uint8_t out_bins16[ON_LOC_CSI_BINS]) {
    if (out_bins16) memset(out_bins16, 0, ON_LOC_CSI_BINS);
    if (!iq_buf || iq_len < 12u) return 0u;

    uint16_t off = (skip_first_word && iq_len > 16u) ? 4u : 0u;
    uint16_t pairs = (uint16_t)((iq_len - off) / 2u);
    if (pairs < 4u) return 0u;

    float bin_sum[ON_LOC_CSI_BINS];
    uint16_t bin_cnt[ON_LOC_CSI_BINS];
    memset(bin_sum, 0, sizeof(bin_sum));
    memset(bin_cnt, 0, sizeof(bin_cnt));

    float total_amp = 0.0f;
    float total_sq  = 0.0f;
    for (uint16_t k = 0u; k < pairs; ++k) {
        float im = (float)iq_buf[off + k * 2u];
        float re = (float)iq_buf[off + k * 2u + 1u];
        float amp = sqrtf(re * re + im * im);
        total_amp += amp;
        total_sq  += amp * amp;
        uint16_t b = (uint16_t)((k * ON_LOC_CSI_BINS) / pairs);
        if (b >= ON_LOC_CSI_BINS) b = ON_LOC_CSI_BINS - 1u;
        bin_sum[b] += amp;
        bin_cnt[b]++;
    }

    float mean_amp = total_amp / (float)pairs;
    float spatial_var = (total_sq / (float)pairs) - (mean_amp * mean_amp);
    if (spatial_var < 0.0f) spatial_var = 0.0f;
    float spatial_std = sqrtf(spatial_var);

    if (out_bins16) {
        for (uint16_t b = 0u; b < ON_LOC_CSI_BINS; ++b) {
            float avg = (bin_cnt[b] > 0u) ? (bin_sum[b] / (float)bin_cnt[b]) : 0.0f;
            int32_t scaled = (int32_t)lroundf(avg * 2.2f);
            if (scaled < 0) scaled = 0;
            if (scaled > 100) scaled = 100;
            out_bins16[b] = (uint8_t)scaled;
        }
    }

    float temporal_delta = 0.0f;
    if (inout_baseline) {
        if (*inout_baseline <= 0.01f) {
            *inout_baseline = mean_amp;
        } else {
            temporal_delta = fabsf(mean_amp - *inout_baseline);
            *inout_baseline = 0.92f * (*inout_baseline) + 0.08f * mean_amp;
        }
    }

    /* Combine temporal amplitude drift + frequency-selective subcarrier std dispersion */
    float raw_metric = temporal_delta * 7.5f + spatial_std * 2.6f;
    if (inout_var_ema) {
        if (*inout_var_ema <= 0.01f) {
            *inout_var_ema = raw_metric;
        } else {
            float alpha = (raw_metric > *inout_var_ema) ? 0.45f : 0.18f;
            *inout_var_ema = (1.0f - alpha) * (*inout_var_ema) + alpha * raw_metric;
        }
        raw_metric = *inout_var_ema;
    }

    int32_t score = (int32_t)lroundf(raw_metric);
    if (score < 0) score = 0;
    if (score > 100) score = 100;
    return (uint8_t)score;
}

on_err_t on_loc_observe_csi(on_loc_ctx_t *ctx, const uint8_t anchor_mac48[6],
                            uint16_t anchor_short_id, uint8_t motion_score,
                            const uint8_t bins16[ON_LOC_CSI_BINS], uint32_t now_ms) {
    if (!ctx) return ON_ERR_INVALID_ARG;
    int aidx = -1;
    if (loc_mac_nonzero(anchor_mac48)) {
        aidx = find_anchor_idx(ctx, anchor_mac48);
    }
    if (aidx < 0 && anchor_short_id != ON_SHORT_ID_UNASSIGNED) {
        for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
            if (ctx->anchors[i].active && ctx->anchors[i].short_id == anchor_short_id) {
                aidx = (int)i;
                break;
            }
        }
    }
    if (aidx < 0) return ON_ERR_NOT_READY;

    on_loc_anchor_t *a = &ctx->anchors[aidx];
    if (anchor_short_id != ON_SHORT_ID_UNASSIGNED) {
        a->short_id = anchor_short_id & ON_SHORT_ID_MASK;
    }
    if (motion_score > 100u) motion_score = 100u;

    /* Fast attack, smooth decay EMA for room motion score */
    if (a->csi_last_ms == 0u) {
        a->csi_motion_score = motion_score;
    } else {
        uint16_t prev = a->csi_motion_score;
        if (motion_score >= prev) {
            a->csi_motion_score = (uint8_t)((prev * 2u + (uint16_t)motion_score * 3u + 2u) / 5u);
        } else {
            a->csi_motion_score = (uint8_t)((prev * 3u + (uint16_t)motion_score * 2u + 2u) / 5u);
        }
    }

    if (a->csi_motion_score >= 45u) {
        a->csi_presence = (uint8_t)ON_CSI_STATE_MOVING;
    } else if (a->csi_motion_score >= 15u) {
        a->csi_presence = (uint8_t)ON_CSI_STATE_STATIC;
    } else {
        a->csi_presence = (uint8_t)ON_CSI_STATE_EMPTY;
    }

    if (bins16) {
        for (uint32_t b = 0u; b < ON_LOC_CSI_BINS; ++b) {
            a->csi_bins[b] = (bins16[b] <= 100u) ? bins16[b] : 100u;
        }
    } else {
        /* Synthesize smooth 16-subcarrier envelope from motion_score when not supplied */
        for (uint32_t b = 0u; b < ON_LOC_CSI_BINS; ++b) {
            int32_t wave = (int32_t)a->csi_motion_score + (int32_t)((b & 3u) * 4u) - 6;
            if (wave < 2) wave = 2;
            if (wave > 100) wave = 100;
            a->csi_bins[b] = (uint8_t)wave;
        }
    }
    a->csi_last_ms = now_ms;
    return ON_OK;
}

/* ========================================================================== */
/* v7.2 Offline 2D Circular Geofence & Spatial / CSI Automation Rule Engine   */
/* ========================================================================== */

on_err_t on_loc_set_geofence(on_loc_ctx_t *ctx, const char *name,
                             int32_t x_cm, int32_t y_cm, uint16_t radius_cm) {
    if (!ctx || !name || name[0] == '\0') return ON_ERR_INVALID_ARG;
    char clean_name[ON_LOC_ROOM_MAX_LEN];
    copy_room(clean_name, name);
    if (clean_name[0] == '\0') return ON_ERR_INVALID_ARG;

    int found = -1;
    int empty = -1;
    for (uint32_t i = 0u; i < ON_LOC_MAX_GEOFENCES; ++i) {
        if (ctx->fences[i].active && strcmp(ctx->fences[i].name, clean_name) == 0) {
            found = (int)i;
            break;
        }
        if (!ctx->fences[i].active && empty < 0) empty = (int)i;
    }

    if (radius_cm == 0u) {
        if (found >= 0) memset(&ctx->fences[found], 0, sizeof(ctx->fences[found]));
        return ON_OK;
    }

    int slot = (found >= 0) ? found : empty;
    if (slot < 0) return ON_ERR_NO_MEM;
    on_loc_geofence_t *f = &ctx->fences[slot];
    f->active    = true;
    copy_room(f->name, clean_name);
    f->x_cm      = x_cm;
    f->y_cm      = y_cm;
    f->radius_cm = radius_cm;
    return ON_OK;
}

bool on_loc_check_geofence(const on_loc_ctx_t *ctx, int32_t x_cm, int32_t y_cm,
                           const char *fence_name) {
    if (!ctx || !fence_name || fence_name[0] == '\0') return false;
    for (uint32_t i = 0u; i < ON_LOC_MAX_GEOFENCES; ++i) {
        const on_loc_geofence_t *f = &ctx->fences[i];
        if (!f->active || strcmp(f->name, fence_name) != 0) continue;
        int64_t dx = (int64_t)x_cm - (int64_t)f->x_cm;
        int64_t dy = (int64_t)y_cm - (int64_t)f->y_cm;
        int64_t r  = (int64_t)f->radius_cm;
        return (dx * dx + dy * dy) <= (r * r);
    }
    return false;
}

on_err_t on_loc_set_rule(on_loc_ctx_t *ctx, uint8_t rule_id, uint8_t trig_type,
                         const char *zone, uint8_t csi_threshold,
                         const char *cmd_tag, const char *cmd_payload) {
    if (!ctx || rule_id == 0u || rule_id > ON_LOC_MAX_RULES) return ON_ERR_INVALID_ARG;
    on_loc_rule_t *r = &ctx->rules[rule_id - 1u];
    if (trig_type == 0u) {
        memset(r, 0, sizeof(*r));
        return ON_OK;
    }
    if (trig_type > (uint8_t)ON_RULE_TRIG_CSI_QUIET || !zone || zone[0] == '\0' ||
        !cmd_payload || cmd_payload[0] == '\0') {
        return ON_ERR_INVALID_ARG;
    }
    memset(r, 0, sizeof(*r));
    r->active        = true;
    r->id            = rule_id;
    r->trig_type     = trig_type;
    copy_room(r->zone, zone);
    r->csi_threshold = (csi_threshold > 0u && csi_threshold <= 100u) ? csi_threshold : 40u;
    copy_sanitized(r->cmd_tag, sizeof(r->cmd_tag), (cmd_tag && cmd_tag[0]) ? cmd_tag : "*");
    copy_sanitized(r->cmd_payload, sizeof(r->cmd_payload), cmd_payload);
    return ON_OK;
}

uint32_t on_loc_evaluate_rules(on_loc_ctx_t *ctx, uint32_t now_ms,
                               void (*on_fire)(void *user_ctx, const on_loc_rule_t *rule),
                               void *user_ctx) {
    if (!ctx) return 0u;

    /* Solve all active targets once */
    on_loc_estimate_t ests[ON_LOC_MAX_TARGETS];
    uint32_t est_cnt = 0u;
    for (uint32_t i = 0u; i < ON_LOC_MAX_TARGETS; ++i) {
        if (!ctx->targets[i].active) continue;
        if (on_loc_solve(ctx, ctx->targets[i].mac48, now_ms, &ests[est_cnt])) {
            est_cnt++;
        }
    }

    /* Update geofence occupancy flags */
    for (uint32_t fi = 0u; fi < ON_LOC_MAX_GEOFENCES; ++fi) {
        on_loc_geofence_t *f = &ctx->fences[fi];
        if (!f->active) continue;
        bool inside = false;
        for (uint32_t ti = 0u; ti < est_cnt; ++ti) {
            if (on_loc_check_geofence(ctx, ests[ti].x_cm, ests[ti].y_cm, f->name)) {
                inside = true;
                break;
            }
        }
        f->occupied = inside;
    }

    uint32_t fired = 0u;
    for (uint32_t ri = 0u; ri < ON_LOC_MAX_RULES; ++ri) {
        on_loc_rule_t *r = &ctx->rules[ri];
        if (!r->active) continue;

        bool cond = false;
        switch ((on_loc_rule_trig_t)r->trig_type) {
            case ON_RULE_TRIG_ENTER_ROOM:
                for (uint32_t ti = 0u; ti < est_cnt; ++ti) {
                    if (strcmp(ests[ti].room, r->zone) == 0) { cond = true; break; }
                }
                break;
            case ON_RULE_TRIG_LEAVE_ROOM:
                if (est_cnt > 0u) {
                    cond = true;
                    for (uint32_t ti = 0u; ti < est_cnt; ++ti) {
                        if (strcmp(ests[ti].room, r->zone) == 0) { cond = false; break; }
                    }
                }
                break;
            case ON_RULE_TRIG_IN_FENCE:
                for (uint32_t ti = 0u; ti < est_cnt; ++ti) {
                    if (on_loc_check_geofence(ctx, ests[ti].x_cm, ests[ti].y_cm, r->zone)) {
                        cond = true;
                        break;
                    }
                }
                break;
            case ON_RULE_TRIG_OUT_FENCE:
                if (est_cnt > 0u) {
                    cond = true;
                    for (uint32_t ti = 0u; ti < est_cnt; ++ti) {
                        if (on_loc_check_geofence(ctx, ests[ti].x_cm, ests[ti].y_cm, r->zone)) {
                            cond = false;
                            break;
                        }
                    }
                }
                break;
            case ON_RULE_TRIG_CSI_MOTION:
                for (uint32_t ai = 0u; ai < ON_LOC_MAX_ANCHORS; ++ai) {
                    const on_loc_anchor_t *a = &ctx->anchors[ai];
                    if (!a->active || strcmp(a->room, r->zone) != 0 || a->csi_last_ms == 0u) continue;
                    uint32_t age = (now_ms >= a->csi_last_ms) ? (now_ms - a->csi_last_ms) : 0u;
                    if (age <= ON_LOC_OBS_STALE_MS && a->csi_motion_score >= r->csi_threshold) {
                        cond = true;
                        break;
                    }
                }
                break;
            case ON_RULE_TRIG_CSI_QUIET:
                for (uint32_t ai = 0u; ai < ON_LOC_MAX_ANCHORS; ++ai) {
                    const on_loc_anchor_t *a = &ctx->anchors[ai];
                    if (!a->active || strcmp(a->room, r->zone) != 0 || a->csi_last_ms == 0u) continue;
                    uint32_t age = (now_ms >= a->csi_last_ms) ? (now_ms - a->csi_last_ms) : 0u;
                    if (age <= ON_LOC_OBS_STALE_MS && a->csi_motion_score < r->csi_threshold) {
                        cond = true;
                        break;
                    }
                }
                break;
            default:
                break;
        }

        if (cond && !r->last_cond_met) {
            uint32_t since_fire = (now_ms >= r->last_fired_ms)
                                  ? (now_ms - r->last_fired_ms) : ON_LOC_RULE_COOLDOWN_MS;
            bool cooled = (r->last_fired_ms == 0u) || (since_fire >= ON_LOC_RULE_COOLDOWN_MS);
            if (cooled) {
                r->fire_count++;
                r->last_fired_ms = now_ms;
                ctx->last_fired_rule_id = r->id;
                snprintf(ctx->last_fired_summary, sizeof(ctx->last_fired_summary),
                         "R#%u [%s] -> %s|%s",
                         (unsigned)r->id, r->zone, r->cmd_tag, r->cmd_payload);
                if (on_fire) on_fire(user_ctx, r);
                fired++;
            }
        }
        r->last_cond_met = cond;
    }
    return fired;
}

/* ========================================================================== */
/* v7.2 Out-of-the-Box 4-Room + Geofence + Wi-Fi CSI Showcase Demo            */
/* ========================================================================== */

void on_loc_load_demo_preset(on_loc_ctx_t *ctx, const uint8_t gw_mac48[6], uint32_t now_ms) {
    if (!ctx) return;
    uint8_t m0[6] = { 0xA4u, 0xCFu, 0x12u, 0x72u, 0x00u, 0x01u };
    if (loc_mac_nonzero(gw_mac48)) memcpy(m0, gw_mac48, 6u);
    const uint8_t m1[6] = { 0xA4u, 0xCFu, 0x12u, 0x72u, 0x00u, 0x02u };
    const uint8_t m2[6] = { 0xA4u, 0xCFu, 0x12u, 0x72u, 0x00u, 0x03u };
    const uint8_t m3[6] = { 0xA4u, 0xCFu, 0x12u, 0x72u, 0x00u, 0x04u };

    on_loc_set_anchor(ctx, m0, 0x001u, "Living",  0,   0);
    on_loc_set_anchor(ctx, m1, 0x002u, "Study",   500, 0);
    on_loc_set_anchor(ctx, m2, 0x003u, "Bedroom", 500, 360);
    on_loc_set_anchor(ctx, m3, 0x004u, "Kitchen", 0,   360);

    on_loc_set_geofence(ctx, "StudyDesk",   470, 50,  140u);
    on_loc_set_geofence(ctx, "KitchenZone", 50,  330, 120u);

    on_loc_set_rule(ctx, 1u, (uint8_t)ON_RULE_TRIG_ENTER_ROOM, "Study",       40u, "Study",   "RELAY:1");
    on_loc_set_rule(ctx, 2u, (uint8_t)ON_RULE_TRIG_IN_FENCE,   "KitchenZone", 40u, "Kitchen", "MODE:STROBE");
    on_loc_set_rule(ctx, 3u, (uint8_t)ON_RULE_TRIG_CSI_MOTION, "Living",      45u, "Living",  "MODE:PULSE");
    on_loc_set_rule(ctx, 4u, (uint8_t)ON_RULE_TRIG_CSI_QUIET,  "Living",      15u, "Living",  "MODE:SYNC");

    on_loc_step_demo(ctx, 0u, now_ms);
}

void on_loc_step_demo(on_loc_ctx_t *ctx, uint32_t step_idx, uint32_t now_ms) {
    if (!ctx) return;
    /* Ensure at least 4 active anchors exist */
    uint32_t acnt = 0u;
    for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
        if (ctx->anchors[i].active) acnt++;
    }
    if (acnt == 0u) {
        on_loc_load_demo_preset(ctx, NULL, now_ms);
        return;
    }

    /* 4-stage demo trajectory for virtual iBeacon badge & cat collar + Wi-Fi CSI wave */
    static const int32_t kBadgeWaypoints[4][2] = {
        { 90,  70  }, /* Stage 0: Living Room (CSI high motion = 72) */
        { 460, 40  }, /* Stage 1: Study Desk Geofence (triggers Rule #1 RELAY:1) */
        { 450, 310 }, /* Stage 2: Bedroom (Living CSI quiet = 8 -> triggers Rule #4 MODE:SYNC) */
        { 40,  330 }  /* Stage 3: Kitchen Forbidden Geofence (triggers Rule #2 MODE:STROBE) */
    };
    static const uint8_t kLivingCsi[4] = { 76u, 28u, 8u, 58u };
    static const uint8_t kStudyCsi[4]  = { 10u, 64u, 22u, 12u };

    uint32_t st = step_idx & 3u;
    double tx = (double)kBadgeWaypoints[st][0];
    double ty = (double)kBadgeWaypoints[st][1];
    const uint8_t badge_mac[6] = { 0x02u, 0xD7u, 0x03u, 0xE9u, 0x00u, 0x58u };

    /* Reset previous demo target smoothing so each stage snaps cleanly to the new room/geofence */
    for (uint32_t ti = 0u; ti < ON_LOC_MAX_TARGETS; ++ti) {
        if (ctx->targets[ti].active && memcmp(ctx->targets[ti].mac48, badge_mac, 6u) == 0) {
            ctx->targets[ti].pos_inited = false;
            memset(ctx->targets[ti].obs, 0, sizeof(ctx->targets[ti].obs));
            break;
        }
    }

    /* Feed multi-anchor RSSI samples so median+EMA converges smoothly to waypoint */
    for (uint32_t rep = 0u; rep < 3u; ++rep) {
        for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
            on_loc_anchor_t *a = &ctx->anchors[i];
            if (!a->active) continue;
            double dx = (tx - (double)a->x_cm) / 100.0;
            double dy = (ty - (double)a->y_cm) / 100.0;
            double dm = sqrt(dx * dx + dy * dy);
            if (dm < 0.25) dm = 0.25;
            int8_t rssi = (int8_t)lroundf(-59.0f - 10.0f * 2.5f * log10f((float)dm));
            on_loc_observe_beacon(ctx, a->mac48, badge_mac, ON_LOC_SRC_IBEACON,
                                  -59, rssi, "iB:DemoBadge", now_ms);
        }
    }

    /* Update Wi-Fi CSI scores across anchors */
    for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
        on_loc_anchor_t *a = &ctx->anchors[i];
        if (!a->active) continue;
        uint8_t csi = (i == 0u) ? kLivingCsi[st] : ((i == 1u) ? kStudyCsi[st] : (uint8_t)(12u + ((st + i) * 14u) % 45u));
        a->csi_last_ms = 0u;
        on_loc_observe_csi(ctx, a->mac48, a->short_id, csi, NULL, now_ms);
    }
}

size_t on_loc_format_json(on_loc_ctx_t *ctx, uint32_t now_ms,
                          char *out_buf, size_t out_cap) {
    if (!ctx || !out_buf || out_cap < 128u) return 0u;
    int pos = snprintf(out_buf, out_cap, "{\"anchors\":[");
    if (pos <= 0 || (size_t)pos >= out_cap) return 0u;

    bool first = true;
    for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
        const on_loc_anchor_t *a = &ctx->anchors[i];
        if (!a->active) continue;
        char mac_s[18];
        loc_fmt_mac(a->mac48, mac_s);
        uint32_t csi_age = (now_ms >= a->csi_last_ms) ? (now_ms - a->csi_last_ms) : 0u;
        uint8_t csi_val = (a->csi_last_ms > 0u && csi_age <= ON_LOC_OBS_STALE_MS)
                          ? a->csi_motion_score : 0u;
        uint8_t csi_st  = (csi_val >= 45u) ? 2u : ((csi_val >= 15u) ? 1u : 0u);
        int n = snprintf(out_buf + pos, out_cap - (size_t)pos,
                         "%s{\"mac\":\"%s\",\"id\":\"0x%03X\",\"room\":\"%s\","
                         "\"x_cm\":%ld,\"y_cm\":%ld,\"csi\":%u,\"csi_st\":%u,\"bins\":[%u,%u,%u,%u,%u,%u,%u,%u]}",
                         first ? "" : ",", mac_s, (unsigned)a->short_id,
                         a->room, (long)a->x_cm, (long)a->y_cm,
                         (unsigned)csi_val, (unsigned)csi_st,
                         (unsigned)a->csi_bins[0],  (unsigned)a->csi_bins[2],
                         (unsigned)a->csi_bins[4],  (unsigned)a->csi_bins[6],
                         (unsigned)a->csi_bins[8],  (unsigned)a->csi_bins[10],
                         (unsigned)a->csi_bins[12], (unsigned)a->csi_bins[14]);
        if (n <= 0 || (size_t)(pos + n) >= out_cap - 96u) break;
        pos += n;
        first = false;
    }

    int mid = snprintf(out_buf + pos, out_cap - (size_t)pos, "],\"targets\":[");
    if (mid <= 0 || (size_t)(pos + mid) >= out_cap) return 0u;
    pos += mid;

    first = true;
    for (uint32_t i = 0u; i < ON_LOC_MAX_TARGETS; ++i) {
        if (!ctx->targets[i].active) continue;
        on_loc_estimate_t est;
        if (!on_loc_solve(ctx, ctx->targets[i].mac48, now_ms, &est)) continue;
        char mac_s[18];
        loc_fmt_mac(est.mac48, mac_s);
        int n = snprintf(out_buf + pos, out_cap - (size_t)pos,
                         "%s{\"mac\":\"%s\",\"id\":\"0x%03X\",\"src\":\"%s\",\"label\":\"%s\","
                         "\"tx1m\":%d,\"room\":\"%s\","
                         "\"x_cm\":%ld,\"y_cm\":%ld,\"rssi\":%d,\"anchors\":%u,\"err_cm\":%u}",
                         first ? "" : ",", mac_s, (unsigned)est.short_id,
                         loc_src_name(est.source), est.label, (int)est.tx_power_1m_dbm,
                         est.room, (long)est.x_cm, (long)est.y_cm, (int)est.best_rssi_dbm,
                         (unsigned)est.anchor_count, (unsigned)est.est_err_cm);
        if (n <= 0 || (size_t)(pos + n) >= out_cap - 96u) break;
        pos += n;
        first = false;
    }

    int fm = snprintf(out_buf + pos, out_cap - (size_t)pos, "],\"fences\":[");
    if (fm <= 0 || (size_t)(pos + fm) >= out_cap) return 0u;
    pos += fm;

    first = true;
    for (uint32_t i = 0u; i < ON_LOC_MAX_GEOFENCES; ++i) {
        const on_loc_geofence_t *f = &ctx->fences[i];
        if (!f->active) continue;
        int n = snprintf(out_buf + pos, out_cap - (size_t)pos,
                         "%s{\"name\":\"%s\",\"x_cm\":%ld,\"y_cm\":%ld,\"r_cm\":%u,\"in\":%u}",
                         first ? "" : ",", f->name, (long)f->x_cm, (long)f->y_cm,
                         (unsigned)f->radius_cm, f->occupied ? 1u : 0u);
        if (n <= 0 || (size_t)(pos + n) >= out_cap - 64u) break;
        pos += n;
        first = false;
    }

    int rm = snprintf(out_buf + pos, out_cap - (size_t)pos, "],\"rules\":[");
    if (rm <= 0 || (size_t)(pos + rm) >= out_cap) return 0u;
    pos += rm;

    first = true;
    for (uint32_t i = 0u; i < ON_LOC_MAX_RULES; ++i) {
        const on_loc_rule_t *r = &ctx->rules[i];
        if (!r->active) continue;
        int n = snprintf(out_buf + pos, out_cap - (size_t)pos,
                         "%s{\"id\":%u,\"trig\":%u,\"zone\":\"%s\",\"th\":%u,"
                         "\"tag\":\"%s\",\"cmd\":\"%s\",\"on\":%u,\"fires\":%u}",
                         first ? "" : ",", (unsigned)r->id, (unsigned)r->trig_type,
                         r->zone, (unsigned)r->csi_threshold,
                         r->cmd_tag, r->cmd_payload,
                         r->last_cond_met ? 1u : 0u, (unsigned)r->fire_count);
        if (n <= 0 || (size_t)(pos + n) >= out_cap - 48u) break;
        pos += n;
        first = false;
    }

    int tail = snprintf(out_buf + pos, out_cap - (size_t)pos, "],\"last_evt\":\"%s\"}",
                        ctx->last_fired_summary);
    if (tail <= 0 || (size_t)(pos + tail) >= out_cap) return 0u;
    return (size_t)(pos + tail);
}
