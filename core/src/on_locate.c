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

static void copy_room(char dst[ON_LOC_ROOM_MAX_LEN], const char *src) {
    size_t j = 0u;
    if (src) {
        for (size_t i = 0u; src[i] != '\0' && j + 1u < ON_LOC_ROOM_MAX_LEN; ++i) {
            unsigned char c = (unsigned char)src[i];
            if (c >= 0x20u && c != '"' && c != '\\') {
                dst[j++] = (char)c;
            }
        }
    }
    dst[j] = '\0';
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

on_err_t on_loc_observe_rssi(on_loc_ctx_t *ctx, const uint8_t anchor_mac48[6],
                             const uint8_t target_mac48[6], uint16_t target_short_id,
                             int8_t rssi_dbm, uint32_t now_ms) {
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
        if ((now_ms - o->last_ms) > ON_LOC_OBS_STALE_MS) continue;
        if (o->anchor_idx >= ON_LOC_MAX_ANCHORS || !ctx->anchors[o->anchor_idx].active) continue;

        const on_loc_anchor_t *a = &ctx->anchors[o->anchor_idx];
        float d = rssi_to_dist_cm(o->smooth_rssi, a->tx_power_1m_dbm, a->path_loss_n);
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
    if (cnt == 0u || best_i < 0) return false;

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

    out_est->valid         = true;
    memcpy(out_est->mac48, t->mac48, 6u);
    out_est->short_id      = t->short_id;
    copy_room(out_est->room, t->cur_room);
    out_est->x_cm          = (int32_t)lroundf(t->smooth_x_cm);
    out_est->y_cm          = (int32_t)lroundf(t->smooth_y_cm);
    out_est->best_rssi_dbm = (int8_t)lroundf(best_rssi);
    out_est->anchor_count  = cnt;
    out_est->est_err_cm    = (uint16_t)lroundf(rms);
    return true;
}

static void loc_fmt_mac(const uint8_t m[6], char out[18]) {
    snprintf(out, 18u, "%02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned)m[0], (unsigned)m[1], (unsigned)m[2],
             (unsigned)m[3], (unsigned)m[4], (unsigned)m[5]);
}

size_t on_loc_format_json(on_loc_ctx_t *ctx, uint32_t now_ms,
                          char *out_buf, size_t out_cap) {
    if (!ctx || !out_buf || out_cap < 64u) return 0u;
    int pos = snprintf(out_buf, out_cap, "{\"anchors\":[");
    if (pos <= 0 || (size_t)pos >= out_cap) return 0u;

    bool first = true;
    for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
        const on_loc_anchor_t *a = &ctx->anchors[i];
        if (!a->active) continue;
        char mac_s[18];
        loc_fmt_mac(a->mac48, mac_s);
        int n = snprintf(out_buf + pos, out_cap - (size_t)pos,
                         "%s{\"mac\":\"%s\",\"id\":\"0x%03X\",\"room\":\"%s\",\"x_cm\":%ld,\"y_cm\":%ld}",
                         first ? "" : ",", mac_s, (unsigned)a->short_id,
                         a->room, (long)a->x_cm, (long)a->y_cm);
        if (n <= 0 || (size_t)(pos + n) >= out_cap - 32u) break;
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
                         "%s{\"mac\":\"%s\",\"id\":\"0x%03X\",\"room\":\"%s\","
                         "\"x_cm\":%ld,\"y_cm\":%ld,\"rssi\":%d,\"anchors\":%u,\"err_cm\":%u}",
                         first ? "" : ",", mac_s, (unsigned)est.short_id, est.room,
                         (long)est.x_cm, (long)est.y_cm, (int)est.best_rssi_dbm,
                         (unsigned)est.anchor_count, (unsigned)est.est_err_cm);
        if (n <= 0 || (size_t)(pos + n) >= out_cap - 8u) break;
        pos += n;
        first = false;
    }

    int tail = snprintf(out_buf + pos, out_cap - (size_t)pos, "]}");
    if (tail <= 0 || (size_t)(pos + tail) >= out_cap) return 0u;
    return (size_t)(pos + tail);
}
