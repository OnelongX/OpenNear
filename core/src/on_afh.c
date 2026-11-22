#include "on_afh.h"
#include <string.h>

/* 16 well-spaced 2.4 GHz ISM frequencies (MHz offset above 2400 MHz) */
static const uint8_t kChFreqOffsetMhz[ON_AFH_NUM_CHANNELS] = {
    2u,  6u,  10u, 14u,   /* Below/low Wi-Fi CH1 */
    22u, 26u, 30u, 34u,   /* Mid band (overlaps Wi-Fi CH6 ~2437 MHz) */
    42u, 46u, 50u, 54u,   /* Upper mid band */
    62u, 66u, 72u, 78u    /* High ISM / above Wi-Fi CH11 */
};

void on_afh_init(on_afh_ctx_t *ctx, bool enable) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->enabled     = enable;
    ctx->channel_map = ON_AFH_ALL_CHANNELS;
    ctx->current_ch  = 0u;
}

uint8_t on_afh_ch_to_rf_offset_mhz(uint8_t ch_idx) {
    return kChFreqOffsetMhz[ch_idx & 0x0Fu];
}

uint8_t on_afh_active_count(const on_afh_ctx_t *ctx) {
    if (!ctx) return 0u;
    uint16_t m = ctx->channel_map;
    uint8_t cnt = 0u;
    for (uint8_t i = 0u; i < ON_AFH_NUM_CHANNELS; ++i) {
        if (m & (1u << i)) cnt++;
    }
    return cnt;
}

uint8_t on_afh_channel_for(const on_afh_ctx_t *ctx, uint32_t sf_seq, uint8_t slot_offset) {
    if (!ctx || !ctx->enabled) {
        return 0u;
    }
    uint8_t active[ON_AFH_NUM_CHANNELS];
    uint8_t count = 0u;
    uint16_t mask = ctx->channel_map ? ctx->channel_map : ON_AFH_ALL_CHANNELS;
    for (uint8_t ch = 0u; ch < ON_AFH_NUM_CHANNELS; ++ch) {
        if (mask & (1u << ch)) {
            active[count++] = ch;
        }
    }
    if (count == 0u) return 0u;

    /* Prime stride 17 (> 16) is coprime with EVERY active count 1..16, guaranteeing full-band coverage */
    uint32_t idx = (sf_seq * 17u + (uint32_t)slot_offset * 3u) % (uint32_t)count;
    return active[idx];
}

void on_afh_apply_map(on_afh_ctx_t *ctx, uint16_t new_map) {
    if (!ctx || new_map == 0u) return;
    ctx->channel_map = new_map;
}

void on_afh_report_rx(on_afh_ctx_t *ctx, uint8_t ch_idx, bool success, uint32_t sf_seq) {
    if (!ctx || !ctx->enabled || ch_idx >= ON_AFH_NUM_CHANNELS) return;

    if (success) {
        ctx->ch_rx_ok[ch_idx]++;
        ctx->ch_miss_streak[ch_idx] = 0u;
        return;
    }

    ctx->ch_rx_miss[ch_idx]++;
    if (ctx->ch_miss_streak[ch_idx] < 255u) {
        ctx->ch_miss_streak[ch_idx]++;
    }

    if (ctx->ch_miss_streak[ch_idx] >= ON_AFH_BAD_MISS_THRESH &&
        (ctx->channel_map & (1u << ch_idx)) != 0u &&
        on_afh_active_count(ctx) > ON_AFH_MIN_ACTIVE_CH) {
        ctx->channel_map &= (uint16_t)(~(1u << ch_idx));
        ctx->blacklist_until_sf[ch_idx] = sf_seq + ON_AFH_REPROBE_INTERVAL;
        ctx->ch_miss_streak[ch_idx] = 0u;
        ctx->blacklist_events++;
    }
}

void on_afh_tick(on_afh_ctx_t *ctx, uint32_t sf_seq) {
    if (!ctx || !ctx->enabled) return;
    for (uint8_t ch = 0u; ch < ON_AFH_NUM_CHANNELS; ++ch) {
        if ((ctx->channel_map & (1u << ch)) == 0u &&
            ctx->blacklist_until_sf[ch] > 0u &&
            sf_seq >= ctx->blacklist_until_sf[ch]) {
            /* Re-admit channel for periodic clean-band probing */
            ctx->channel_map |= (uint16_t)(1u << ch);
            ctx->blacklist_until_sf[ch] = 0u;
            ctx->ch_miss_streak[ch] = 0u;
        }
    }
}
