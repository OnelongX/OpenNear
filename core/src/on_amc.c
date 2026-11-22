#include "on_amc.h"
#include <string.h>

void on_amc_init(on_amc_ctx_t *ctx, on_phy_mode_t initial_mode) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->current_mode = initial_mode;
    ctx->snr_ewma_db  = 10.0f;
    ctx->per_ewma     = 0.0f;
}

on_phy_mode_t on_amc_update(on_amc_ctx_t *ctx, float inst_snr_db, bool rx_ok) {
    if (!ctx) return ON_PHY_LE_MSK_2M;

    /* EWMA tracking (alpha = 0.25) */
    ctx->snr_ewma_db = 0.75f * ctx->snr_ewma_db + 0.25f * inst_snr_db;
    ctx->per_ewma    = 0.75f * ctx->per_ewma    + 0.25f * (rx_ok ? 0.0f : 1.0f);

    if (rx_ok) {
        ctx->clean_streak++;
    } else {
        ctx->clean_streak = 0u;
    }

    /* Fast downgrade on high PER (> 15%) or SNR margin collapse */
    if (ctx->current_mode == ON_PHY_HS_OFDM_20M &&
        (ctx->per_ewma > 0.15f || ctx->snr_ewma_db < 13.0f)) {
        ctx->current_mode = ON_PHY_LE_QPSK_4M;
        ctx->clean_streak = 0u;
        ctx->downgrades++;
    } else if (ctx->current_mode == ON_PHY_LE_QPSK_4M &&
               (ctx->per_ewma > 0.15f || ctx->snr_ewma_db < 5.5f)) {
        ctx->current_mode = ON_PHY_LE_MSK_2M;
        ctx->clean_streak = 0u;
        ctx->downgrades++;
    }

    /* Conservative upgrade after 8 consecutive clean frames with >= 2 dB hysteresis margin */
    if (ctx->clean_streak >= 8u && ctx->per_ewma < 0.02f) {
        if (ctx->current_mode == ON_PHY_LE_MSK_2M && ctx->snr_ewma_db >= 8.5f) {
            ctx->current_mode = ON_PHY_LE_QPSK_4M;
            ctx->clean_streak = 0u;
            ctx->upgrades++;
        } else if (ctx->current_mode == ON_PHY_LE_QPSK_4M && ctx->snr_ewma_db >= 17.0f) {
            ctx->current_mode = ON_PHY_HS_OFDM_20M;
            ctx->clean_streak = 0u;
            ctx->upgrades++;
        }
    }

    return ctx->current_mode;
}

uint32_t on_amc_air_time_us(on_phy_mode_t mode, size_t frame_bytes) {
    switch (mode) {
        case ON_PHY_LE_MSK_2M:   return (uint32_t)(8u  + frame_bytes * 4u);  /* 2 Mbps */
        case ON_PHY_LE_QPSK_4M:  return (uint32_t)(8u  + frame_bytes * 2u);  /* 4 Mbps */
        case ON_PHY_HS_OFDM_20M: return (uint32_t)(16u + (frame_bytes + 4u) / 5u); /* ~40 Mbps OFDM */
        default:                 return (uint32_t)(20u + frame_bytes * 2u);
    }
}
