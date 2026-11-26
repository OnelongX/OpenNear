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
    } else if (ctx->current_mode == ON_PHY_LE_MSK_2M &&
               ctx->snr_ewma_db < -1.5f) {
        ctx->current_mode = ON_PHY_LR_DSSS_1M;
        ctx->clean_streak = 0u;
        ctx->downgrades++;
    } else if (ctx->current_mode == ON_PHY_LR_DSSS_1M &&
               ctx->snr_ewma_db < -5.0f) {
        ctx->current_mode = ON_PHY_LR_SPREAD_250K;
        ctx->clean_streak = 0u;
        ctx->downgrades++;
    }

    /* Conservative upgrade after 8 consecutive clean frames with >= 2 dB hysteresis margin */
    if (ctx->clean_streak >= 8u && ctx->per_ewma < 0.02f) {
        if (ctx->current_mode == ON_PHY_LR_SPREAD_250K && ctx->snr_ewma_db >= -2.5f) {
            ctx->current_mode = ON_PHY_LR_DSSS_1M;
            ctx->clean_streak = 0u;
            ctx->upgrades++;
        } else if (ctx->current_mode == ON_PHY_LR_DSSS_1M && ctx->snr_ewma_db >= 2.0f) {
            ctx->current_mode = ON_PHY_LE_MSK_2M;
            ctx->clean_streak = 0u;
            ctx->upgrades++;
        } else if (ctx->current_mode == ON_PHY_LE_MSK_2M && ctx->snr_ewma_db >= 8.5f) {
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
        case ON_PHY_LE_MSK_2M:      return (uint32_t)(8u  + frame_bytes * 4u);  /* 2 Mbps */
        case ON_PHY_LE_QPSK_4M:     return (uint32_t)(8u  + frame_bytes * 2u);  /* 4 Mbps */
        case ON_PHY_HS_OFDM_20M:    return (uint32_t)(16u + (frame_bytes + 4u) / 5u); /* ~40 Mbps OFDM */
        case ON_PHY_ESP32_RAW54M:   return (uint32_t)(20u + (frame_bytes + 6u) / 7u); /* 54 Mbps OFDM */
        case ON_PHY_LR_DSSS_1M:     return (uint32_t)(48u + frame_bytes * 8u);  /* 1 Mbps DSSS Long-Range */
        case ON_PHY_LR_SPREAD_250K: return (uint32_t)(80u + frame_bytes * 16u); /* 250-500 Kbps LR Spread */
        default:                    return (uint32_t)(20u + frame_bytes * 2u);
    }
}

int8_t on_amc_sensitivity_dbm(on_phy_mode_t mode) {
    switch (mode) {
        case ON_PHY_ESP32_RAW54M:   return -75;  /* 54 Mbps 64-QAM OFDM */
        case ON_PHY_HS_OFDM_20M:    return -80;  /* 20 Mbps OFDM */
        case ON_PHY_LE_QPSK_4M:     return -87;  /* 4 Mbps QPSK */
        case ON_PHY_LE_MSK_2M:      return -93;  /* 2 Mbps GFSK/MSK */
        case ON_PHY_LR_DSSS_1M:     return -98;  /* 1 Mbps DSSS Barker-11 */
        case ON_PHY_LR_SPREAD_250K: return -105; /* 250 Kbps LR Spread + CA-Polar Soft-HARQ */
        default:                    return -90;
    }
}

uint32_t on_amc_est_los_range_m(on_phy_mode_t mode, int8_t tx_power_dbm) {
    /* Practical 2.4 GHz outdoor line-of-sight range with 10 dB fade margin (FSPL @ 1m = 40 dB, n ~ 2.35) */
    int32_t budget_db = (int32_t)tx_power_dbm - (int32_t)on_amc_sensitivity_dbm(mode);
    if (budget_db <= 85)  return 35u;
    if (budget_db <= 95)  return 75u;   /* +19.5 dBm @ 54M (-75 dBm) -> ~75 m */
    if (budget_db <= 105) return 150u;
    if (budget_db <= 113) return 280u;  /* +20 dBm @ 2M MSK (-93 dBm) -> ~280 m */
    if (budget_db <= 119) return 500u;  /* +20 dBm @ 1M DSSS (-98 dBm) -> ~500 m */
    return 800u;                        /* +20 dBm @ 250K LR + Polar (-105 dBm, 125 dB budget) -> ~800 m (700m+ class) */
}
