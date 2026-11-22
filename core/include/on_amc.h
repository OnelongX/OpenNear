#ifndef OPENNEAR_AMC_H
#define OPENNEAR_AMC_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v3.0 L1 Adaptive Modulation & Coding (AMC) Dual-Waveform State Machine
 *
 * Smoothly adapts between:
 *   - ON_PHY_LE_MSK_2M   (Ultra-robust / zero-PAPR wearable mode, min SNR >= 0 dB)
 *   - ON_PHY_LE_QPSK_4M  (Balanced single-carrier mode, min SNR >= 7 dB)
 *   - ON_PHY_HS_OFDM_20M (High-throughput multi-carrier OFDM bus, min SNR >= 15 dB)
 * Uses asymmetric hysteresis (fast downgrade on PER spike, conservative upgrade on clean SNR)
 * so links never oscillate under multipath fading.
 */
typedef struct {
    on_phy_mode_t current_mode;
    float         snr_ewma_db;
    float         per_ewma;
    uint32_t      clean_streak;
    uint32_t      upgrades;
    uint32_t      downgrades;
} on_amc_ctx_t;

void          on_amc_init(on_amc_ctx_t *ctx, on_phy_mode_t initial_mode);
on_phy_mode_t on_amc_update(on_amc_ctx_t *ctx, float inst_snr_db, bool rx_ok);
uint32_t      on_amc_air_time_us(on_phy_mode_t mode, size_t frame_bytes);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_AMC_H */
