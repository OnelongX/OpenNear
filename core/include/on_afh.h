#ifndef OPENNEAR_AFH_H
#define OPENNEAR_AFH_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v2.0 TSCH Adaptive Frequency Hopping (AFH) & Dynamic Blacklist Engine
 *
 * Inspired by Contiki-NG TSCH, DotBots/Mari, and ExpressLRS:
 * - Divides 2.4 GHz ISM band into 16 orthogonal 2 MHz logical channels
 *   (2402, 2406, 2410, ..., 2462..2478 MHz).
 * - Zero-negotiation deterministic channel schedule:
 *     ch = active_list[(sf_seq * 7 + slot_offset * 3) % active_count]
 *   so G, 1-hop T, Sub-G, and wall-blocked 2-hop edge nodes all hop in unison!
 * - Primary G tracks per-channel packet delivery vs. misses; congested channels
 *   (e.g. Wi-Fi 20 MHz interference) are automatically blacklisted in the 16-bit
 *   channel_map broadcast inside SYNC/MAP & SUB_SYNC.
 */

#define ON_AFH_NUM_CHANNELS     16u
#define ON_AFH_ALL_CHANNELS     0xFFFFu
#define ON_AFH_MIN_ACTIVE_CH    4u     /* Regulatory & diversity minimum active channels */
#define ON_AFH_BAD_MISS_THRESH  3u     /* 3 consecutive misses on a channel -> blacklist */
#define ON_AFH_REPROBE_INTERVAL 120u   /* Re-probe blacklisted channels after 120 SFs */

typedef struct {
    bool     enabled;
    uint16_t channel_map;                     /* 16-bit active channel mask (1=clean, 0=blacklisted) */
    uint8_t  ch_miss_streak[ON_AFH_NUM_CHANNELS];
    uint16_t ch_rx_ok[ON_AFH_NUM_CHANNELS];
    uint16_t ch_rx_miss[ON_AFH_NUM_CHANNELS];
    uint32_t blacklist_until_sf[ON_AFH_NUM_CHANNELS];
    uint32_t blacklist_events;
    uint8_t  current_ch;
} on_afh_ctx_t;

void     on_afh_init(on_afh_ctx_t *ctx, bool enable);

/* Map logical channel index (0..15) -> 2.4 GHz RF offset in MHz (for nRF52 RADIO->FREQUENCY) */
uint8_t  on_afh_ch_to_rf_offset_mhz(uint8_t ch_idx);

/* Compute deterministic TSCH physical channel index (0..15) for (sf_seq, slot_offset) */
uint8_t  on_afh_channel_for(const on_afh_ctx_t *ctx, uint32_t sf_seq, uint8_t slot_offset);

/* Apply network-wide channel_map received in SYNC/MAP or SUB_SYNC */
void     on_afh_apply_map(on_afh_ctx_t *ctx, uint16_t new_map);

/* Primary G channel quality feedback (per-slot success or expected-slot miss) */
void     on_afh_report_rx(on_afh_ctx_t *ctx, uint8_t ch_idx, bool success, uint32_t sf_seq);

/* Periodic maintenance: re-enable expired blacklisted channels for probing */
void     on_afh_tick(on_afh_ctx_t *ctx, uint32_t sf_seq);

/* Count currently active (non-blacklisted) channels */
uint8_t  on_afh_active_count(const on_afh_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_AFH_H */
