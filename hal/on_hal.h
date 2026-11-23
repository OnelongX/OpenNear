#ifndef OPENNEAR_HAL_H
#define OPENNEAR_HAL_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear Platform Hardware Abstraction Layer (HAL)
 *
 * Implemented identically by:
 *   - sim/sim_world.c               (Host discrete-event multi-node radio simulator)
 *   - ports/esp32_wroom32/          (Route C: ESP32 802.11 0xD0 raw TX + rx_ctrl.timestamp)
 *   - ports/nrf52840_bare/          (Route A: Nordic NRF_RADIO + PPI + TIMER0 + GPIOTE)
 */
typedef struct on_hal_ops {
    void *user_ctx;

    /* Monotonic local hardware timer in nanoseconds */
    int64_t  (*now_ns)(void *ctx);

    /* Transmit raw OpenNear frame (starting with 6-byte MAC header).
     * If tx_at_local_ns > 0, hardware triggers TX at that exact local timestamp (PPI / timer). */
    on_err_t (*tx_raw)(void *ctx, const uint8_t *frame, size_t len, int64_t tx_at_local_ns);

    /* Control radio power state & RF channel (TSCH/AFH) for micro-amp TDMA deep sleep */
    void     (*radio_set_rx)(void *ctx, bool enable);
    void     (*radio_sleep)(void *ctx);
    void     (*radio_set_channel)(void *ctx, uint8_t ch_idx);

    /* Synchronized ACT/QUIET hardware action hook (e.g., GPIO/LED toggle) */
    void     (*on_act_trigger)(void *ctx, uint32_t sf_seq, uint8_t act_cmd, int64_t local_ns);

    /* Read 48-bit factory hardware MAC address */
    void     (*get_mac48)(void *ctx, uint8_t mac_out[6]);

    /* Optional structured event callback / logger */
    void     (*on_event)(void *ctx, const char *event_name, uint32_t arg1, int64_t arg2);

    /* True when TX is queued and its requested time is not an on-air timestamp. */
    bool     tx_timing_best_effort;
} on_hal_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_HAL_H */
