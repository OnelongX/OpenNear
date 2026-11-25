#ifndef OPENNEAR_HAL_ESP32_RAW80211_H
#define OPENNEAR_HAL_ESP32_RAW80211_H

#include "opennear_config.h"
#include "on_hal.h"
#include "on_node.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Route C: ESP32-WROOM-32 Raw 802.11 0xD0 Vendor Action Carrier
 *
 * - Runs OpenNear 6B Short-ID + 4-Phase Superframe + Sub-G + Backup_ID over
 *   raw 802.11 0xD0 frames @ 54 Mbps (esp_wifi_config_80211_tx_rate before start).
 * - Captures hardware baseband rx_ctrl.timestamp (32-bit 1 us counter) and unwraps
 *   it cleanly into 64-bit monotonic nanoseconds (immune to 71.5-min wraparound).
 * - Drives GPIO2 (onboard LED) synchronously during the ACT/QUIET guard band.
 */

#define ON_ESP32_DEFAULT_CHANNEL  6u
#define ON_ESP32_ACT_LED_GPIO     2
#define ON_ESP32_MASTER_STRAP_PIN 4  /* Pull LOW at boot to force Primary G role */

typedef struct {
    uint8_t channel;
    int     act_gpio;
    bool    share_existing_wifi; /* True when sharing Wi-Fi STA with public Internet TCP/IP */
} on_esp32_cfg_t;

/* Initialize Wi-Fi STA + 54M raw TX + promiscuous 0xD0 RX filter + LED GPIO */
on_err_t on_esp32_hal_init(const on_esp32_cfg_t *cfg, on_hal_ops_t *hal_out);

/* v5.0: Lock/sync OpenNear channel to connected public Wi-Fi AP channel (1..13) */
void     on_esp32_hal_lock_wifi_channel(bool lock, uint8_t wifi_ch);
uint8_t  on_esp32_hal_current_wifi_channel(void);

/* Drain lock-free ISR RX ring buffer into the OpenNear node state machine */
void     on_esp32_hal_poll_rx(on_node_t *node);

/* Wi-Fi driver submissions & RX ring overflow counter */
void     on_esp32_hal_tx_counts(uint32_t *accepted, uint32_t *failed);
uint32_t on_esp32_hal_rx_drops(void);

#ifdef OPENNEAR_ACT_TRACE
typedef struct {
    uint32_t seq, act_sf, beacon_sf, rx_hw_us;
    uint16_t sf_us, act_offset_us;
    uint8_t level, role, scheduled;
    int64_t planned_us, step_us, pre_write_us, post_write_us;
    int64_t rx_mapped_us, rx_callback_us, rx_dequeue_us, rx_accepted_us;
} on_esp32_act_trace_t;
bool on_esp32_hal_trace_init(void);
bool on_esp32_hal_trace_read(on_esp32_act_trace_t *trace);
#endif

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_HAL_ESP32_RAW80211_H */
