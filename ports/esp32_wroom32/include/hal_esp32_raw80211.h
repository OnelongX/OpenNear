#ifndef OPENNEAR_HAL_ESP32_RAW80211_H
#define OPENNEAR_HAL_ESP32_RAW80211_H

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
} on_esp32_cfg_t;

/* Initialize Wi-Fi STA + 54M raw TX + promiscuous 0xD0 RX filter + LED GPIO */
on_err_t on_esp32_hal_init(const on_esp32_cfg_t *cfg, on_hal_ops_t *hal_out);

/* Drain lock-free ISR RX ring buffer into the OpenNear node state machine */
void     on_esp32_hal_poll_rx(on_node_t *node);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_HAL_ESP32_RAW80211_H */
