#ifndef OPENNEAR_ESP32_NET_GATEWAY_H
#define OPENNEAR_ESP32_NET_GATEWAY_H

#include "on_node.h"
#include "on_gateway.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ESP32-WROOM-32 Public Network Wi-Fi STA + BLE GATT Gateway Bridge
 *
 * - Public Wi-Fi STA (OPENNEAR_ENABLE_WIFI_GW):
 *   Connects to router/hotspot via NVS or compile-time SSID/PASS, automatically
 *   aligns OpenNear's raw 802.11 carrier channel to WiFi.channel(), posts 1-hop &
 *   2-hop node telemetry JSON to cloud HTTP endpoints, and downloads OTA firmware
 *   from HTTP URLs into the inactive OTA partition for self-flash or cluster OTA.
 *
 * - Bluetooth Low Energy GATT Bridge (OPENNEAR_ENABLE_BLE_GW):
 *   Exposes Service 0xFFE0:
 *     0xFFE1 (Notify) - Real-time OpenNear node telemetry & status JSON
 *     0xFFE2 (Write)  - Control & Provisioning ("WIFI:ssid|pass|url", "OTA:id|ver|url", "CMD:id|msg")
 *     0xFFE3 (Write)  - Direct BLE OTA trigger / raw command tunnel
 */

/* Initialize Public Wi-Fi STA and/or BLE GATT BEFORE on_esp32_hal_init (when sharing Wi-Fi) */
void on_esp32_net_gw_preinit_wifi(on_gw_ctx_t *gw, const char *default_ssid,
                                  const char *default_pass, const char *default_cloud_url);

/* Complete BLE GATT & transport bindings and attach gw to node */
void on_esp32_net_gw_start(on_node_t *node, on_gw_ctx_t *gw, const char *ble_name);

/* Run on Core 0 (Arduino loop): service Wi-Fi reconnect, AP channel sync, HTTP POST & Cloud OTA pull */
void on_esp32_net_gw_poll(on_node_t *node, on_gw_ctx_t *gw);

/* Cross-core spinlock helpers protecting shared on_node_t reads/writes between Core 0 and Core 1 */
void on_esp32_node_lock(void);
void on_esp32_node_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_ESP32_NET_GATEWAY_H */
