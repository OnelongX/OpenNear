#ifndef OPENNEAR_ESP32_DEMO_APP_H
#define OPENNEAR_ESP32_DEMO_APP_H

#include "on_node.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v6.9 Out-of-the-Box Multi-Board Sync & Telemetry Showcase Demo
 *
 * Activated with -DOPENNEAR_ENABLE_DEMO_APP=1 (used by [env:esp32_demo_sync_node]
 * and optionally on gateways). Zero external library dependencies!
 *
 * Supported downlink / MQTT / Web commands (sent via CMD:FFF|<cmd> or opennear/cmd/<MAC>):
 *   MODE:SYNC    - Superframe-locked synchronized blink (default, proves failover continuity)
 *   MODE:STROBE  - Fast 8 Hz synchronized flash across all 0~6-hop boards
 *   MODE:PULSE   - Global sf_seq-locked rhythm pattern (all boards stay in exact phase)
 *   RELAY:1 / 0  - Toggle actuator GPIO (default GPIO26) & immediately reflect in telemetry
 *   PING         - Trigger immediate visual confirmation flash
 *
 * Live 8-byte TDMA telemetry payload (visible in Web Portal & Home Assistant):
 *   ASCII compact format e.g. "S1:1234" -> [Mode char][Relay 0/1]:[4-digit ADC mV]
 */

#ifndef OPENNEAR_DEMO_RELAY_GPIO
#define OPENNEAR_DEMO_RELAY_GPIO 26
#endif

#ifndef OPENNEAR_DEMO_ADC_GPIO
#define OPENNEAR_DEMO_ADC_GPIO 34
#endif

void on_esp32_demo_init(void);

/* Handle inbound downlink control text from G / Web / MQTT / BLE */
void on_esp32_demo_on_cmd(const uint8_t *payload, size_t len);

/* Periodic Core-0 tick (~20 Hz): updates live 8B TDMA telemetry & visual demo pattern */
void on_esp32_demo_poll(on_node_t *node);

/* Optional hook called on each synchronized ACT superframe edge (returns desired LED level 0/1) */
uint8_t on_esp32_demo_act_level(uint32_t sf_seq, uint8_t default_level);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_ESP32_DEMO_APP_H */
