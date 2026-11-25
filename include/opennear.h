#ifndef OPENNEAR_CPP_FACADE_H
#define OPENNEAR_CPP_FACADE_H

/*
 * OpenNear High-Level Arduino / ESP32 C++ SDK Facade (v7.3.0)
 *
 * Drop-in 10-line API wrapping dual-core RT task scheduling, recursive locking,
 * NVS credential & geofence persistence, Wi-Fi/BLE coexistence, multi-gateway
 * zero-touch encrypted air-sync (0x05), and Wi-Fi CSI radar.
 *
 * Example:
 *   #include <OpenNear.h>
 *
 *   void setup() {
 *     OpenNear.begin(OpenNearConfig::Gateway()
 *       .withWifi("MySSID", "MyPass")
 *       .onCommand([](const char *cmd, size_t len) {
 *         Serial.printf("RX CMD: %.*s\n", (int)len, cmd);
 *       }));
 *   }
 *   void loop() {
 *     OpenNear.poll();
 *     OpenNear.sendTelemetry("T=24.5");
 *     delay(1000);
 *   }
 */

#include "opennear_core.h"

#if defined(__cplusplus) && defined(ARDUINO)
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*OpenNearCmdCallback)(const char *cmd, size_t len);
typedef void (*OpenNearRuleCallback)(uint8_t rule_id, const char *zone,
                                     const char *tag, const char *cmd);
typedef void (*OpenNearRoleCallback)(on_role_t new_role, uint16_t short_id);

struct OpenNearConfig {
    bool        is_gateway;
    bool        leaderless_boot;
    uint8_t     channel;
    int         act_gpio;
    uint8_t     capabilities;
    const char *wifi_ssid;
    const char *wifi_pass;
    const char *cloud_url;
    const char *ble_name;
    const uint8_t *net_key16; /* NULL = load NVS custom key or default demo key */
    OpenNearCmdCallback  cmd_cb;
    OpenNearRuleCallback rule_cb;
    OpenNearRoleCallback role_cb;

    static OpenNearConfig Gateway() {
        OpenNearConfig c;
        c.is_gateway      = true;
        c.leaderless_boot = true;
        c.channel         = 1u;
        c.act_gpio        = 2;
        c.capabilities    = 0x07u; /* Backup + Sub-G + Gateway Hot-Standby Priority */
        c.wifi_ssid       = "";
        c.wifi_pass       = "";
        c.cloud_url       = "";
        c.ble_name        = "OpenNear-G-GW";
        c.net_key16       = nullptr;
        c.cmd_cb          = nullptr;
        c.rule_cb         = nullptr;
        c.role_cb         = nullptr;
        return c;
    }

    static OpenNearConfig Node() {
        OpenNearConfig c  = Gateway();
        c.is_gateway      = false;
        c.capabilities    = 0x03u; /* Backup + Sub-G */
        c.ble_name        = "OpenNear-T-Node";
        return c;
    }

    OpenNearConfig &withChannel(uint8_t ch)                  { channel = ch; return *this; }
    OpenNearConfig &withActGpio(int gpio)                    { act_gpio = gpio; return *this; }
    OpenNearConfig &withLeaderless(bool en)                  { leaderless_boot = en; return *this; }
    OpenNearConfig &withWifi(const char *ssid, const char *pass, const char *url = "") {
        wifi_ssid = ssid ? ssid : "";
        wifi_pass = pass ? pass : "";
        cloud_url = url  ? url  : "";
        return *this;
    }
    OpenNearConfig &withBleName(const char *name)            { ble_name = name; return *this; }
    OpenNearConfig &withNetKey(const uint8_t key16[16])      { net_key16 = key16; return *this; }
    OpenNearConfig &onCommand(OpenNearCmdCallback cb)        { cmd_cb = cb; return *this; }
    OpenNearConfig &onRuleFired(OpenNearRuleCallback cb)     { rule_cb = cb; return *this; }
    OpenNearConfig &onRoleChange(OpenNearRoleCallback cb)    { role_cb = cb; return *this; }
};

class OpenNearClass {
public:
    OpenNearClass();

    /* Initialize HAL, AES-128 security, Mesh, OTA, Gateway & start Core-1 RT protocol task */
    bool begin(const OpenNearConfig &cfg = OpenNearConfig::Gateway());

    /* Call periodically from Arduino loop() (Core 0) to service Wi-Fi/MQTT/Web/BLE/Rules */
    void poll();

    /* Send up to 8 bytes of custom application telemetry in this node's private TDMA slot */
    void sendTelemetry(const uint8_t *data, uint8_t len);
    void sendTelemetry(const char *text);

    /* Send a downlink command to a specific Short-ID (0xFFF = cluster broadcast) */
    bool sendCommand(uint16_t target_short_id, const char *payload);

    /* Send a group command to all nodes matching dynamic Tag (e.g. "LivingRoom") */
    bool sendTagCommand(const char *tag, const char *payload);

    /* Execute any OpenNear gateway control command ("FENCE:...", "RULE:...", "ANCHOR:...", "LOCDEMO:1") */
    bool execControl(const char *control_text);

    /* Convenience helpers for v7.2 Offline 2D Geofences, Rules & Showcase Demo */
    bool setFence(const char *name, int32_t x_cm, int32_t y_cm, uint16_t radius_cm);
    bool setRule(uint8_t rule_id, on_loc_rule_trig_t trig, const char *zone,
                 uint8_t csi_threshold, const char *tag, const char *cmd_payload);
    bool startShowcaseDemo();
    bool stopShowcaseDemo();

    /* Inspect live node state safely (thread-safe across Core 0 / Core 1) */
    on_role_t role() const;
    uint16_t  shortId() const;
    uint32_t  superframeSeq() const;
    bool      isPrimaryGateway() const;

    /* Advanced escape hatch: access underlying C11 structs (acquire lock before mutating!) */
    on_node_t   *rawNode();
    on_gw_ctx_t *rawGateway();

private:
    bool                 started_;
    on_role_t            last_role_;
    OpenNearRoleCallback role_cb_;
};

extern OpenNearClass OpenNear;

#endif /* __cplusplus && ARDUINO */
#endif /* OPENNEAR_CPP_FACADE_H */
