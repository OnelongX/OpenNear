#include "esp32_net_gateway.h"
#include "hal_esp32_raw80211.h"
#include "esp32_ota_flash.h"
#include "esp32_demo_app.h"
#include "on_mqtt.h"
#include "on_locate.h"
#include <Arduino.h>
#include <string.h>

#if defined(OPENNEAR_ENABLE_WIFI_GW) || defined(OPENNEAR_ENABLE_BLE_GW)
#include <Preferences.h>
#endif

#ifdef OPENNEAR_ENABLE_WIFI_GW
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp32_tls_transport.h"
#include "esp32_web_portal.h"
#endif

#ifdef OPENNEAR_ENABLE_BLE_GW
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#endif

static on_gw_ctx_t      *s_gw_ref     = nullptr;
static on_node_t        *s_node_ref   = nullptr;
static SemaphoreHandle_t s_node_mutex = nullptr;

static void (*s_user_cmd_cb)(void *ctx, const uint8_t *payload, size_t len) = nullptr;
static void *s_user_cmd_ctx = nullptr;
static void (*s_user_rule_cb)(void *ctx, uint8_t rule_id, const char *zone,
                              const char *tag, const char *cmd) = nullptr;
static void *s_user_rule_ctx = nullptr;

void on_esp32_net_gw_set_user_cmd_cb(void (*cb)(void *ctx, const uint8_t *payload, size_t len),
                                     void *ctx) {
    s_user_cmd_cb  = cb;
    s_user_cmd_ctx = ctx;
}

void on_esp32_net_gw_set_user_rule_cb(void (*cb)(void *ctx, uint8_t rule_id, const char *zone,
                                                 const char *tag, const char *cmd),
                                      void *ctx) {
    s_user_rule_cb  = cb;
    s_user_rule_ctx = ctx;
}

static void ensure_node_mutex(void) {
    if (!s_node_mutex) {
        s_node_mutex = xSemaphoreCreateRecursiveMutex();
    }
}

void on_esp32_node_lock(void) {
    ensure_node_mutex();
    if (s_node_mutex) xSemaphoreTakeRecursive(s_node_mutex, portMAX_DELAY);
}

void on_esp32_node_unlock(void) {
    if (s_node_mutex) xSemaphoreGiveRecursive(s_node_mutex);
}

#if defined(OPENNEAR_ENABLE_WIFI_GW) || defined(OPENNEAR_ENABLE_BLE_GW)
static Preferences s_prefs;
static bool        s_netkey_custom = false;

bool on_esp32_net_gw_load_netkey(uint8_t out_key[16]) {
    if (!out_key) return false;
    if (s_prefs.begin("opennear_gw", true)) {
        size_t n = s_prefs.getBytesLength("netkey");
        if (n == 16u && s_prefs.getBytes("netkey", out_key, 16u) == 16u) {
            s_prefs.end();
            s_netkey_custom = true;
            return true;
        }
        s_prefs.end();
    }
    s_netkey_custom = false;
    return false;
}

void on_esp32_net_gw_factory_reset(void) {
    if (s_prefs.begin("opennear_gw", false)) {
        s_prefs.clear();
        s_prefs.end();
    }
    if (s_prefs.begin("opennear_alias", false)) {
        s_prefs.clear();
        s_prefs.end();
    }
    if (s_prefs.begin("opennear_tag", false)) {
        s_prefs.clear();
        s_prefs.end();
    }
    if (s_prefs.begin("opennear_anch", false)) {
        s_prefs.clear();
        s_prefs.end();
    }
    if (s_prefs.begin("opennear_fence", false)) {
        s_prefs.clear();
        s_prefs.end();
    }
    if (s_prefs.begin("opennear_rule", false)) {
        s_prefs.clear();
        s_prefs.end();
    }
    Serial.println("[OpenNear GW] Hardware factory reset (GPIO0 held 5s) -> NVS cleared, rebooting into OpenNear-Setup!");
    Serial.flush();
    delay(200);
    esp_restart();
}
#else
bool on_esp32_net_gw_load_netkey(uint8_t out_key[16]) {
    (void)out_key;
    return false;
}

void on_esp32_net_gw_factory_reset(void) {
    esp_restart();
}
#endif

#ifdef OPENNEAR_ENABLE_WIFI_GW
static WebServer   s_http_srv(80);
static DNSServer   s_dns_srv;
static Client     *s_active_mqtt_cli    = nullptr;
static bool        s_http_srv_started   = false;
static bool        s_wifi_was_connected = false;
static bool        s_ap_active          = false;
static bool        s_dns_active         = false;
static bool        s_mdns_active        = false;
static bool        s_mqtt_wait_connack  = false;
static bool        s_scan_evt_reg       = false;
static volatile bool s_scan_running     = false;
static volatile bool s_scan_done        = false;
static uint32_t    s_scan_started_ms    = 0u;
static char        s_ap_ssid[32]        = {0};
static char        s_mdns_host[32]      = {0};
static char        s_mqtt_client_id[32] = {0};
static char        s_mqtt_avty_topic[48]= {0};
static uint32_t    s_ap_off_at_ms       = 0u;
static uint32_t    s_sta_start_ms       = 0u;
static uint32_t    s_last_wifi_retry_ms = 0u;
static uint32_t    s_last_mqtt_retry_ms = 0u;
static uint32_t    s_mqtt_conn_start_ms = 0u;
static uint32_t    s_mqtt_backoff_ms    = 5000u;
static uint32_t    s_last_mqtt_ping_ms  = 0u;
static uint32_t    s_last_ha_pub_ms     = 0u;
static uint8_t     s_mqtt_rxbuf[512];
static size_t      s_mqtt_rxlen         = 0u;

/* v6.7 MAC-anchored node alias cache (persisted in NVS namespace "opennear_alias") */
#define ON_MAX_ALIASES 24u
typedef struct {
    bool    used;
    uint8_t mac48[6];
    char    name[24];
} on_alias_entry_t;
static on_alias_entry_t s_aliases[ON_MAX_ALIASES];

static void format_mac_key(const uint8_t mac[6], char out[13]) {
    snprintf(out, 13u, "%02X%02X%02X%02X%02X%02X",
             (unsigned)mac[0], (unsigned)mac[1], (unsigned)mac[2],
             (unsigned)mac[3], (unsigned)mac[4], (unsigned)mac[5]);
}

static const char *lookup_mac_alias(const uint8_t mac[6]) {
    if (!mac) return nullptr;
    if ((mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5]) == 0u) return nullptr;
    for (uint32_t i = 0u; i < ON_MAX_ALIASES; ++i) {
        if (s_aliases[i].used && memcmp(s_aliases[i].mac48, mac, 6u) == 0) {
            return s_aliases[i].name;
        }
    }
    /* Lazy-load from NVS on first lookup for unseen MAC */
    char key[13];
    format_mac_key(mac, key);
    if (s_prefs.begin("opennear_alias", true)) {
        String val = s_prefs.getString(key, "");
        s_prefs.end();
        if (val.length() > 0) {
            for (uint32_t i = 0u; i < ON_MAX_ALIASES; ++i) {
                if (!s_aliases[i].used) {
                    s_aliases[i].used = true;
                    memcpy(s_aliases[i].mac48, mac, 6u);
                    strncpy(s_aliases[i].name, val.c_str(), sizeof(s_aliases[i].name) - 1u);
                    s_aliases[i].name[sizeof(s_aliases[i].name) - 1u] = '\0';
                    return s_aliases[i].name;
                }
            }
        }
    }
    return nullptr;
}

static void save_mac_alias(const uint8_t mac[6], const char *name) {
    if (!mac) return;
    char key[13];
    format_mac_key(mac, key);
    bool clearing = (!name || name[0] == '\0');
    if (s_prefs.begin("opennear_alias", false)) {
        if (clearing) s_prefs.remove(key);
        else s_prefs.putString(key, name);
        s_prefs.end();
    }
    int free_idx = -1;
    for (uint32_t i = 0u; i < ON_MAX_ALIASES; ++i) {
        if (s_aliases[i].used && memcmp(s_aliases[i].mac48, mac, 6u) == 0) {
            if (clearing) {
                s_aliases[i].used = false;
                s_aliases[i].name[0] = '\0';
            } else {
                strncpy(s_aliases[i].name, name, sizeof(s_aliases[i].name) - 1u);
                s_aliases[i].name[sizeof(s_aliases[i].name) - 1u] = '\0';
            }
            return;
        }
        if (!s_aliases[i].used && free_idx < 0) free_idx = (int)i;
    }
    if (!clearing && free_idx >= 0) {
        s_aliases[free_idx].used = true;
        memcpy(s_aliases[free_idx].mac48, mac, 6u);
        strncpy(s_aliases[free_idx].name, name, sizeof(s_aliases[free_idx].name) - 1u);
        s_aliases[free_idx].name[sizeof(s_aliases[free_idx].name) - 1u] = '\0';
    }
}

/* v7.0 Dynamic MAC-anchored Node Tags (persisted in NVS "opennear_tag") & Indoor Positioning */
#define ON_MAX_TAG_ENTRIES 24u
typedef struct {
    bool    used;
    uint8_t mac48[6];
    char    tags[32];
} on_tag_entry_t;
static on_tag_entry_t s_node_tags[ON_MAX_TAG_ENTRIES];
static on_loc_ctx_t   s_loc;
static bool           s_loc_inited        = false;
static bool           s_loc_demo_running  = false;
static uint32_t       s_loc_demo_step     = 0u;
static uint32_t       s_last_demo_step_ms = 0u;
static uint32_t       s_last_rule_eval_ms = 0u;
static on_role_t      s_last_obs_role     = ON_ROLE_UNJOINED;
static uint16_t       s_last_synced_bk_id = ON_SHORT_ID_UNASSIGNED;

/* v7.2: Broadcast a provisioning/config command over AES-128-CCM air link to all Standby Gateways */
static void sync_prov_to_standby_gw(on_node_t *node, const char *cmd_str, size_t cmd_len) {
    if (!node || !cmd_str || cmd_len == 0u || cmd_len > ON_GW_MAX_PAYLOAD_LEN) return;
    on_esp32_node_lock();
    if (node->role == ON_ROLE_G && node->tdma_bitmap != 0u) {
        on_node_send_gw_downlink(node, ON_SHORT_ID_BROADCAST, ON_GW_MSG_WIFI_PROV,
                                 reinterpret_cast<const uint8_t *>(cmd_str),
                                 static_cast<uint8_t>(cmd_len));
    }
    on_esp32_node_unlock();
}

static const char *lookup_mac_tags(const uint8_t mac[6]) {
    if (!mac || (mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5]) == 0u) return nullptr;
    for (uint32_t i = 0u; i < ON_MAX_TAG_ENTRIES; ++i) {
        if (s_node_tags[i].used && memcmp(s_node_tags[i].mac48, mac, 6u) == 0) {
            return s_node_tags[i].tags;
        }
    }
    char key[13];
    format_mac_key(mac, key);
    if (s_prefs.begin("opennear_tag", true)) {
        String val = s_prefs.getString(key, "");
        s_prefs.end();
        if (val.length() > 0) {
            for (uint32_t i = 0u; i < ON_MAX_TAG_ENTRIES; ++i) {
                if (!s_node_tags[i].used) {
                    s_node_tags[i].used = true;
                    memcpy(s_node_tags[i].mac48, mac, 6u);
                    strncpy(s_node_tags[i].tags, val.c_str(), sizeof(s_node_tags[i].tags) - 1u);
                    s_node_tags[i].tags[sizeof(s_node_tags[i].tags) - 1u] = '\0';
                    return s_node_tags[i].tags;
                }
            }
        }
    }
    return nullptr;
}

static void save_mac_tags(const uint8_t mac[6], const char *tags) {
    if (!mac) return;
    char key[13];
    format_mac_key(mac, key);
    bool clearing = (!tags || tags[0] == '\0');
    if (s_prefs.begin("opennear_tag", false)) {
        if (clearing) s_prefs.remove(key);
        else s_prefs.putString(key, tags);
        s_prefs.end();
    }
    int free_idx = -1;
    for (uint32_t i = 0u; i < ON_MAX_TAG_ENTRIES; ++i) {
        if (s_node_tags[i].used && memcmp(s_node_tags[i].mac48, mac, 6u) == 0) {
            if (clearing) {
                s_node_tags[i].used = false;
                s_node_tags[i].tags[0] = '\0';
            } else {
                strncpy(s_node_tags[i].tags, tags, sizeof(s_node_tags[i].tags) - 1u);
                s_node_tags[i].tags[sizeof(s_node_tags[i].tags) - 1u] = '\0';
            }
            return;
        }
        if (!s_node_tags[i].used && free_idx < 0) free_idx = (int)i;
    }
    if (!clearing && free_idx >= 0) {
        s_node_tags[free_idx].used = true;
        memcpy(s_node_tags[free_idx].mac48, mac, 6u);
        strncpy(s_node_tags[free_idx].tags, tags, sizeof(s_node_tags[free_idx].tags) - 1u);
        s_node_tags[free_idx].tags[sizeof(s_node_tags[free_idx].tags) - 1u] = '\0';
    }
}

static bool mac_has_tag(const uint8_t mac[6], const char *needle) {
    if (!needle || needle[0] == '\0') return false;
    const char *csv = lookup_mac_tags(mac);
    if (!csv || csv[0] == '\0') return false;
    size_t nlen = strlen(needle);
    const char *p = csv;
    while (*p) {
        while (*p == ',' || *p == ' ') ++p;
        const char *start = p;
        while (*p && *p != ',') ++p;
        const char *end = p;
        while (end > start && end[-1] == ' ') --end;
        if ((size_t)(end - start) == nlen && memcmp(start, needle, nlen) == 0) {
            return true;
        }
    }
    return false;
}

static void save_anchors_to_nvs(void) {
    if (!s_prefs.begin("opennear_anch", false)) return;
    s_prefs.clear();
    uint32_t slot = 0u;
    for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
        if (!s_loc.anchors[i].active) continue;
        char k[8], v[64], mhex[13];
        format_mac_key(s_loc.anchors[i].mac48, mhex);
        snprintf(k, sizeof(k), "a%u", (unsigned)slot++);
        snprintf(v, sizeof(v), "%s|%s|%ld|%ld", mhex, s_loc.anchors[i].room,
                 (long)s_loc.anchors[i].x_cm, (long)s_loc.anchors[i].y_cm);
        s_prefs.putString(k, v);
    }
    s_prefs.end();
}

static void load_anchors_from_nvs(void) {
    if (!s_loc_inited) {
        on_loc_init(&s_loc);
        s_loc_inited = true;
    }
    if (!s_prefs.begin("opennear_anch", true)) return;
    for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
        char k[8];
        snprintf(k, sizeof(k), "a%u", (unsigned)i);
        String v = s_prefs.getString(k, "");
        if (v.length() < 14) continue;
        int b1 = v.indexOf('|');
        int b2 = (b1 > 0) ? v.indexOf('|', b1 + 1) : -1;
        int b3 = (b2 > 0) ? v.indexOf('|', b2 + 1) : -1;
        if (b1 != 12 || b2 <= b1 || b3 <= b2) continue;
        uint8_t mac[6];
        bool ok = true;
        for (int b = 0; b < 6; ++b) {
            char hs[3] = { v[b * 2], v[b * 2 + 1], '\0' };
            char *ep = nullptr;
            unsigned long val = strtoul(hs, &ep, 16);
            if (!ep || *ep != '\0') { ok = false; break; }
            mac[b] = static_cast<uint8_t>(val & 0xFFu);
        }
        if (!ok) continue;
        String room = v.substring(b1 + 1, b2);
        int32_t x_cm = static_cast<int32_t>(v.substring(b2 + 1, b3).toInt());
        int32_t y_cm = static_cast<int32_t>(v.substring(b3 + 1).toInt());
        on_loc_set_anchor(&s_loc, mac, ON_SHORT_ID_UNASSIGNED, room.c_str(), x_cm, y_cm);
    }
    s_prefs.end();
}

/* Invoked on Core 1 under on_esp32_node_lock() whenever G hears or receives an Anchor RSSI report */
static void esp32_on_rssi_observed_cb(void *ctx, uint16_t anchor_id,
                                      uint16_t target_id, int8_t rssi_dbm) {
    auto *node = static_cast<on_node_t *>(ctx);
    if (!node || !s_loc_inited) return;
    uint8_t amac[6] = {0}, tmac[6] = {0};
    if (!on_node_find_mac_by_short_id(node, anchor_id, amac)) return;
    if (!on_node_find_mac_by_short_id(node, target_id, tmac)) return;
    on_loc_observe_rssi(&s_loc, amac, tmac, target_id, rssi_dbm, millis());
}

/* v7.1: Invoked on Core 1 (from Anchor ON_GW_MSG_BLE_BEACON_REPORT) or Core 0 (local scan on G) */
static void esp32_on_ble_beacon_observed_cb(void *ctx, uint16_t anchor_id,
                                            const uint8_t tag_mac48[6], uint8_t source,
                                            int8_t tx_power_1m_dbm, int8_t rssi_dbm) {
    auto *node = static_cast<on_node_t *>(ctx);
    if (!node || !s_loc_inited || !tag_mac48) return;
    uint8_t amac[6] = {0};
    if (!on_node_find_mac_by_short_id(node, anchor_id, amac)) return;
    char auto_lbl[ON_LOC_LABEL_MAX_LEN] = {0};
    if (source == ON_LOC_SRC_IBEACON && tag_mac48[0] == 0x02u) {
        uint16_t maj = static_cast<uint16_t>((static_cast<uint16_t>(tag_mac48[2]) << 8) | tag_mac48[3]);
        uint16_t min = static_cast<uint16_t>((static_cast<uint16_t>(tag_mac48[4]) << 8) | tag_mac48[5]);
        snprintf(auto_lbl, sizeof(auto_lbl), "iB:%u/%u", (unsigned)maj, (unsigned)min);
    } else if (source == ON_LOC_SRC_EDDYSTONE && tag_mac48[0] == 0x06u) {
        snprintf(auto_lbl, sizeof(auto_lbl), "Ed:%02X%02X%02X%02X",
                 (unsigned)tag_mac48[2], (unsigned)tag_mac48[3],
                 (unsigned)tag_mac48[4], (unsigned)tag_mac48[5]);
    }
    on_loc_observe_beacon(&s_loc, amac, tag_mac48, static_cast<on_loc_source_t>(source),
                          tx_power_1m_dbm, rssi_dbm, auto_lbl[0] ? auto_lbl : nullptr, millis());
}

/* v7.2: Invoked on Core 1 when G hears or receives ON_GW_MSG_CSI_REPORT (0x0C) from an Anchor */
static void esp32_on_csi_observed_cb(void *ctx, uint16_t anchor_id,
                                     uint8_t csi_score, const uint8_t subcarrier_bins[16]) {
    auto *node = static_cast<on_node_t *>(ctx);
    if (!node || !s_loc_inited) return;
    uint8_t amac[6] = {0};
    (void)on_node_find_mac_by_short_id(node, anchor_id, amac);
    on_loc_observe_csi(&s_loc, amac, anchor_id, csi_score, subcarrier_bins, millis());
}

static void save_fences_to_nvs(void) {
    if (!s_prefs.begin("opennear_fence", false)) return;
    s_prefs.clear();
    uint32_t slot = 0u;
    for (uint32_t i = 0u; i < ON_LOC_MAX_GEOFENCES; ++i) {
        if (!s_loc.fences[i].active) continue;
        char k[8], v[64];
        snprintf(k, sizeof(k), "f%u", (unsigned)slot++);
        snprintf(v, sizeof(v), "%s|%ld|%ld|%u", s_loc.fences[i].name,
                 (long)s_loc.fences[i].x_cm, (long)s_loc.fences[i].y_cm,
                 (unsigned)s_loc.fences[i].radius_cm);
        s_prefs.putString(k, v);
    }
    s_prefs.end();
}

static void load_fences_from_nvs(void) {
    if (!s_prefs.begin("opennear_fence", true)) return;
    for (uint32_t i = 0u; i < ON_LOC_MAX_GEOFENCES; ++i) {
        char k[8];
        snprintf(k, sizeof(k), "f%u", (unsigned)i);
        String v = s_prefs.getString(k, "");
        if (v.length() < 5) continue;
        int b1 = v.indexOf('|');
        int b2 = (b1 > 0) ? v.indexOf('|', b1 + 1) : -1;
        int b3 = (b2 > 0) ? v.indexOf('|', b2 + 1) : -1;
        if (b1 <= 0 || b2 <= b1 || b3 <= b2) continue;
        String nm = v.substring(0, b1);
        int32_t x_cm = static_cast<int32_t>(v.substring(b1 + 1, b2).toInt());
        int32_t y_cm = static_cast<int32_t>(v.substring(b2 + 1, b3).toInt());
        uint16_t rad = static_cast<uint16_t>(v.substring(b3 + 1).toInt());
        if (rad > 0u) on_loc_set_geofence(&s_loc, nm.c_str(), x_cm, y_cm, rad);
    }
    s_prefs.end();
}

static void save_rules_to_nvs(void) {
    if (!s_prefs.begin("opennear_rule", false)) return;
    s_prefs.clear();
    for (uint32_t i = 0u; i < ON_LOC_MAX_RULES; ++i) {
        if (!s_loc.rules[i].active) continue;
        char k[8], v[96];
        snprintf(k, sizeof(k), "r%u", (unsigned)s_loc.rules[i].id);
        snprintf(v, sizeof(v), "%u|%u|%s|%u|%s|%s",
                 (unsigned)s_loc.rules[i].id,
                 (unsigned)s_loc.rules[i].trig_type,
                 s_loc.rules[i].zone,
                 (unsigned)s_loc.rules[i].csi_threshold,
                 s_loc.rules[i].cmd_tag,
                 s_loc.rules[i].cmd_payload);
        s_prefs.putString(k, v);
    }
    s_prefs.end();
}

static void load_rules_from_nvs(void) {
    if (!s_prefs.begin("opennear_rule", true)) return;
    for (uint32_t i = 1u; i <= ON_LOC_MAX_RULES; ++i) {
        char k[8];
        snprintf(k, sizeof(k), "r%u", (unsigned)i);
        String v = s_prefs.getString(k, "");
        if (v.length() < 9) continue;
        int b1 = v.indexOf('|');
        int b2 = (b1 > 0) ? v.indexOf('|', b1 + 1) : -1;
        int b3 = (b2 > 0) ? v.indexOf('|', b2 + 1) : -1;
        int b4 = (b3 > 0) ? v.indexOf('|', b3 + 1) : -1;
        int b5 = (b4 > 0) ? v.indexOf('|', b4 + 1) : -1;
        if (b1 <= 0 || b2 <= b1 || b3 <= b2 || b4 <= b3 || b5 <= b4) continue;
        uint8_t rid  = static_cast<uint8_t>(v.substring(0, b1).toInt());
        uint8_t trig = static_cast<uint8_t>(v.substring(b1 + 1, b2).toInt());
        String zone  = v.substring(b2 + 1, b3);
        uint8_t th   = static_cast<uint8_t>(v.substring(b3 + 1, b4).toInt());
        String tag   = v.substring(b4 + 1, b5);
        String cmd   = v.substring(b5 + 1);
        on_loc_set_rule(&s_loc, rid, trig, zone.c_str(), th, tag.c_str(), cmd.c_str());
    }
    s_prefs.end();
}

static uint32_t dispatch_tag_or_local_cmd(on_node_t *node, const char *tag,
                                          const uint8_t *pay, uint8_t plen) {
    if (!tag || !pay || plen == 0u) return 0u;
    bool all_tags = (strcmp(tag, "*") == 0);
    uint32_t sent_cnt = 0u;
    if (node) {
        on_cluster_snapshot_t snap;
        on_esp32_node_lock();
        on_node_get_cluster_snapshot(node, &snap);
        on_esp32_node_unlock();
        for (uint8_t i = 0u; i < snap.active_count; ++i) {
            const on_cluster_node_t *cn = &snap.nodes[i];
            if (!all_tags && !mac_has_tag(cn->mac48, tag)) continue;
            if (cn->short_id == node->short_id) {
                on_esp32_demo_on_cmd(pay, plen);
                sent_cnt++;
            } else {
                on_esp32_node_lock();
                if (node->mesh &&
                    on_mesh_lookup_next_hop(node->mesh, cn->short_id) != ON_SHORT_ID_UNASSIGNED) {
                    on_node_send_mesh_downlink(node, cn->short_id, pay, plen);
                } else {
                    on_node_send_gw_downlink(node, cn->short_id, ON_GW_MSG_CLOUD_CMD_DOWN, pay, plen);
                }
                on_esp32_node_unlock();
                sent_cnt++;
            }
        }
    }
    /* When running on a single standalone board, wildcard '*', or showcase demo mode without manual tags,
     * also trigger local demo LED/Relay AND broadcast over OpenNear so all demo boards actuate in sync! */
    if (sent_cnt == 0u) {
        on_esp32_demo_on_cmd(pay, plen);
        if (s_user_cmd_cb) s_user_cmd_cb(s_user_cmd_ctx, pay, plen);
        if (node && (all_tags || s_loc_demo_running)) {
            on_esp32_node_lock();
            on_node_send_gw_downlink(node, ON_SHORT_ID_BROADCAST, ON_GW_MSG_CLOUD_CMD_DOWN, pay, plen);
            on_esp32_node_unlock();
        }
        sent_cnt = 1u;
    }
    return sent_cnt;
}

static void esp32_on_rule_fired_cb(void *user_ctx, const on_loc_rule_t *rule) {
    auto *node = static_cast<on_node_t *>(user_ctx);
    if (!rule || !rule->cmd_payload[0]) return;
    size_t clen = strlen(rule->cmd_payload);
    if (clen == 0u || clen > ON_GW_MAX_PAYLOAD_LEN) return;
    const char *tag = rule->cmd_tag[0] ? rule->cmd_tag : "*";
    if (s_user_rule_cb) {
        s_user_rule_cb(s_user_rule_ctx, rule->id, rule->zone, tag, rule->cmd_payload);
    }
    uint32_t sent = dispatch_tag_or_local_cmd(node, tag,
                                              reinterpret_cast<const uint8_t *>(rule->cmd_payload),
                                              static_cast<uint8_t>(clen));
    Serial.printf("[OpenNear Rule] Fired R#%u [%s] -> tag='%s' cmd='%s' (dispatched=%u)\n",
                  (unsigned)rule->id, rule->zone, tag, rule->cmd_payload, (unsigned)sent);
}

static void ensure_gateway_mac_ids(const on_node_t *node) {
    if (s_mqtt_client_id[0] != '\0') return;
    uint8_t mac[6] = {0};
    if (node && (node->mac48[0] | node->mac48[1] | node->mac48[2] |
                 node->mac48[3] | node->mac48[4] | node->mac48[5]) != 0u) {
        memcpy(mac, node->mac48, 6u);
    } else {
        WiFi.macAddress(mac);
    }
    snprintf(s_mqtt_client_id, sizeof(s_mqtt_client_id), "opennear_gw_%02X%02X%02X",
             (unsigned)mac[3], (unsigned)mac[4], (unsigned)mac[5]);
    snprintf(s_mqtt_avty_topic, sizeof(s_mqtt_avty_topic), "opennear/gw/%02X%02X%02X/status",
             (unsigned)mac[3], (unsigned)mac[4], (unsigned)mac[5]);
}

static bool http_token_matches(const char *expected) {
    if (!expected || expected[0] == '\0') return true;
    if (!s_http_srv.hasHeader("X-OpenNear-Token")) return false;
    String got = s_http_srv.header("X-OpenNear-Token");
    size_t elen = strlen(expected);
    if (got.length() != elen) return false;
    unsigned char diff = 0u;
    const char *gp = got.c_str();
    for (size_t i = 0u; i < elen; ++i) {
        diff |= static_cast<unsigned char>(gp[i] ^ expected[i]);
    }
    return diff == 0u;
}

static void apply_wifi_coex_ps(void) {
#ifdef OPENNEAR_ENABLE_BLE_GW
    /* ESP-IDF SW Wi-Fi/BLE coexistence requires WIFI_PS_MIN_MODEM; WIFI_PS_NONE aborts! */
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
#else
    WiFi.setSleep(WIFI_PS_NONE);
#endif
}

static void append_json_escaped(char *dst, size_t cap, size_t *pos, const char *src) {
    if (!dst || !pos || !src || *pos >= cap) return;
    while (*src && *pos + 2u < cap) {
        unsigned char c = static_cast<unsigned char>(*src++);
        if (c == '"' || c == '\\') {
            if (*pos + 3u >= cap) break;
            dst[(*pos)++] = '\\';
            dst[(*pos)++] = static_cast<char>(c);
        } else if (c >= 0x20u) {
            dst[(*pos)++] = static_cast<char>(c);
        }
    }
    dst[*pos] = '\0';
}

static void append_cstr(char *dst, size_t cap, size_t *pos, const char *s) {
    if (!dst || !pos || !s || *pos >= cap) return;
    while (*s && *pos + 1u < cap) {
        dst[(*pos)++] = *s++;
    }
    dst[*pos] = '\0';
}

static on_err_t esp32_mqtt_publish_cb(void *ctx, const char *topic,
                                      const char *payload, bool retain) {
    (void)ctx;
    if (!s_active_mqtt_cli || !s_active_mqtt_cli->connected() || !topic || !payload) {
        return ON_ERR_NOT_READY;
    }
    static uint8_t pkt[640];
    size_t n = on_mqtt_encode_publish(topic, reinterpret_cast<const uint8_t *>(payload),
                                      strlen(payload), retain, pkt, sizeof(pkt));
    if (n == 0u) return ON_ERR_TOO_LONG;
    return (s_active_mqtt_cli->write(pkt, n) == n) ? ON_OK : ON_ERR_NOT_READY;
}

static void esp32_mqtt_poll(on_node_t *node, on_gw_ctx_t *gw) {
    if (!gw) return;
    if (gw->cfg.mqtt_creds_updated) {
        gw->cfg.mqtt_creds_updated = false;
        on_esp32_tls_stop_mqtt_clients();
        s_active_mqtt_cli   = nullptr;
        s_mqtt_wait_connack = false;
        s_mqtt_rxlen        = 0u;
        s_mqtt_backoff_ms   = 5000u;
        s_last_mqtt_retry_ms = 0u;
        if (s_prefs.begin("opennear_gw", false)) {
            s_prefs.putString("mhost", gw->cfg.mqtt_host);
            s_prefs.putUShort("mport", gw->cfg.mqtt_port);
            s_prefs.putString("muser", gw->cfg.mqtt_user);
            s_prefs.putString("mpass", gw->cfg.mqtt_pass);
            s_prefs.putBool("mtls", gw->cfg.mqtt_use_tls);
            s_prefs.end();
        }
        char pbuf[ON_GW_MAX_PAYLOAD_LEN + 1u];
        size_t mlen = on_gw_format_mqtt_prov(gw, pbuf, sizeof(pbuf));
        if (mlen > 0u) sync_prov_to_standby_gw(node, pbuf, mlen);
    }
    if (WiFi.status() != WL_CONNECTED || gw->cfg.mqtt_host[0] == '\0') {
        gw->mqtt_connected = false;
        return;
    }

    Client *cli = on_esp32_tls_get_mqtt_client(gw->cfg.mqtt_use_tls);
    if (!cli) {
        gw->mqtt_connected  = false;
        s_mqtt_wait_connack = false;
        s_active_mqtt_cli   = nullptr;
        return; /* MQTTS requested, but firmware built without OPENNEAR_ENABLE_TLS=1 */
    }
    s_active_mqtt_cli = cli;
    ensure_gateway_mac_ids(node);

    uint32_t now_ms = millis();
    if (!cli->connected()) {
        gw->mqtt_connected  = false;
        s_mqtt_wait_connack = false;
        s_mqtt_rxlen        = 0u;
        if (s_last_mqtt_retry_ms != 0u && (now_ms - s_last_mqtt_retry_ms) < s_mqtt_backoff_ms) return;
        s_last_mqtt_retry_ms = now_ms;
        uint16_t port = gw->cfg.mqtt_port ? gw->cfg.mqtt_port : (gw->cfg.mqtt_use_tls ? 8883u : 1883u);
        if (cli->connect(gw->cfg.mqtt_host, port)) {
            uint8_t cbuf[256];
            size_t clen = on_mqtt_encode_connect_lwt(s_mqtt_client_id, gw->cfg.mqtt_user,
                                                     gw->cfg.mqtt_pass, 30u,
                                                     s_mqtt_avty_topic, "offline", true,
                                                     cbuf, sizeof(cbuf));
            if (clen > 0u && cli->write(cbuf, clen) == clen) {
                s_mqtt_wait_connack  = true;
                s_mqtt_conn_start_ms = now_ms;
            } else {
                cli->stop();
                s_mqtt_backoff_ms = (s_mqtt_backoff_ms < 15000u) ? (s_mqtt_backoff_ms * 2u) : 30000u;
            }
        } else {
            /* Exponential backoff (5s -> 10s -> 20s -> 30s) so unreachable broker never stalls Web Portal */
            s_mqtt_backoff_ms = (s_mqtt_backoff_ms < 15000u) ? (s_mqtt_backoff_ms * 2u) : 30000u;
        }
        return;
    }

    if (s_mqtt_wait_connack && (now_ms - s_mqtt_conn_start_ms) > 5000u) {
        s_mqtt_wait_connack = false;
        cli->stop();
        s_mqtt_backoff_ms = (s_mqtt_backoff_ms < 15000u) ? (s_mqtt_backoff_ms * 2u) : 30000u;
        Serial.println("[OpenNear MQTT] CONNACK timeout -> retrying with backoff");
        return;
    }

    /* Keepalive PINGREQ every 20 s (only after CONNACK verified) */
    if (gw->mqtt_connected && (now_ms - s_last_mqtt_ping_ms) > 20000u) {
        s_last_mqtt_ping_ms = now_ms;
        uint8_t ping[2];
        on_mqtt_encode_pingreq(ping);
        cli->write(ping, 2u);
    }

    /* v7.2 Dual-Gateway HA: detect instant role promotion to Primary G -> trigger immediate HA publish */
    if (node) {
        on_esp32_node_lock();
        on_role_t cur_role = node->role;
        on_esp32_node_unlock();
        if (cur_role != s_last_obs_role) {
            if (cur_role == ON_ROLE_G && s_last_obs_role != ON_ROLE_UNJOINED) {
                s_last_ha_pub_ms = 0u;
                Serial.println("[OpenNear HA] Standby Gateway promoted to Primary G -> immediate MQTT/HA & Rule takeover!");
            }
            s_last_obs_role = cur_role;
        }
    }

    /* Periodically publish Home Assistant MQTT Auto-Discovery (MAC + Alias + LWT) every 15 s
     * (In Dual-Gateway mode, only the active Primary G publishes cluster-wide HA state;
     *  a standby Backup_ID gateway stays connected to Wi-Fi/MQTT and takes over in 150 ms on failover!) */
    if (gw->mqtt_connected && node && node->role == ON_ROLE_G &&
        (s_last_ha_pub_ms == 0u || (now_ms - s_last_ha_pub_ms) > 15000u)) {
        s_last_ha_pub_ms = now_ms;
        on_cluster_snapshot_t snap;
        on_esp32_node_lock();
        on_node_get_cluster_snapshot(node, &snap);
        on_esp32_node_unlock();
        static char topic[96];
        static char payload[600];
        for (uint8_t i = 0u; i < snap.active_count; ++i) {
            const on_cluster_node_t *cn = &snap.nodes[i];
            const char *rname = (cn->role == ON_ROLE_G) ? "G" :
                                (cn->role == ON_ROLE_SUB_G ? "SUB_G" : "T");
            const char *alias = lookup_mac_alias(cn->mac48);
            if (on_mqtt_format_ha_discovery_full(cn->short_id, cn->mac48, cn->hop_depth, rname,
                                                 alias, s_mqtt_avty_topic,
                                                 topic, sizeof(topic), payload, sizeof(payload))) {
                esp32_mqtt_publish_cb(gw, topic, payload, true);
            }
            if (on_mqtt_format_node_state_mac(cn->short_id, cn->mac48, cn->parent_id, cn->hop_depth,
                                              cn->last_skew_ns, cn->last_data, cn->last_data_len,
                                              topic, sizeof(topic), payload, sizeof(payload))) {
                esp32_mqtt_publish_cb(gw, topic, payload, false);
            }
            /* v7.0: Publish Home Assistant device_tracker room presence when indoor position solved */
            if (s_loc_inited && (cn->mac48[0] | cn->mac48[1] | cn->mac48[2] |
                                 cn->mac48[3] | cn->mac48[4] | cn->mac48[5]) != 0u) {
                on_loc_estimate_t est;
                on_esp32_node_lock();
                bool solved = on_loc_solve(&s_loc, cn->mac48, now_ms, &est);
                on_esp32_node_unlock();
                if (solved && est.room[0] != '\0') {
                    char mhex[13];
                    format_mac_key(cn->mac48, mhex);
                    snprintf(topic, sizeof(topic), "homeassistant/device_tracker/opennear_loc_%s/config", mhex);
                    snprintf(payload, sizeof(payload),
                             "{\"name\":\"%s Location\",\"uniq_id\":\"opennear_loc_%s\","
                             "\"stat_t\":\"opennear/loc/%s/room\",\"source_type\":\"router\","
                             "\"dev\":{\"ids\":[\"opennear_%s\"]}}",
                             alias ? alias : mhex, mhex, mhex, mhex);
                    esp32_mqtt_publish_cb(gw, topic, payload, true);
                    snprintf(topic, sizeof(topic), "opennear/loc/%s/room", mhex);
                    esp32_mqtt_publish_cb(gw, topic, est.room, false);
                }
            }
        }
        /* v7.1: Publish Home Assistant device_tracker + attributes for passive BLE iBeacon/Eddystone tags */
        if (s_loc_inited) {
            on_loc_estimate_t ble_ests[ON_LOC_MAX_TARGETS];
            uint32_t ble_cnt = 0u;
            on_esp32_node_lock();
            for (uint32_t bi = 0u; bi < ON_LOC_MAX_TARGETS; ++bi) {
                if (!s_loc.targets[bi].active || s_loc.targets[bi].source == ON_LOC_SRC_OPENNEAR) continue;
                on_loc_estimate_t e;
                if (on_loc_solve(&s_loc, s_loc.targets[bi].mac48, now_ms, &e) && e.room[0] != '\0') {
                    ble_ests[ble_cnt++] = e;
                }
            }
            on_esp32_node_unlock();
            for (uint32_t bi = 0u; bi < ble_cnt; ++bi) {
                const on_loc_estimate_t &est = ble_ests[bi];
                const char *alias = lookup_mac_alias(est.mac48);
                char mhex[13];
                format_mac_key(est.mac48, mhex);
                const char *disp = (alias && alias[0]) ? alias : (est.label[0] ? est.label : mhex);
                snprintf(topic, sizeof(topic), "homeassistant/device_tracker/opennear_loc_%s/config", mhex);
                snprintf(payload, sizeof(payload),
                         "{\"name\":\"%s\",\"uniq_id\":\"opennear_loc_%s\","
                         "\"stat_t\":\"opennear/loc/%s/room\",\"json_attr_t\":\"opennear/loc/%s/attrs\","
                         "\"source_type\":\"bluetooth_le\",\"dev\":{\"ids\":[\"opennear_ble_%s\"],\"name\":\"%s\"}}",
                         disp, mhex, mhex, mhex, mhex, disp);
                esp32_mqtt_publish_cb(gw, topic, payload, true);
                snprintf(topic, sizeof(topic), "opennear/loc/%s/room", mhex);
                esp32_mqtt_publish_cb(gw, topic, est.room, false);
                snprintf(topic, sizeof(topic), "opennear/loc/%s/attrs", mhex);
                snprintf(payload, sizeof(payload),
                         "{\"x_cm\":%ld,\"y_cm\":%ld,\"rssi\":%d,\"tx1m\":%d,\"anchors\":%u,\"err_cm\":%u,\"label\":\"%s\"}",
                         (long)est.x_cm, (long)est.y_cm, (int)est.best_rssi_dbm, (int)est.tx_power_1m_dbm,
                         (unsigned)est.anchor_count, (unsigned)est.est_err_cm, est.label);
                esp32_mqtt_publish_cb(gw, topic, payload, false);
            }
        }
    }

    /* Drain inbound MQTT TCP/TLS stream cleanly across coalesced CONNACK/SUBACK/PINGRESP/PUBLISH packets */
    while (cli->available() > 0 && s_mqtt_rxlen < sizeof(s_mqtt_rxbuf)) {
        int avail = cli->available();
        size_t space = sizeof(s_mqtt_rxbuf) - s_mqtt_rxlen;
        size_t to_read = (static_cast<size_t>(avail) < space) ? static_cast<size_t>(avail) : space;
        int n = cli->read(s_mqtt_rxbuf + s_mqtt_rxlen, to_read);
        if (n <= 0) break;
        s_mqtt_rxlen += static_cast<size_t>(n);
    }

    size_t consumed = 0u;
    while (consumed < s_mqtt_rxlen) {
        uint8_t pkt_type = 0u;
        size_t  pkt_len  = 0u;
        int st = on_mqtt_next_packet(s_mqtt_rxbuf + consumed, s_mqtt_rxlen - consumed,
                                     &pkt_type, &pkt_len);
        if (st == 0) break; /* Wait for remaining TCP bytes */
        if (st < 0 || pkt_len == 0u) {
            /* Malformed header: discard stream buffer to resync */
            consumed = s_mqtt_rxlen;
            break;
        }
        if (pkt_type == ON_MQTT_PKT_CONNACK && s_mqtt_wait_connack) {
            uint8_t rc = 0xFFu;
            if (on_mqtt_decode_connack(s_mqtt_rxbuf + consumed, pkt_len, &rc) && rc == 0u) {
                s_mqtt_wait_connack = false;
                gw->mqtt_connected  = true;
                s_mqtt_backoff_ms   = 5000u;
                s_last_mqtt_ping_ms = now_ms;
                s_last_ha_pub_ms    = 0u;
                uint8_t sbuf[64];
                size_t s1 = on_mqtt_encode_subscribe(1u, "opennear/cmd/#", sbuf, sizeof(sbuf));
                if (s1 > 0u) cli->write(sbuf, s1);
                size_t s2 = on_mqtt_encode_subscribe(2u, "opennear/ota/trigger", sbuf, sizeof(sbuf));
                if (s2 > 0u) cli->write(sbuf, s2);
                esp32_mqtt_publish_cb(gw, s_mqtt_avty_topic, "online", true);
                uint16_t port = gw->cfg.mqtt_port ? gw->cfg.mqtt_port : (gw->cfg.mqtt_use_tls ? 8883u : 1883u);
                Serial.printf("[OpenNear MQTT] Connected to %s://%s:%u (client=%s avty=%s)\n",
                              gw->cfg.mqtt_use_tls ? "mqtts" : "mqtt",
                              gw->cfg.mqtt_host, (unsigned)port,
                              s_mqtt_client_id, s_mqtt_avty_topic);
            } else {
                s_mqtt_wait_connack = false;
                gw->mqtt_connected  = false;
                cli->stop();
                s_mqtt_backoff_ms = (s_mqtt_backoff_ms < 15000u) ? (s_mqtt_backoff_ms * 2u) : 30000u;
                Serial.printf("[OpenNear MQTT] Broker rejected CONNACK rc=%u (check credentials)\n", (unsigned)rc);
                s_mqtt_rxlen = 0u;
                return;
            }
        } else if (pkt_type == ON_MQTT_PKT_PUBLISH) {
            on_mqtt_pub_view_t pub;
            if (on_mqtt_decode_publish(s_mqtt_rxbuf + consumed, pkt_len, &pub)) {
                uint16_t target_id = 0u;
                uint8_t target_mac[6] = {0};
                bool has_mac = false;
                bool is_ota = false;
                if (on_mqtt_parse_cmd_topic_ex(pub.topic, pub.topic_len, &target_id,
                                               target_mac, &has_mac, &is_ota)) {
                    if (is_ota) {
                        char cmd_txt[256];
                        int cl = 0;
                        const char *pay_s = reinterpret_cast<const char *>(pub.payload);
                        const char *bar = (pub.payload_len > 6u && memcmp(pay_s, "AUTH:", 5u) == 0)
                                          ? static_cast<const char *>(memchr(pay_s + 5, '|', pub.payload_len - 5u))
                                          : nullptr;
                        if (bar) {
                            size_t auth_prefix_len = static_cast<size_t>((bar - pay_s) + 1);
                            size_t rest_len = pub.payload_len - auth_prefix_len;
                            cl = snprintf(cmd_txt, sizeof(cmd_txt), "%.*sOTA:%.*s",
                                          static_cast<int>(auth_prefix_len), pay_s,
                                          static_cast<int>(rest_len), bar + 1);
                        } else {
                            cl = snprintf(cmd_txt, sizeof(cmd_txt), "OTA:%.*s",
                                          static_cast<int>(pub.payload_len), pay_s);
                        }
                        if (cl > 4 && static_cast<size_t>(cl) < sizeof(cmd_txt)) {
                            on_gw_handle_control_text(gw, cmd_txt, static_cast<size_t>(cl), false);
                        }
                    } else if (node) {
                        uint8_t plen = (pub.payload_len <= ON_GW_MAX_PAYLOAD_LEN)
                                       ? static_cast<uint8_t>(pub.payload_len) : ON_GW_MAX_PAYLOAD_LEN;
                        on_esp32_node_lock();
                        if (has_mac) {
                            target_id = on_node_find_short_id_by_mac(node, target_mac);
                        }
                        if (target_id != ON_SHORT_ID_UNASSIGNED) {
                            if (node->mesh &&
                                on_mesh_lookup_next_hop(node->mesh, target_id) != ON_SHORT_ID_UNASSIGNED) {
                                on_node_send_mesh_downlink(node, target_id, pub.payload, plen);
                            } else {
                                on_node_send_gw_downlink(node, target_id, ON_GW_MSG_CLOUD_CMD_DOWN,
                                                         pub.payload, plen);
                            }
                        }
                        on_esp32_node_unlock();
                    }
                }
            }
        }
        consumed += pkt_len;
    }
    if (consumed > 0u) {
        if (consumed < s_mqtt_rxlen) {
            memmove(s_mqtt_rxbuf, s_mqtt_rxbuf + consumed, s_mqtt_rxlen - consumed);
            s_mqtt_rxlen -= consumed;
        } else {
            s_mqtt_rxlen = 0u;
        }
    }
}

static void start_local_http_api(void) {
    if (s_http_srv_started) return;

    s_http_srv.on("/", HTTP_GET, []() {
        s_http_srv.send_P(200, "text/html; charset=utf-8", kOpenNearPortalHtml);
    });

    s_http_srv.on("/api/status", HTTP_GET, []() {
        static char json[1280];
        size_t pos = 0u;
        json[0] = '\0';

        const char *role_s = "UNJOINED";
        uint16_t short_id  = 0u;
        uint32_t sf_seq    = 0u;
        uint8_t  sta_mac[6] = {0};
        uint8_t  ble_mac[6] = {0};
        if (s_node_ref) {
            on_esp32_node_lock();
            switch (s_node_ref->role) {
                case ON_ROLE_G:     role_s = "G"; break;
                case ON_ROLE_SUB_G: role_s = "SUB_G"; break;
                case ON_ROLE_T:     role_s = "T"; break;
                default:            role_s = "UNJOINED"; break;
            }
            short_id = s_node_ref->short_id;
            sf_seq   = s_node_ref->current_sf_seq;
            memcpy(sta_mac, s_node_ref->mac48, 6u);
            on_esp32_node_unlock();
        }
        esp_read_mac(ble_mac, ESP_MAC_BT);
        char mac_s[18], bmac_s[18];
        snprintf(mac_s, sizeof(mac_s), "%02X:%02X:%02X:%02X:%02X:%02X",
                 sta_mac[0], sta_mac[1], sta_mac[2], sta_mac[3], sta_mac[4], sta_mac[5]);
        snprintf(bmac_s, sizeof(bmac_s), "%02X:%02X:%02X:%02X:%02X:%02X",
                 ble_mac[0], ble_mac[1], ble_mac[2], ble_mac[3], ble_mac[4], ble_mac[5]);

        bool wifi_ok = (WiFi.status() == WL_CONNECTED);
        IPAddress ip = wifi_ok ? WiFi.localIP() : IPAddress(0, 0, 0, 0);
        char ip_s[24];
        snprintf(ip_s, sizeof(ip_s), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
        uint32_t tx_ok = 0u, tx_fail = 0u;
        on_esp32_hal_tx_counts(&tx_ok, &tx_fail);
        char num[192];

        append_cstr(json, sizeof(json), &pos, "{\"role\":\"");
        append_cstr(json, sizeof(json), &pos, role_s);
        snprintf(num, sizeof(num), "\",\"id\":\"0x%03X\",\"mac\":\"%s\",\"ble_mac\":\"%s\",\"sf\":%u,\"fw_ver\":\"v7.2\",\"uptime_s\":%u",
                 (unsigned)short_id, mac_s, bmac_s, (unsigned)sf_seq, (unsigned)(millis() / 1000u));
        append_cstr(json, sizeof(json), &pos, num);

        snprintf(num, sizeof(num), ",\"free_heap\":%u,\"min_heap\":%u,\"tx_ok\":%u,\"tx_fail\":%u,\"rx_drops\":%u,\"mdns\":\"",
                 (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
                 (unsigned)tx_ok, (unsigned)tx_fail, (unsigned)on_esp32_hal_rx_drops());
        append_cstr(json, sizeof(json), &pos, num);
        append_json_escaped(json, sizeof(json), &pos, s_mdns_active ? s_mdns_host : "");

        snprintf(num, sizeof(num), "\",\"wifi_ok\":%s,\"wifi_ch\":%u,\"wifi_rssi\":%d,\"wifi_ip\":\"%s\",\"wifi_ssid\":\"",
                 wifi_ok ? "true" : "false",
                 (unsigned)(wifi_ok ? WiFi.channel() : 0),
                 (int)(wifi_ok ? WiFi.RSSI() : 0),
                 ip_s);
        append_cstr(json, sizeof(json), &pos, num);
        append_json_escaped(json, sizeof(json), &pos, s_gw_ref ? s_gw_ref->cfg.wifi_ssid : "");

        append_cstr(json, sizeof(json), &pos, "\",\"cloud_url\":\"");
        append_json_escaped(json, sizeof(json), &pos, s_gw_ref ? s_gw_ref->cfg.cloud_url : "");

        snprintf(num, sizeof(num), "\",\"ap_active\":%s,\"ap_ssid\":\"",
                 s_ap_active ? "true" : "false");
        append_cstr(json, sizeof(json), &pos, num);
        append_json_escaped(json, sizeof(json), &pos, s_ap_ssid);

        snprintf(num, sizeof(num), "\",\"mqtt_ok\":%s,\"mqtt_port\":%u,\"mqtt_host\":\"",
                 (s_gw_ref && s_gw_ref->mqtt_connected) ? "true" : "false",
                 (unsigned)((s_gw_ref && s_gw_ref->cfg.mqtt_port) ? s_gw_ref->cfg.mqtt_port : 1883u));
        append_cstr(json, sizeof(json), &pos, num);
        append_json_escaped(json, sizeof(json), &pos, s_gw_ref ? s_gw_ref->cfg.mqtt_host : "");

        ensure_gateway_mac_ids(s_node_ref);
        snprintf(num, sizeof(num), "\",\"ble_ok\":%s,\"ble_scan\":%s,\"loc_demo\":%s,\"token_set\":%s,\"netkey_custom\":%s,\"tls_compiled\":%s,\"mqtt_tls\":%s,\"tls_ca_set\":%s,\"mqtt_avty\":\"",
                 (s_gw_ref && s_gw_ref->ble_connected) ? "true" : "false",
                 (s_gw_ref && s_gw_ref->cfg.ble_scan_enabled) ? "true" : "false",
                 s_loc_demo_running ? "true" : "false",
                 (s_gw_ref && s_gw_ref->cfg.admin_token[0] != '\0') ? "true" : "false",
                 s_netkey_custom ? "true" : "false",
                 on_esp32_tls_is_compiled() ? "true" : "false",
                 (s_gw_ref && s_gw_ref->cfg.mqtt_use_tls) ? "true" : "false",
                 on_esp32_tls_has_ca() ? "true" : "false");
        append_cstr(json, sizeof(json), &pos, num);
        append_json_escaped(json, sizeof(json), &pos, s_mqtt_avty_topic);
        append_cstr(json, sizeof(json), &pos, "\"}");

        s_http_srv.send(200, "application/json", json);
    });

    s_http_srv.on("/api/scan", HTTP_GET, []() {
        static char json[1536];
        static wifi_ap_record_t recs[20];
        if (!s_scan_evt_reg) {
            WiFi.onEvent([](WiFiEvent_t event) {
                if (event == ARDUINO_EVENT_WIFI_SCAN_DONE) {
                    s_scan_running = false;
                    s_scan_done    = true;
                }
            });
            s_scan_evt_reg = true;
        }
        uint32_t now_ms = millis();
        if (s_scan_running && !s_scan_done && (now_ms - s_scan_started_ms) < 8000u) {
            s_http_srv.send(200, "application/json", "{\"scanning\":true,\"aps\":[]}");
            return;
        }
        if (!s_scan_running && !s_scan_done) {
            /* Non-blocking async scan using ESP-IDF BLE-safe default active timing (min=0, max=0) */
            WiFi.enableSTA(true);
            wifi_scan_config_t scfg;
            memset(&scfg, 0, sizeof(scfg));
            scfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
            if (esp_wifi_scan_start(&scfg, false) == ESP_OK) {
                s_scan_running    = true;
                s_scan_done       = false;
                s_scan_started_ms = now_ms;
                s_http_srv.send(200, "application/json", "{\"scanning\":true,\"aps\":[]}");
                return;
            }
            s_http_srv.send(200, "application/json", "{\"scanning\":false,\"aps\":[]}");
            return;
        }

        s_scan_running = false;
        s_scan_done    = false;
        size_t pos = 0u;
        json[0] = '\0';
        append_cstr(json, sizeof(json), &pos, "{\"scanning\":false,\"aps\":[");
        uint16_t count = static_cast<uint16_t>(sizeof(recs) / sizeof(recs[0]));
        if (esp_wifi_scan_get_ap_records(&count, recs) == ESP_OK) {
            int emitted = 0;
            for (uint16_t i = 0u; i < count && emitted < 15; ++i) {
                char ssid_buf[33];
                memcpy(ssid_buf, recs[i].ssid, 32u);
                ssid_buf[32] = '\0';
                if (ssid_buf[0] == '\0') continue;
                bool dup = false;
                for (uint16_t j = 0u; j < i; ++j) {
                    if (strncmp(reinterpret_cast<const char *>(recs[j].ssid), ssid_buf, 32u) == 0) {
                        dup = true;
                        break;
                    }
                }
                if (dup) continue;
                if (pos + 96u >= sizeof(json)) break;
                if (emitted > 0) append_cstr(json, sizeof(json), &pos, ",");
                append_cstr(json, sizeof(json), &pos, "{\"ssid\":\"");
                append_json_escaped(json, sizeof(json), &pos, ssid_buf);
                char tail[64];
                snprintf(tail, sizeof(tail), "\",\"rssi\":%d,\"ch\":%u,\"enc\":%d}",
                         (int)recs[i].rssi, (unsigned)recs[i].primary,
                         (recs[i].authmode == WIFI_AUTH_OPEN) ? 0 : 1);
                append_cstr(json, sizeof(json), &pos, tail);
                emitted++;
            }
        }
        append_cstr(json, sizeof(json), &pos, "]}");
        s_http_srv.send(200, "application/json", json);
    });

    s_http_srv.on("/api/aliases", HTTP_GET, []() {
        static char json[1024];
        size_t pos = 0u;
        json[0] = '\0';
        append_cstr(json, sizeof(json), &pos, "{");
        if (s_node_ref) {
            on_cluster_snapshot_t snap;
            on_esp32_node_lock();
            on_node_get_cluster_snapshot(s_node_ref, &snap);
            if (s_loc_inited) {
                for (uint32_t i = 0u; i < ON_LOC_MAX_TARGETS; ++i) {
                    if (s_loc.targets[i].active) lookup_mac_alias(s_loc.targets[i].mac48);
                }
            }
            on_esp32_node_unlock();
            for (uint8_t i = 0u; i < snap.active_count; ++i) {
                lookup_mac_alias(snap.nodes[i].mac48);
            }
        }
        bool first = true;
        for (uint32_t i = 0u; i < ON_MAX_ALIASES; ++i) {
            if (!s_aliases[i].used || s_aliases[i].name[0] == '\0') continue;
            if (pos + 64u >= sizeof(json)) break;
            if (!first) append_cstr(json, sizeof(json), &pos, ",");
            first = false;
            char mcol[28];
            snprintf(mcol, sizeof(mcol), "\"%02X:%02X:%02X:%02X:%02X:%02X\":\"",
                     (unsigned)s_aliases[i].mac48[0], (unsigned)s_aliases[i].mac48[1],
                     (unsigned)s_aliases[i].mac48[2], (unsigned)s_aliases[i].mac48[3],
                     (unsigned)s_aliases[i].mac48[4], (unsigned)s_aliases[i].mac48[5]);
            append_cstr(json, sizeof(json), &pos, mcol);
            append_json_escaped(json, sizeof(json), &pos, s_aliases[i].name);
            append_cstr(json, sizeof(json), &pos, "\"");
        }
        append_cstr(json, sizeof(json), &pos, "}");
        s_http_srv.send(200, "application/json", json);
    });

    s_http_srv.on("/api/tags", HTTP_GET, []() {
        static char json[1024];
        size_t pos = 0u;
        json[0] = '\0';
        append_cstr(json, sizeof(json), &pos, "{");
        if (s_node_ref) {
            on_cluster_snapshot_t snap;
            on_esp32_node_lock();
            on_node_get_cluster_snapshot(s_node_ref, &snap);
            if (s_loc_inited) {
                for (uint32_t i = 0u; i < ON_LOC_MAX_TARGETS; ++i) {
                    if (s_loc.targets[i].active) lookup_mac_tags(s_loc.targets[i].mac48);
                }
            }
            on_esp32_node_unlock();
            for (uint8_t i = 0u; i < snap.active_count; ++i) {
                lookup_mac_tags(snap.nodes[i].mac48);
            }
        }
        bool first = true;
        for (uint32_t i = 0u; i < ON_MAX_TAG_ENTRIES; ++i) {
            if (!s_node_tags[i].used || s_node_tags[i].tags[0] == '\0') continue;
            if (pos + 64u >= sizeof(json)) break;
            if (!first) append_cstr(json, sizeof(json), &pos, ",");
            first = false;
            char mcol[28];
            snprintf(mcol, sizeof(mcol), "\"%02X:%02X:%02X:%02X:%02X:%02X\":\"",
                     (unsigned)s_node_tags[i].mac48[0], (unsigned)s_node_tags[i].mac48[1],
                     (unsigned)s_node_tags[i].mac48[2], (unsigned)s_node_tags[i].mac48[3],
                     (unsigned)s_node_tags[i].mac48[4], (unsigned)s_node_tags[i].mac48[5]);
            append_cstr(json, sizeof(json), &pos, mcol);
            append_json_escaped(json, sizeof(json), &pos, s_node_tags[i].tags);
            append_cstr(json, sizeof(json), &pos, "\"");
        }
        append_cstr(json, sizeof(json), &pos, "}");
        s_http_srv.send(200, "application/json", json);
    });

    s_http_srv.on("/api/locate", HTTP_GET, []() {
        static char json[3072];
        if (!s_loc_inited) {
            s_http_srv.send(200, "application/json", "{\"anchors\":[],\"targets\":[],\"fences\":[],\"rules\":[]}");
            return;
        }
        on_esp32_node_lock();
        if (s_node_ref) {
            for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
                if (!s_loc.anchors[i].active) continue;
                uint16_t sid = on_node_find_short_id_by_mac(s_node_ref, s_loc.anchors[i].mac48);
                if (sid != ON_SHORT_ID_UNASSIGNED) s_loc.anchors[i].short_id = sid;
            }
        }
        size_t n = on_loc_format_json(&s_loc, millis(), json, sizeof(json));
        on_esp32_node_unlock();
        s_http_srv.send(200, "application/json", (n > 0u) ? json : "{\"anchors\":[],\"targets\":[]}");
    });

    s_http_srv.on("/api/topology", HTTP_GET, []() {
        static char json[3072];
        if (!s_node_ref) { s_http_srv.send(503, "application/json", "{\"error\":\"no_node\"}"); return; }
        on_esp32_node_lock();
        size_t n = on_node_format_cluster_json(s_node_ref, false, json, sizeof(json));
        on_esp32_node_unlock();
        s_http_srv.send(200, "application/json", (n > 0u) ? json : "{}");
    });
    s_http_srv.on("/api/data", HTTP_GET, []() {
        static char json[3072];
        if (!s_node_ref) { s_http_srv.send(503, "application/json", "{\"error\":\"no_node\"}"); return; }
        on_esp32_node_lock();
        size_t n = on_node_format_cluster_json(s_node_ref, true, json, sizeof(json));
        on_esp32_node_unlock();
        s_http_srv.send(200, "application/json", (n > 0u) ? json : "{}");
    });
    s_http_srv.on("/api/cmd", HTTP_POST, []() {
        if (!s_gw_ref || !s_http_srv.hasArg("plain")) {
            s_http_srv.send(400, "application/json", "{\"ok\":false}");
            return;
        }
        String body = s_http_srv.arg("plain");
        /* Support X-OpenNear-Token header (constant-time verify) when admin_token is configured */
        if (s_gw_ref->cfg.admin_token[0] != '\0' && !body.startsWith("AUTH:")) {
            if (!http_token_matches(s_gw_ref->cfg.admin_token)) {
                s_http_srv.send(401, "application/json", "{\"ok\":false,\"error\":\"unauthorized\"}");
                return;
            }
            body = String("AUTH:") + String(s_gw_ref->cfg.admin_token) + "|" + body;
        }
        on_err_t rc = on_gw_handle_control_text(s_gw_ref, body.c_str(), body.length(), false);
        /* Also mirror local broadcast/self CMD payloads to gateway's own demo app */
        if (rc == ON_OK && s_node_ref) {
            int cpos = body.indexOf("CMD:");
            if (cpos >= 0) {
                int bar = body.indexOf('|', cpos + 4);
                if (bar > cpos + 4) {
                    String id_s = body.substring(cpos + 4, bar);
                    uint16_t tid = static_cast<uint16_t>(strtoul(id_s.c_str(), nullptr, 16) & ON_SHORT_ID_MASK);
                    if (tid == ON_SHORT_ID_BROADCAST || tid == s_node_ref->short_id) {
                        String pay = body.substring(bar + 1);
                        on_esp32_demo_on_cmd(reinterpret_cast<const uint8_t *>(pay.c_str()), pay.length());
                    }
                }
            }
        }
        s_http_srv.send((rc == ON_OK) ? 200 : 400, "application/json",
                        (rc == ON_OK) ? "{\"ok\":true}" : "{\"ok\":false}");
    });
    s_http_srv.on("/api/tls_ca", HTTP_POST, []() {
        if (!s_gw_ref) { s_http_srv.send(503, "application/json", "{\"ok\":false}"); return; }
        if (s_gw_ref->cfg.admin_token[0] != '\0') {
            if (!http_token_matches(s_gw_ref->cfg.admin_token)) {
                s_http_srv.send(401, "application/json", "{\"ok\":false,\"error\":\"unauthorized\"}");
                return;
            }
        }
        if (!on_esp32_tls_is_compiled()) {
            s_http_srv.send(501, "application/json", "{\"ok\":false,\"error\":\"tls_not_compiled\"}");
            return;
        }
        String pem = s_http_srv.hasArg("plain") ? s_http_srv.arg("plain") : String("");
        pem.trim();
        on_err_t rc = on_esp32_tls_set_ca_pem(pem.c_str(), pem.length());
        s_http_srv.send((rc == ON_OK) ? 200 : 400, "application/json",
                        (rc == ON_OK) ? "{\"ok\":true}" : "{\"ok\":false}");
    });
    s_http_srv.onNotFound([]() {
        if (s_ap_active) {
            s_http_srv.sendHeader("Location", "http://192.168.4.1/", true);
            s_http_srv.send(302, "text/plain", "");
        } else {
            s_http_srv.send(404, "text/plain", "Not Found");
        }
    });
    const char *hdr_keys[1] = { "X-OpenNear-Token" };
    s_http_srv.collectHeaders(hdr_keys, 1);
    s_http_srv.begin();
    s_http_srv_started = true;
    Serial.println("[OpenNear GW] Embedded Web Portal & HTTP API started: /, /api/status, /api/scan, /api/topology, /api/data, /api/cmd");
}

static void start_softap_portal(void) {
    if (s_ap_active) return;
    /* Initialize Wi-Fi driver BEFORE reading factory MAC so SSID suffix is never 0000 */
    WiFi.mode(WIFI_AP_STA);
    apply_wifi_coex_ps();
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "OpenNear-Setup-%02X%02X",
             (unsigned)mac[4], (unsigned)mac[5]);
    WiFi.softAP(s_ap_ssid, nullptr, ON_ESP32_DEFAULT_CHANNEL);
    s_ap_active    = true;
    s_ap_off_at_ms = 0u;
    if (!s_dns_active) {
        s_dns_srv.start(53, "*", WiFi.softAPIP());
        s_dns_active = true;
    }
    start_local_http_api();
    Serial.printf("[OpenNear Portal] SoftAP '%s' active (Captive Portal DNS) -> http://192.168.4.1\n", s_ap_ssid);
}

static void save_wifi_nvs(const char *ssid, const char *pass, const char *url) {
    if (s_prefs.begin("opennear_gw", false)) {
        if (ssid && ssid[0]) s_prefs.putString("ssid", ssid);
        if (pass)            s_prefs.putString("pass", pass);
        if (url && url[0])   s_prefs.putString("url", url);
        s_prefs.end();
    }
}

static on_err_t esp32_wifi_connect_cb(void *ctx, const char *ssid, const char *pass) {
    (void)ctx;
    (void)ssid;
    (void)pass;
    if (!ssid || !ssid[0]) return ON_ERR_INVALID_ARG;
    /* Actual NVS save, WiFi.begin() & Standby Gateway air-sync run on Core 0 in on_esp32_net_gw_poll()
     * via gw->cfg.wifi_creds_updated so Core 1 real-time RX (ON_GW_MSG_WIFI_PROV) never blocks! */
    return ON_OK;
}

static on_err_t esp32_cloud_post_cb(void *ctx, const char *url, const char *json_body) {
    (void)ctx;
    if (!url || !url[0] || !json_body || WiFi.status() != WL_CONNECTED) {
        return ON_ERR_NOT_READY;
    }
    HTTPClient http;
    http.setConnectTimeout(1500);
    http.setTimeout(2000);
    if (!on_esp32_tls_http_begin(http, url)) return ON_ERR_NOT_READY;
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(json_body)),
                         strlen(json_body));
    http.end();
    return (code >= 200 && code < 300) ? ON_OK : ON_ERR_NOT_READY;
}

static on_err_t esp32_channel_align_cb(void *ctx, uint8_t wifi_ch) {
    (void)ctx;
    on_esp32_hal_lock_wifi_channel(true, wifi_ch);
    Serial.printf("[OpenNear GW] Coexistence channel aligned to public Wi-Fi AP ch=%u\n",
                  (unsigned)wifi_ch);
    return ON_OK;
}

/* Download firmware binary from public HTTP URL into inactive OTA partition and start cluster OTA */
static void esp32_pull_http_ota(on_node_t *node, on_gw_ctx_t *gw) {
    if (!node || !gw || !gw->cfg.ota_url[0] || WiFi.status() != WL_CONNECTED) return;

    Serial.printf("[OpenNear OTA] Pulling firmware from %s (target=0x%03X ver=0x%08X)...\n",
                  gw->cfg.ota_url, (unsigned)gw->cfg.ota_target_id, (unsigned)gw->cfg.ota_target_ver);

    /* Free ~35 KB TLS heap before an HTTPS OTA download on ESP32-WROOM-32 (MQTT auto-reconnects after) */
    if (strncmp(gw->cfg.ota_url, "https://", 8u) == 0 && gw->cfg.mqtt_use_tls) {
        on_esp32_tls_stop_mqtt_clients();
        s_active_mqtt_cli  = nullptr;
        gw->mqtt_connected = false;
    }

    HTTPClient http;
    http.setConnectTimeout(3000);
    http.setTimeout(5000);
    if (!on_esp32_tls_http_begin(http, gw->cfg.ota_url)) {
        Serial.println("[OpenNear OTA] HTTP/HTTPS begin failed");
        return;
    }
    int code = http.GET();
    int total = http.getSize();
    if (code != HTTP_CODE_OK || total <= 0) {
        Serial.printf("[OpenNear OTA] HTTP GET failed: code=%d size=%d\n", code, total);
        http.end();
        return;
    }

    const esp_partition_t *part = esp_ota_get_next_update_partition(nullptr);
    if (!part || static_cast<uint32_t>(total) > part->size) {
        Serial.println("[OpenNear OTA] No OTA partition or image too large");
        http.end();
        return;
    }

    esp_ota_handle_t handle = 0;
    if (esp_ota_begin(part, static_cast<size_t>(total), &handle) != ESP_OK) {
        Serial.println("[OpenNear OTA] esp_ota_begin failed");
        http.end();
        return;
    }

    WiFiClient *stream = http.getStreamPtr();
    uint8_t buf[512];
    uint32_t written = 0u;
    uint32_t crc = 0u;
    while (http.connected() && written < static_cast<uint32_t>(total)) {
        size_t avail = stream->available();
        if (avail > 0u) {
            size_t to_read = (avail > sizeof(buf)) ? sizeof(buf) : avail;
            int n = stream->readBytes(buf, to_read);
            if (n > 0) {
                if (esp_ota_write(handle, buf, static_cast<size_t>(n)) != ESP_OK) {
                    esp_ota_abort(handle);
                    http.end();
                    return;
                }
                crc = on_ota_crc32_update(crc, buf, static_cast<size_t>(n));
                written += static_cast<uint32_t>(n);
            }
        } else {
            vTaskDelay(1);
        }
    }
    http.end();

    if (written != static_cast<uint32_t>(total) || esp_ota_end(handle) != ESP_OK) {
        Serial.printf("[OpenNear OTA] Download incomplete (%u/%d)\n", (unsigned)written, total);
        return;
    }

    /* Self-update if target matches our own Short-ID */
    if (gw->cfg.ota_target_id == node->short_id) {
        Serial.printf("[OpenNear OTA] Self-OTA complete (%u B, CRC32=0x%08X) -> rebooting!\n",
                      (unsigned)written, (unsigned)crc);
        if (esp_ota_set_boot_partition(part) == ESP_OK) {
            delay(50);
            esp_restart();
        }
        return;
    }

    /* Otherwise distribute staged partition image over OpenNear to target node(s)! */
    if (node->ota && node->role == ON_ROLE_G) {
        uint8_t tag8[ON_OTA_AUTH_TAG_LEN] = {0};
        if (node->sec.enabled) {
            /* Compute true streaming 64-bit AES-128-CBC-MAC over all staged flash partition bytes */
            on_ota_meta_t tmp_meta;
            memset(&tmp_meta, 0, sizeof(tmp_meta));
            tmp_meta.ota_id     = static_cast<uint16_t>((gw->cfg.ota_target_ver & 0xFFFFu) ^ 0x5A01u);
            tmp_meta.fw_version = gw->cfg.ota_target_ver;
            tmp_meta.total_size = written;
            tmp_meta.crc32      = crc;
            tmp_meta.hw_compat  = 1u;
            on_ota_compute_auth_tag_reader(node->sec.master_key, &tmp_meta,
                                           on_esp32_ota_read_staged, nullptr, tag8);
        }
        on_esp32_node_lock();
        on_ota_tx_start_reader(&node->ota->tx,
                               static_cast<uint16_t>((gw->cfg.ota_target_ver & 0xFFFFu) ^ 0x5A01u),
                               gw->cfg.ota_target_ver,
                               gw->cfg.ota_target_id,
                               1u,
                               ON_OTA_FLAG_AUTO_REBOOT,
                               written,
                               64u,
                               crc,
                               tag8,
                               on_esp32_ota_read_staged,
                               nullptr);
        node->ota->tx.meta_repeat_cnt = 2u;
        on_esp32_node_unlock();
        Serial.printf("[OpenNear OTA] Staged %u B (CRC32=0x%08X) -> streaming to OpenNear target 0x%03X!\n",
                      (unsigned)written, (unsigned)crc, (unsigned)gw->cfg.ota_target_id);
    }
}
#endif /* OPENNEAR_ENABLE_WIFI_GW */

#ifdef OPENNEAR_ENABLE_BLE_GW
static constexpr const char *kBleSvcUuid  = "0000ffe0-0000-1000-8000-00805f9b34fb";
static constexpr const char *kBleTxUuid   = "0000ffe1-0000-1000-8000-00805f9b34fb";
static constexpr const char *kBleCtrlUuid = "0000ffe2-0000-1000-8000-00805f9b34fb";

static BLECharacteristic *s_ble_tx_char = nullptr;
static BLEScan           *s_ble_scanner = nullptr;
static volatile bool      s_ble_scan_active  = false;
static bool               s_ble_scan_enabled = true;
static uint32_t           s_ble_scan_last_ms = 0u;

/* Lock-free SPSC ring decoupling Bluedroid scan callback from Core-1 node mutex */
#define ON_BLE_RX_RING_SZ 16u
typedef struct {
    uint8_t tag_mac48[6];
    uint8_t source;
    int8_t  tx_power_1m_dbm;
    int8_t  rssi_dbm;
    char    label[ON_LOC_LABEL_MAX_LEN];
} on_ble_rx_evt_t;
static on_ble_rx_evt_t  s_ble_rx_ring[ON_BLE_RX_RING_SZ];
static volatile uint8_t s_ble_rx_head = 0u;
static volatile uint8_t s_ble_rx_tail = 0u;

class OpenNearBleBeaconScanCb : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) override {
        const uint8_t *pay = dev.getPayload();
        size_t plen = dev.getPayloadLength();
        if (!pay || plen < 4u) return;
        on_ble_beacon_t b;
        if (!on_ble_beacon_parse(pay, plen, &b) || !b.valid) return;
        int rssi = dev.getRSSI();
        if (rssi >= 0 || rssi < -120) return;
        uint8_t next = static_cast<uint8_t>((s_ble_rx_tail + 1u) & (ON_BLE_RX_RING_SZ - 1u));
        if (next == s_ble_rx_head) return; /* ring full: drop newest cleanly */
        const uint8_t *raw_mac = reinterpret_cast<const uint8_t *>(dev.getAddress().getNative());
        on_ble_rx_evt_t &e = s_ble_rx_ring[s_ble_rx_tail];
        on_ble_beacon_derive_mac(&b, raw_mac, e.tag_mac48);
        e.source          = static_cast<uint8_t>(b.source);
        e.tx_power_1m_dbm = b.tx_power_1m_dbm;
        e.rssi_dbm        = static_cast<int8_t>(rssi);
        memcpy(e.label, b.label, sizeof(e.label));
        __sync_synchronize();
        s_ble_rx_tail = next;
    }
};

static void on_ble_scan_done_cb(BLEScanResults res) {
    (void)res;
    if (s_ble_scanner) s_ble_scanner->clearResults();
    s_ble_scan_active  = false;
    s_ble_scan_last_ms = millis();
}

class OpenNearBleServerCb : public BLEServerCallbacks {
    void onConnect(BLEServer *srv) override {
        (void)srv;
        if (s_ble_scanner && s_ble_scan_active) {
            s_ble_scanner->stop();
            s_ble_scan_active = false;
        }
        if (s_gw_ref) on_gw_on_ble_state(s_gw_ref, true, 185u);
        Serial.println("[OpenNear BLE] Mobile/PC client connected");
    }
    void onDisconnect(BLEServer *srv) override {
        if (s_gw_ref) on_gw_on_ble_state(s_gw_ref, false, 0u);
        Serial.println("[OpenNear BLE] Client disconnected -> restarting advertising");
        if (srv) srv->startAdvertising();
    }
};

class OpenNearBleCtrlCb : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *ch) override {
        if (!ch || !s_gw_ref) return;
        std::string val = ch->getValue();
        if (!val.empty()) {
            on_err_t rc = on_gw_handle_control_text(s_gw_ref, val.data(), val.size(), true);
            Serial.printf("[OpenNear BLE] RX ctrl (%u B) rc=%d\n",
                          (unsigned)val.size(), (int)rc);
        }
    }
};

static on_err_t esp32_ble_notify_cb(void *ctx, const uint8_t *data, size_t len) {
    (void)ctx;
    if (!s_ble_tx_char || !data || len == 0u) return ON_ERR_NOT_READY;
    /* Auto-fragment large JSON payloads (>180B) with newline terminator so mobile apps never see truncated JSON */
    size_t off = 0u;
    while (off < len) {
        size_t chunk = len - off;
        if (chunk > 180u) chunk = 180u;
        s_ble_tx_char->setValue(const_cast<uint8_t *>(data + off), chunk);
        s_ble_tx_char->notify();
        off += chunk;
        if (off < len) vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (data[len - 1u] != '\n') {
        uint8_t nl = '\n';
        s_ble_tx_char->setValue(&nl, 1u);
        s_ble_tx_char->notify();
    }
    return ON_OK;
}
#endif /* OPENNEAR_ENABLE_BLE_GW */

static on_err_t esp32_downlink_cb(void *ctx, const on_gw_bridge_msg_t *msg) {
    (void)ctx;
    if (!msg) return ON_ERR_INVALID_ARG;
    if (msg->bridge_type == ON_GW_MSG_WIFI_PROV) {
        Serial.printf("[OpenNear Sync] Encrypted air-sync config applied (%u B): %.*s\n",
                      (unsigned)msg->payload_len, (int)msg->payload_len,
                      reinterpret_cast<const char *>(msg->payload));
        return ON_OK;
    }
    on_esp32_demo_on_cmd(msg->payload, msg->payload_len);
    if (s_user_cmd_cb) s_user_cmd_cb(s_user_cmd_ctx, msg->payload, msg->payload_len);
    Serial.printf("[OpenNear GW] Downlink cmd seq=%u type=%u len=%u: %.*s\n",
                  (unsigned)msg->seq, (unsigned)msg->bridge_type,
                  (unsigned)msg->payload_len, (int)msg->payload_len,
                  reinterpret_cast<const char *>(msg->payload));
    return ON_OK;
}

void on_esp32_net_gw_preinit_wifi(on_gw_ctx_t *gw, const char *default_ssid,
                                  const char *default_pass, const char *default_cloud_url) {
    if (!gw) return;
    on_gw_mode_t mode = ON_GW_MODE_OFF;
#if defined(OPENNEAR_ENABLE_WIFI_GW) && defined(OPENNEAR_ENABLE_BLE_GW)
    mode = ON_GW_MODE_WIFI_AND_BLE;
#elif defined(OPENNEAR_ENABLE_WIFI_GW)
    mode = ON_GW_MODE_WIFI_STA;
#elif defined(OPENNEAR_ENABLE_BLE_GW)
    mode = ON_GW_MODE_BLE_GATT;
#endif

    on_gw_transport_ops_t ops;
    memset(&ops, 0, sizeof(ops));
    ops.user_ctx              = gw;
    ops.on_downlink_delivered = esp32_downlink_cb;
#ifdef OPENNEAR_ENABLE_WIFI_GW
    ops.wifi_connect          = esp32_wifi_connect_cb;
    ops.cloud_post_json       = esp32_cloud_post_cb;
    ops.mqtt_publish          = esp32_mqtt_publish_cb;
    ops.on_channel_align      = esp32_channel_align_cb;
#endif
#ifdef OPENNEAR_ENABLE_BLE_GW
    ops.ble_notify            = esp32_ble_notify_cb;
#endif
    on_gw_init(gw, mode, &ops);

#ifdef OPENNEAR_ENABLE_WIFI_GW
    String ssid = default_ssid ? String(default_ssid) : String("");
    String pass = default_pass ? String(default_pass) : String("");
    String url  = default_cloud_url ? String(default_cloud_url) : String("");

    if (s_prefs.begin("opennear_gw", true)) {
        String nvs_ssid  = s_prefs.getString("ssid", "");
        String nvs_pass  = s_prefs.getString("pass", "");
        String nvs_url   = s_prefs.getString("url", "");
        String nvs_mhost = s_prefs.getString("mhost", "");
        uint16_t mport   = s_prefs.getUShort("mport", 1883u);
        String nvs_muser = s_prefs.getString("muser", "");
        String nvs_mpass = s_prefs.getString("mpass", "");
        bool nvs_mtls    = s_prefs.getBool("mtls", mport == 8883u);
        String nvs_tok   = s_prefs.getString("atoken", "");
        s_prefs.end();
        if (nvs_ssid.length() > 0)  ssid = nvs_ssid;
        if (nvs_pass.length() > 0)  pass = nvs_pass;
        if (nvs_url.length() > 0)   url  = nvs_url;
        if (nvs_mhost.length() > 0) {
            strncpy(gw->cfg.mqtt_host, nvs_mhost.c_str(), sizeof(gw->cfg.mqtt_host) - 1u);
            gw->cfg.mqtt_port    = mport;
            gw->cfg.mqtt_use_tls = nvs_mtls;
            strncpy(gw->cfg.mqtt_user, nvs_muser.c_str(), sizeof(gw->cfg.mqtt_user) - 1u);
            strncpy(gw->cfg.mqtt_pass, nvs_mpass.c_str(), sizeof(gw->cfg.mqtt_pass) - 1u);
        }
        if (nvs_tok.length() > 0) {
            on_gw_set_admin_token(gw, nvs_tok.c_str());
        }
    }
    on_esp32_tls_init_from_nvs();
#ifdef OPENNEAR_ADMIN_TOKEN
    if (gw->cfg.admin_token[0] == '\0') {
        on_gw_set_admin_token(gw, OPENNEAR_ADMIN_TOKEN);
    }
#endif

    on_gw_set_wifi_creds(gw, ssid.c_str(), pass.c_str(), url.c_str());
    gw->cfg.wifi_creds_updated = false;
    s_sta_start_ms = millis();
    if (ssid.length() > 0) {
        WiFi.mode(WIFI_STA);
        apply_wifi_coex_ps();
        WiFi.begin(ssid.c_str(), pass.c_str());
        Serial.printf("[OpenNear GW] Wi-Fi STA starting (SSID='%s')\n", ssid.c_str());
    } else {
        /* No saved Wi-Fi SSID -> boot straight into SoftAP Web Provisioning Portal */
        start_softap_portal();
    }
#else
    (void)default_ssid;
    (void)default_pass;
    (void)default_cloud_url;
#endif
}

void on_esp32_net_gw_start(on_node_t *node, on_gw_ctx_t *gw, const char *ble_name) {
    s_node_ref = node;
    s_gw_ref   = gw;
    if (node && gw) {
        on_node_attach_gateway(node, gw);
    }
#ifdef OPENNEAR_ENABLE_WIFI_GW
    load_anchors_from_nvs();
    load_fences_from_nvs();
    load_rules_from_nvs();
    if (node) {
        on_node_set_rssi_observer(node, esp32_on_rssi_observed_cb, node);
        on_node_set_ble_beacon_observer(node, esp32_on_ble_beacon_observed_cb, node);
        on_node_set_csi_observer(node, esp32_on_csi_observed_cb, node);
    }
#endif
#ifdef OPENNEAR_ENABLE_BLE_GW
    if (s_prefs.begin("opennear_gw", true)) {
        s_ble_scan_enabled = s_prefs.getBool("blescan", true);
        s_prefs.end();
    }
    if (gw) gw->cfg.ble_scan_enabled = s_ble_scan_enabled;
    /* Guarantee Wi-Fi modem sleep is enabled before starting BT controller under ESP-IDF SW coexist */
#ifdef OPENNEAR_ENABLE_WIFI_GW
    apply_wifi_coex_ps();
#endif
    /* Start BT controller in BLE-only mode & release unused Classic BT DRAM (~36 KB saved for TLS/Web) */
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        bt_cfg.mode = ESP_BT_MODE_BLE;
        esp_err_t brc = esp_bt_controller_init(&bt_cfg);
        if (brc == ESP_OK) {
            brc = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        }
        if (brc != ESP_OK) {
            Serial.printf("[OpenNear BLE] Controller BLE-only start failed (rc=0x%x); Wi-Fi/Web gateway continues\n",
                          (unsigned)brc);
            return;
        }
    }
    BLEDevice::init(ble_name && ble_name[0] ? ble_name : "OpenNear-GW");
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        Serial.println("[OpenNear BLE] Bluedroid enable failed (low heap); Wi-Fi/Web gateway continues");
        return;
    }
    BLEServer *srv = BLEDevice::createServer();
    srv->setCallbacks(new OpenNearBleServerCb());
    BLEService *svc = srv->createService(kBleSvcUuid);

    s_ble_tx_char = svc->createCharacteristic(kBleTxUuid, BLECharacteristic::PROPERTY_NOTIFY);
    s_ble_tx_char->addDescriptor(new BLE2902());

    BLECharacteristic *ctrl = svc->createCharacteristic(
        kBleCtrlUuid, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    ctrl->setCallbacks(new OpenNearBleCtrlCb());

    svc->start();
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(kBleSvcUuid);
    adv->setScanResponse(true);
    adv->start();

    /* v7.1: Low-duty passive BLE iBeacon/Eddystone scanner (25% duty, shouldParse=false -> 0 heap churn) */
    s_ble_scanner = BLEDevice::getScan();
    if (s_ble_scanner) {
        s_ble_scanner->setAdvertisedDeviceCallbacks(new OpenNearBleBeaconScanCb(), true, false);
        s_ble_scanner->setActiveScan(false);
        s_ble_scanner->setInterval(100);
        s_ble_scanner->setWindow(25);
        s_ble_scan_last_ms = millis();
    }
    Serial.printf("[OpenNear BLE] GATT Bridge + Passive iBeacon/Eddystone Scanner (%s) as '%s' (heap=%u B)\n",
                  s_ble_scan_enabled ? "ON" : "OFF",
                  ble_name && ble_name[0] ? ble_name : "OpenNear-GW",
                  (unsigned)ESP.getFreeHeap());
#else
    (void)ble_name;
#endif
}

void on_esp32_net_gw_poll(on_node_t *node, on_gw_ctx_t *gw) {
    if (!gw) return;
#ifdef OPENNEAR_ENABLE_WIFI_GW
    bool connected = (WiFi.status() == WL_CONNECTED);
    if (connected && !s_wifi_was_connected) {
        s_wifi_was_connected = true;
        IPAddress ip = WiFi.localIP();
        uint8_t ip4[4] = { ip[0], ip[1], ip[2], ip[3] };
        uint8_t ch = static_cast<uint8_t>(WiFi.channel());
        on_gw_on_wifi_connected(gw, ch, static_cast<int8_t>(WiFi.RSSI()), ip4);
        start_local_http_api();
        s_mqtt_backoff_ms    = 5000u;
        s_last_mqtt_retry_ms = 0u;
        if (!s_mdns_active) {
            uint8_t mac[6] = {0};
            WiFi.macAddress(mac);
            char mdns_name[24];
            snprintf(mdns_name, sizeof(mdns_name), "opennear-%02x%02x",
                     (unsigned)mac[4], (unsigned)mac[5]);
            if (MDNS.begin(mdns_name)) {
                MDNS.addService("http", "tcp", 80);
                snprintf(s_mdns_host, sizeof(s_mdns_host), "%s.local", mdns_name);
                s_mdns_active = true;
            }
        }
        if (s_ap_active) {
            /* Give phone browser 5 s to display the new LAN IP before closing fallback SoftAP */
            s_ap_off_at_ms = millis() + 5000u;
        }
        Serial.printf("[OpenNear GW] Wi-Fi ONLINE http://%u.%u.%u.%u/ (http://%s/) ch=%u rssi=%d\n",
                      ip4[0], ip4[1], ip4[2], ip4[3],
                      s_mdns_active ? s_mdns_host : "mDNS-off",
                      (unsigned)ch, (int)WiFi.RSSI());
    } else if (!connected && s_wifi_was_connected) {
        s_wifi_was_connected = false;
        s_sta_start_ms = millis();
        if (s_mdns_active) {
            MDNS.end();
            s_mdns_active = false;
            s_mdns_host[0] = '\0';
        }
        on_gw_on_wifi_disconnected(gw);
        Serial.println("[OpenNear GW] Wi-Fi disconnected, retrying...");
    } else if (!connected && gw->cfg.wifi_ssid[0] != '\0' &&
               (millis() - s_last_wifi_retry_ms) > 10000u) {
        s_last_wifi_retry_ms = millis();
        WiFi.reconnect();
    }

    /* Auto-fallback to SoftAP Web Portal if router cannot be reached within 20 s */
    if (!connected && !s_ap_active &&
        (gw->cfg.wifi_ssid[0] == '\0' || (millis() - s_sta_start_ms) >= 20000u)) {
        start_softap_portal();
    }

    /* Gracefully shut down SoftAP once STA is stably online to keep single-channel RF clean */
    if (s_ap_active && s_ap_off_at_ms > 0u &&
        static_cast<int32_t>(millis() - s_ap_off_at_ms) >= 0) {
        s_ap_off_at_ms = 0u;
        if (connected) {
            if (s_dns_active) {
                s_dns_srv.stop();
                s_dns_active = false;
            }
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_STA);
            apply_wifi_coex_ps();
            esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
            s_ap_active = false;
            Serial.println("[OpenNear Portal] Fallback SoftAP closed; Web Portal stays live on LAN STA IP");
        }
    }
    if (s_dns_active) {
        s_dns_srv.processNextRequest();
    }
    if (s_http_srv_started) {
        s_http_srv.handleClient();
    }
    if (gw->cfg.wifi_creds_updated) {
        gw->cfg.wifi_creds_updated = false;
        if (gw->cfg.wifi_ssid[0] != '\0') {
            save_wifi_nvs(gw->cfg.wifi_ssid, gw->cfg.wifi_pass, gw->cfg.cloud_url);
            WiFi.mode(s_ap_active ? WIFI_AP_STA : WIFI_STA);
            apply_wifi_coex_ps();
            WiFi.begin(gw->cfg.wifi_ssid, gw->cfg.wifi_pass);
            s_sta_start_ms = millis();
            Serial.printf("[OpenNear GW] Connecting to public Wi-Fi SSID='%s'...\n", gw->cfg.wifi_ssid);
            char pbuf[ON_GW_MAX_PAYLOAD_LEN + 1u];
            size_t wlen = on_gw_format_wifi_prov(gw, pbuf, sizeof(pbuf));
            if (wlen > 0u) sync_prov_to_standby_gw(node, pbuf, wlen);
        }
    }
    esp32_mqtt_poll(node, gw);

    if (gw->cfg.wifi_reset_requested) {
        gw->cfg.wifi_reset_requested = false;
        if (s_prefs.begin("opennear_gw", false)) {
            s_prefs.remove("ssid");
            s_prefs.remove("pass");
            s_prefs.end();
        }
        on_esp32_tls_stop_mqtt_clients();
        s_active_mqtt_cli    = nullptr;
        gw->mqtt_connected   = false;
        s_wifi_was_connected = false;
        if (s_mdns_active) {
            MDNS.end();
            s_mdns_active = false;
            s_mdns_host[0] = '\0';
        }
        WiFi.disconnect(true, false);
        on_gw_on_wifi_disconnected(gw);
        start_softap_portal();
        Serial.println("[OpenNear GW] Wi-Fi credentials cleared; SoftAP provisioning portal reopened");
    }

    if (gw->cfg.reboot_requested) {
        gw->cfg.reboot_requested = false;
        Serial.println("[OpenNear GW] Reboot requested -> restarting in 300 ms...");
        Serial.flush();
        delay(300);
        esp_restart();
    }

    if (gw->cfg.admin_token_updated) {
        gw->cfg.admin_token_updated = false;
        if (s_prefs.begin("opennear_gw", false)) {
            s_prefs.putString("atoken", gw->cfg.admin_token);
            s_prefs.end();
        }
        char pbuf[64];
        int tn = snprintf(pbuf, sizeof(pbuf), "TOKEN:%s", gw->cfg.admin_token);
        if (tn >= 6) sync_prov_to_standby_gw(node, pbuf, static_cast<size_t>(tn));
        Serial.printf("[OpenNear GW] Admin token %s in NVS\n",
                      gw->cfg.admin_token[0] ? "updated & saved" : "cleared");
    }

    if (gw->cfg.alias_updated) {
        gw->cfg.alias_updated = false;
        save_mac_alias(gw->cfg.alias_mac, gw->cfg.alias_name);
        s_last_ha_pub_ms = 0u; /* Republish HA Discovery immediately with new friendly name */
        char mhex[13], pbuf[64];
        format_mac_key(gw->cfg.alias_mac, mhex);
        int an = snprintf(pbuf, sizeof(pbuf), "ALIAS:%s|%s", mhex, gw->cfg.alias_name);
        if (an > 6) sync_prov_to_standby_gw(node, pbuf, static_cast<size_t>(an));
        Serial.printf("[OpenNear GW] Alias updated for %02X:%02X:%02X:%02X:%02X:%02X -> '%s'\n",
                      (unsigned)gw->cfg.alias_mac[0], (unsigned)gw->cfg.alias_mac[1],
                      (unsigned)gw->cfg.alias_mac[2], (unsigned)gw->cfg.alias_mac[3],
                      (unsigned)gw->cfg.alias_mac[4], (unsigned)gw->cfg.alias_mac[5],
                      gw->cfg.alias_name);
    }

    if (gw->cfg.tag_updated) {
        gw->cfg.tag_updated = false;
        save_mac_tags(gw->cfg.tag_mac, gw->cfg.tag_list);
        char mhex[13], pbuf[64];
        format_mac_key(gw->cfg.tag_mac, mhex);
        int gn = snprintf(pbuf, sizeof(pbuf), "TAG:%s|%s", mhex, gw->cfg.tag_list);
        if (gn > 4) sync_prov_to_standby_gw(node, pbuf, static_cast<size_t>(gn));
        Serial.printf("[OpenNear Tag] Tags updated for %02X:%02X:%02X:%02X:%02X:%02X -> '%s'\n",
                      (unsigned)gw->cfg.tag_mac[0], (unsigned)gw->cfg.tag_mac[1],
                      (unsigned)gw->cfg.tag_mac[2], (unsigned)gw->cfg.tag_mac[3],
                      (unsigned)gw->cfg.tag_mac[4], (unsigned)gw->cfg.tag_mac[5],
                      gw->cfg.tag_list);
    }

    if (gw->cfg.anchor_updated) {
        gw->cfg.anchor_updated = false;
        on_esp32_node_lock();
        if (!s_loc_inited) { on_loc_init(&s_loc); s_loc_inited = true; }
        uint16_t sid = node ? on_node_find_short_id_by_mac(node, gw->cfg.anchor_mac) : ON_SHORT_ID_UNASSIGNED;
        on_loc_set_anchor(&s_loc, gw->cfg.anchor_mac, sid,
                          gw->cfg.anchor_room, gw->cfg.anchor_x_cm, gw->cfg.anchor_y_cm);
        on_esp32_node_unlock();
        save_anchors_to_nvs();
        char mhex[13], pbuf[80];
        format_mac_key(gw->cfg.anchor_mac, mhex);
        int an = snprintf(pbuf, sizeof(pbuf), "ANCHOR:%s|%s|%ld|%ld",
                          mhex, gw->cfg.anchor_room,
                          (long)gw->cfg.anchor_x_cm, (long)gw->cfg.anchor_y_cm);
        if (an > 7) sync_prov_to_standby_gw(node, pbuf, static_cast<size_t>(an));
        Serial.printf("[OpenNear Locate] Anchor %02X:%02X:%02X:%02X:%02X:%02X -> room='%s' (%ld,%ld cm)\n",
                      (unsigned)gw->cfg.anchor_mac[0], (unsigned)gw->cfg.anchor_mac[1],
                      (unsigned)gw->cfg.anchor_mac[2], (unsigned)gw->cfg.anchor_mac[3],
                      (unsigned)gw->cfg.anchor_mac[4], (unsigned)gw->cfg.anchor_mac[5],
                      gw->cfg.anchor_room, (long)gw->cfg.anchor_x_cm, (long)gw->cfg.anchor_y_cm);
    }

    if (gw->cfg.cmd_tag_requested && node) {
        gw->cfg.cmd_tag_requested = false;
        uint32_t sent_cnt = dispatch_tag_or_local_cmd(node, gw->cfg.cmd_tag,
                                                      gw->cfg.cmd_tag_payload,
                                                      gw->cfg.cmd_tag_payload_len);
        Serial.printf("[OpenNear Tag] Group CMD tag='%s' -> dispatched to %u node(s)\n",
                      gw->cfg.cmd_tag, (unsigned)sent_cnt);
    }

    if (gw->cfg.fence_updated) {
        gw->cfg.fence_updated = false;
        on_esp32_node_lock();
        if (!s_loc_inited) { on_loc_init(&s_loc); s_loc_inited = true; }
        on_loc_set_geofence(&s_loc, gw->cfg.fence_name,
                            gw->cfg.fence_x_cm, gw->cfg.fence_y_cm, gw->cfg.fence_radius_cm);
        on_esp32_node_unlock();
        if (!s_loc_demo_running) save_fences_to_nvs();
        char pbuf[80];
        int fn = snprintf(pbuf, sizeof(pbuf), "FENCE:%s|%ld|%ld|%u",
                          gw->cfg.fence_name, (long)gw->cfg.fence_x_cm,
                          (long)gw->cfg.fence_y_cm, (unsigned)gw->cfg.fence_radius_cm);
        if (fn > 6) sync_prov_to_standby_gw(node, pbuf, static_cast<size_t>(fn));
        Serial.printf("[OpenNear Fence] Geofence '%s' (%ld,%ld cm, r=%u cm)\n",
                      gw->cfg.fence_name, (long)gw->cfg.fence_x_cm,
                      (long)gw->cfg.fence_y_cm, (unsigned)gw->cfg.fence_radius_cm);
    }

    if (gw->cfg.rule_updated) {
        gw->cfg.rule_updated = false;
        on_esp32_node_lock();
        if (!s_loc_inited) { on_loc_init(&s_loc); s_loc_inited = true; }
        on_loc_set_rule(&s_loc, gw->cfg.rule_id, gw->cfg.rule_trig_type,
                        gw->cfg.rule_zone, gw->cfg.rule_csi_thresh,
                        gw->cfg.rule_cmd_tag, gw->cfg.rule_cmd_payload);
        on_esp32_node_unlock();
        if (!s_loc_demo_running) save_rules_to_nvs();
        char pbuf[96];
        int rn = snprintf(pbuf, sizeof(pbuf), "RULE:%u|%u|%s|%u|%s|%s",
                          (unsigned)gw->cfg.rule_id, (unsigned)gw->cfg.rule_trig_type,
                          gw->cfg.rule_zone, (unsigned)gw->cfg.rule_csi_thresh,
                          gw->cfg.rule_cmd_tag, gw->cfg.rule_cmd_payload);
        if (rn > 5) sync_prov_to_standby_gw(node, pbuf, static_cast<size_t>(rn));
        Serial.printf("[OpenNear Rule] Rule R#%u trig=%u zone='%s' th=%u -> tag='%s' cmd='%s'\n",
                      (unsigned)gw->cfg.rule_id, (unsigned)gw->cfg.rule_trig_type,
                      gw->cfg.rule_zone, (unsigned)gw->cfg.rule_csi_thresh,
                      gw->cfg.rule_cmd_tag, gw->cfg.rule_cmd_payload);
    }

    /* v7.2: When a new Standby Backup_ID joins Primary G, bulk-clone saved Anchors, Geofences & Rules */
    if (node && s_loc_inited && !s_loc_demo_running) {
        uint16_t cur_bk = ON_SHORT_ID_UNASSIGNED;
        on_esp32_node_lock();
        if (node->role == ON_ROLE_G) cur_bk = node->backup_id;
        on_esp32_node_unlock();
        if (cur_bk != ON_SHORT_ID_UNASSIGNED && cur_bk != s_last_synced_bk_id) {
            s_last_synced_bk_id = cur_bk;
            char pbuf[96], mhex[13];
            for (uint32_t i = 0u; i < ON_LOC_MAX_ANCHORS; ++i) {
                if (!s_loc.anchors[i].active) continue;
                format_mac_key(s_loc.anchors[i].mac48, mhex);
                int n = snprintf(pbuf, sizeof(pbuf), "ANCHOR:%s|%s|%ld|%ld",
                                 mhex, s_loc.anchors[i].room,
                                 (long)s_loc.anchors[i].x_cm, (long)s_loc.anchors[i].y_cm);
                if (n > 7) sync_prov_to_standby_gw(node, pbuf, static_cast<size_t>(n));
            }
            for (uint32_t i = 0u; i < ON_LOC_MAX_GEOFENCES; ++i) {
                if (!s_loc.fences[i].active) continue;
                int n = snprintf(pbuf, sizeof(pbuf), "FENCE:%s|%ld|%ld|%u",
                                 s_loc.fences[i].name, (long)s_loc.fences[i].x_cm,
                                 (long)s_loc.fences[i].y_cm, (unsigned)s_loc.fences[i].radius_cm);
                if (n > 6) sync_prov_to_standby_gw(node, pbuf, static_cast<size_t>(n));
            }
            for (uint32_t i = 0u; i < ON_LOC_MAX_RULES; ++i) {
                if (!s_loc.rules[i].active) continue;
                int n = snprintf(pbuf, sizeof(pbuf), "RULE:%u|%u|%s|%u|%s|%s",
                                 (unsigned)s_loc.rules[i].id, (unsigned)s_loc.rules[i].trig_type,
                                 s_loc.rules[i].zone, (unsigned)s_loc.rules[i].csi_threshold,
                                 s_loc.rules[i].cmd_tag, s_loc.rules[i].cmd_payload);
                if (n > 5) sync_prov_to_standby_gw(node, pbuf, static_cast<size_t>(n));
            }
        } else if (cur_bk == ON_SHORT_ID_UNASSIGNED) {
            s_last_synced_bk_id = ON_SHORT_ID_UNASSIGNED;
        }
    }

    if (gw->cfg.loc_demo_updated) {
        gw->cfg.loc_demo_updated = false;
        uint32_t dnow = millis();
        on_esp32_node_lock();
        if (!s_loc_inited) { on_loc_init(&s_loc); s_loc_inited = true; }
        if (gw->cfg.loc_demo_mode == 1u) {
            on_loc_load_demo_preset(&s_loc, node ? node->mac48 : nullptr, dnow);
            s_loc_demo_running  = true;
            s_loc_demo_step     = 1u;
            s_last_demo_step_ms = dnow;
            Serial.println("[OpenNear Demo] Loaded 4-Room + 2-Geofence + Wi-Fi CSI Showcase Preset (AUTO RUN)");
        } else if (gw->cfg.loc_demo_mode == 2u) {
            if (!s_loc_demo_running && s_loc.fences[0].active == false) {
                on_loc_load_demo_preset(&s_loc, node ? node->mac48 : nullptr, dnow);
            }
            on_loc_step_demo(&s_loc, s_loc_demo_step++, dnow);
            s_last_demo_step_ms = dnow;
        } else {
            s_loc_demo_running = false;
            on_loc_init(&s_loc);
            load_anchors_from_nvs();
            load_fences_from_nvs();
            load_rules_from_nvs();
            Serial.println("[OpenNear Demo] Showcase stopped -> restored NVS anchors/fences/rules");
        }
        on_esp32_node_unlock();
    }

    /* v7.2: Step interactive showcase trajectory every 2.5 s when demo mode is running */
    uint32_t loc_now_ms = millis();
    if (s_loc_demo_running && (loc_now_ms - s_last_demo_step_ms) >= 2500u) {
        s_last_demo_step_ms = loc_now_ms;
        on_esp32_node_lock();
        on_loc_step_demo(&s_loc, s_loc_demo_step++, loc_now_ms);
        on_esp32_node_unlock();
    }

    /* v7.2: Evaluate offline edge geofence & Wi-Fi CSI rules every 500 ms (zero cloud / zero HA dependency) */
    if (s_loc_inited && (!node || node->role == ON_ROLE_G || s_loc_demo_running) &&
        (loc_now_ms - s_last_rule_eval_ms) >= 500u) {
        s_last_rule_eval_ms = loc_now_ms;
        on_esp32_node_lock();
        on_loc_evaluate_rules(&s_loc, loc_now_ms, esp32_on_rule_fired_cb, node);
        on_esp32_node_unlock();
    }

    if (connected && gw->cfg.ota_pull_requested) {
        gw->cfg.ota_pull_requested = false;
        esp32_pull_http_ota(node, gw);
    }

    /* Auto-commit staged cluster OTA once all target nodes verify & report READY */
    if (node) {
        bool committed_now = false;
        on_esp32_node_lock();
        if (node->role == ON_ROLE_G && node->ota &&
            node->ota->tx.active && !node->ota->tx.commit_sent &&
            on_ota_tx_all_targets_ready(&node->ota->tx, 1u)) {
            on_node_ota_commit(node, 8u, ON_OTA_FLAG_AUTO_REBOOT);
            committed_now = true;
        }
        on_esp32_node_unlock();
        if (committed_now) {
            Serial.println("[OpenNear OTA] All targets READY -> scheduled synchronized cluster commit (+8 SFs)");
        }
    }
#else
    (void)node;
#endif

#if defined(OPENNEAR_ENABLE_WIFI_GW) || defined(OPENNEAR_ENABLE_BLE_GW)
#ifdef OPENNEAR_ENABLE_BLE_GW
    if (gw->cfg.ble_scan_updated) {
        gw->cfg.ble_scan_updated = false;
        s_ble_scan_enabled = gw->cfg.ble_scan_enabled;
        if (s_prefs.begin("opennear_gw", false)) {
            s_prefs.putBool("blescan", s_ble_scan_enabled);
            s_prefs.end();
        }
        if (!s_ble_scan_enabled && s_ble_scanner && s_ble_scan_active) {
            s_ble_scanner->stop();
            s_ble_scan_active = false;
        }
        Serial.printf("[OpenNear BLE] Passive iBeacon/Eddystone scanner -> %s\n",
                      s_ble_scan_enabled ? "ENABLED" : "DISABLED");
    }

    /* Drain lock-free passive BLE beacon ring into OpenNear cluster / local locator */
    while (s_ble_rx_head != s_ble_rx_tail) {
        on_ble_rx_evt_t ev = s_ble_rx_ring[s_ble_rx_head];
        __sync_synchronize();
        s_ble_rx_head = static_cast<uint8_t>((s_ble_rx_head + 1u) & (ON_BLE_RX_RING_SZ - 1u));
        if (node) {
            on_esp32_node_lock();
#ifdef OPENNEAR_ENABLE_WIFI_GW
            if (node->role == ON_ROLE_G && s_loc_inited) {
                on_loc_observe_beacon(&s_loc, node->mac48, ev.tag_mac48,
                                      static_cast<on_loc_source_t>(ev.source),
                                      ev.tx_power_1m_dbm, ev.rssi_dbm,
                                      ev.label[0] ? ev.label : nullptr, millis());
            } else
#endif
            {
                on_node_report_ble_beacon(node, ev.tag_mac48, ev.source,
                                          ev.tx_power_1m_dbm, ev.rssi_dbm);
            }
            on_esp32_node_unlock();
        }
    }

    /* Non-blocking 2 s passive scan every 6 s (auto-paused during active phone GATT connection) */
    uint32_t ble_now_ms = millis();
    if (s_ble_scan_enabled && s_ble_scanner && !s_ble_scan_active && !gw->ble_connected &&
        (ble_now_ms - s_ble_scan_last_ms) >= 6000u) {
        s_ble_scan_active = s_ble_scanner->start(2u, on_ble_scan_done_cb, false);
        if (!s_ble_scan_active) {
            s_ble_scan_last_ms = ble_now_ms;
        }
    }
#endif

    if (gw->cfg.net_key_updated) {
        gw->cfg.net_key_updated = false;
        static const uint8_t kFallbackKey[16] = {
            0x4F,0x50,0x45,0x4E,0x4E,0x45,0x41,0x52,
            0x33,0x30,0x2D,0x4B,0x45,0x59,0x21,0x01
        };
        const uint8_t *apply_key = gw->cfg.net_key_custom ? gw->cfg.net_key : kFallbackKey;
        if (s_prefs.begin("opennear_gw", false)) {
            if (gw->cfg.net_key_custom) s_prefs.putBytes("netkey", gw->cfg.net_key, 16u);
            else s_prefs.remove("netkey");
            s_prefs.end();
        }
        s_netkey_custom = gw->cfg.net_key_custom;
        if (node) {
            on_esp32_node_lock();
            on_node_enable_security(node, apply_key);
            on_esp32_node_unlock();
        }
        Serial.printf("[OpenNear Sec] Cluster AES-128 key %s in NVS & applied live\n",
                      s_netkey_custom ? "CUSTOM saved" : "reset to DEFAULT");
    }
#endif

    if (node && gw->cfg.topo_query_requested) {
        gw->cfg.topo_query_requested = false;
        static char topo_json[3072];
        on_esp32_node_lock();
        if (node->role != ON_ROLE_G && !node->cluster_snapshot_valid) {
            on_node_request_cluster_snapshot(node);
        }
        size_t jlen = on_node_format_cluster_json(node, gw->cfg.topo_include_data,
                                                  topo_json, sizeof(topo_json));
        on_esp32_node_unlock();
        if (jlen > 0u) {
            Serial.printf("[OpenNear Cluster] %s\n", topo_json);
#ifdef OPENNEAR_ENABLE_BLE_GW
            if (gw->ble_connected) {
                esp32_ble_notify_cb(gw, reinterpret_cast<const uint8_t *>(topo_json), jlen);
            }
#endif
        }
    }

    on_gw_poll_network(gw);
}
