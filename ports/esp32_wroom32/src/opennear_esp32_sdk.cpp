#include "OpenNear.h"

#ifdef ARDUINO
#include "hal_esp32_raw80211.h"
#include "esp32_net_gateway.h"
#include "esp32_ota_flash.h"
#include "esp32_demo_app.h"
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>

static on_node_t   s_sdk_node;
static on_mesh_ctx_t s_sdk_mesh;
static on_gw_ctx_t s_sdk_gw;
#ifdef OPENNEAR_ENABLE_OTA
static on_ota_ctx_t s_sdk_ota;
static bool         s_sdk_ota_boot_confirmed = false;
#endif

static OpenNearCmdCallback  s_sdk_cmd_cb  = nullptr;
static OpenNearRuleCallback s_sdk_rule_cb = nullptr;

static const uint8_t kSdkDefaultKey[ON_SEC_KEY_LEN] = {
    0x4F,0x50,0x45,0x4E,0x4E,0x45,0x41,0x52,
    0x33,0x30,0x2D,0x4B,0x45,0x59,0x21,0x01
};

static void sdk_cmd_trampoline(void *ctx, const uint8_t *payload, size_t len) {
    (void)ctx;
    if (s_sdk_cmd_cb && payload && len > 0u) {
        char buf[ON_GW_MAX_PAYLOAD_LEN + 1u];
        size_t n = (len > ON_GW_MAX_PAYLOAD_LEN) ? ON_GW_MAX_PAYLOAD_LEN : len;
        memcpy(buf, payload, n);
        buf[n] = '\0';
        s_sdk_cmd_cb(buf, n);
    }
}

static void sdk_rule_trampoline(void *ctx, uint8_t rule_id, const char *zone,
                                const char *tag, const char *cmd) {
    (void)ctx;
    if (s_sdk_rule_cb) {
        s_sdk_rule_cb(rule_id, zone ? zone : "", tag ? tag : "", cmd ? cmd : "");
    }
}

static void opennear_sdk_rt_task(void *arg) {
    (void)arg;
    uint32_t iters = 0u;
    for (;;) {
        on_esp32_node_lock();
        on_esp32_hal_poll_rx(&s_sdk_node);
        int64_t now_ns = s_sdk_node.hal.now_ns(s_sdk_node.hal.user_ctx);
        on_node_step(&s_sdk_node, now_ns);
        on_esp32_node_unlock();

        if ((++iters & 0x3Fu) == 0u) {
            const int64_t tick_ns = 1000000000LL / configTICK_RATE_HZ;
            on_esp32_node_lock();
            int64_t now = s_sdk_node.hal.now_ns(s_sdk_node.hal.user_ctx);
            int64_t until_wakeup = on_node_next_wakeup_local_ns(&s_sdk_node, now) - now;
            bool can_delay = !s_sdk_node.sync.locked || until_wakeup > tick_ns + 300000LL;
            on_esp32_node_unlock();
            if (can_delay) {
                vTaskDelay(1);
                continue;
            }
        }
        taskYIELD();
    }
}

OpenNearClass::OpenNearClass()
    : started_(false), last_role_(ON_ROLE_UNJOINED), role_cb_(nullptr) {}

bool OpenNearClass::begin(const OpenNearConfig &cfg) {
    if (started_) return true;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    s_sdk_cmd_cb  = cfg.cmd_cb;
    s_sdk_rule_cb = cfg.rule_cb;
    role_cb_      = cfg.role_cb;

    on_esp32_net_gw_set_user_cmd_cb(sdk_cmd_trampoline, nullptr);
    on_esp32_net_gw_set_user_rule_cb(sdk_rule_trampoline, nullptr);
    on_esp32_net_gw_set_wifi_portal_enabled(cfg.is_gateway);
    pinMode(0, INPUT_PULLUP); /* Onboard BOOT button: hold LOW 5s after boot for factory reset */

    on_esp32_net_gw_preinit_wifi(&s_sdk_gw, cfg.wifi_ssid, cfg.wifi_pass, cfg.cloud_url);

    on_esp32_cfg_t hcfg;
    hcfg.channel  = cfg.channel;
    hcfg.act_gpio = cfg.act_gpio;
#ifdef OPENNEAR_ENABLE_WIFI_GW
    hcfg.share_existing_wifi = cfg.is_gateway;
#else
    hcfg.share_existing_wifi = false;
#endif

    on_hal_ops_t hal;
    if (on_esp32_hal_init(&hcfg, &hal) != ON_OK) {
        return false;
    }

    on_node_init(&s_sdk_node, &hal,
                 cfg.is_gateway ? ON_ROLE_G : ON_ROLE_UNJOINED,
                 cfg.capabilities);
    on_mesh_init(&s_sdk_mesh,
                 cfg.is_gateway ? ON_SHORT_ID_G_DEFAULT : ON_SHORT_ID_UNASSIGNED,
                 cfg.is_gateway ? 0u : 0xFFu,
                 ON_SHORT_ID_UNASSIGNED);
    on_node_attach_mesh(&s_sdk_node, &s_sdk_mesh);
    on_node_enable_rssi_anchor_report(&s_sdk_node, true);

#ifdef OPENNEAR_ENABLE_OTA
    on_ota_storage_ops_t ota_ops;
    on_esp32_ota_storage_init(&ota_ops);
    on_ota_init(&s_sdk_ota, OPENNEAR_SDK_VERSION_HEX, 1u, &ota_ops);
    on_node_attach_ota(&s_sdk_node, &s_sdk_ota);
#endif

    on_esp32_net_gw_start(&s_sdk_node, &s_sdk_gw,
                          cfg.ble_name ? cfg.ble_name : (cfg.is_gateway ? "OpenNear-G-GW" : "OpenNear-T-Node"));
#ifdef OPENNEAR_ENABLE_DEMO_APP
    on_esp32_demo_init();
#endif

    if (cfg.leaderless_boot) {
        on_node_enable_leaderless_boot(&s_sdk_node, 1500u);
    }
    if (cfg.lr_mode > 0u) {
        on_node_set_long_range_mode(&s_sdk_node, cfg.lr_mode);
        on_esp32_hal_set_long_range_mode(cfg.lr_mode);
    }

    /* ESP32 queued-TX safe defaults */
    s_sdk_node.peer_lease_timeout_sf      = 250u;
    s_sdk_node.backup_miss_threshold      = 12u;
    s_sdk_node.backup_min_sync_silence_ns = 150000000LL;

    uint8_t active_key[ON_SEC_KEY_LEN];
    if (cfg.net_key16) {
        memcpy(active_key, cfg.net_key16, ON_SEC_KEY_LEN);
    } else if (!on_esp32_net_gw_load_netkey(active_key)) {
        memcpy(active_key, kSdkDefaultKey, ON_SEC_KEY_LEN);
    }
    on_node_enable_security(&s_sdk_node, active_key);
    if (cfg.is_gateway) {
        on_node_set_session_salt(&s_sdk_node, esp_random());
    }

    last_role_ = s_sdk_node.role;
    started_   = true;

    /* Initialize recursive mutex before spawning Core 1 real-time task */
    on_esp32_node_lock();
    on_esp32_node_unlock();

    xTaskCreatePinnedToCore(opennear_sdk_rt_task, "on_sdk_rt", 8192, nullptr,
                            configMAX_PRIORITIES - 2, nullptr, 1);
    return true;
}

void OpenNearClass::poll() {
    if (!started_) return;
    on_esp32_net_gw_poll(&s_sdk_node, &s_sdk_gw);
    /* Hardware factory reset: hold onboard BOOT button (GPIO0) LOW continuously for 5 seconds */
    static uint32_t s_boot_hold_start_ms = 0u;
    if (millis() > 3000u && digitalRead(0) == LOW) {
        if (s_boot_hold_start_ms == 0u) {
            s_boot_hold_start_ms = millis();
        } else if ((millis() - s_boot_hold_start_ms) >= 5000u) {
            on_esp32_net_gw_factory_reset();
        }
    } else {
        s_boot_hold_start_ms = 0u;
    }
#ifdef OPENNEAR_ENABLE_OTA
    if (!s_sdk_ota_boot_confirmed && millis() >= 10000u) {
        on_esp32_node_lock();
        bool healthy = (s_sdk_node.role == ON_ROLE_G) || (s_sdk_node.sync_rx_total >= 4u) || (millis() >= 30000u);
        on_esp32_node_unlock();
        if (healthy) {
            on_esp32_ota_confirm_running_app();
            s_sdk_ota_boot_confirmed = true;
        }
    }
#endif
    if (role_cb_) {
        on_esp32_node_lock();
        on_role_t cur = s_sdk_node.role;
        uint16_t  sid = s_sdk_node.short_id;
        on_esp32_node_unlock();
        if (cur != last_role_) {
            last_role_ = cur;
            role_cb_(cur, sid);
        }
    }
}

void OpenNearClass::sendTelemetry(const uint8_t *data, uint8_t len) {
    if (!started_ || !data || len == 0u) return;
    on_esp32_node_lock();
    on_node_set_tx_payload(&s_sdk_node, data, (len > 8u) ? 8u : len);
    on_esp32_node_unlock();
}

void OpenNearClass::sendTelemetry(const char *text) {
    if (!text) return;
    size_t n = strlen(text);
    sendTelemetry(reinterpret_cast<const uint8_t *>(text), static_cast<uint8_t>(n > 8u ? 8u : n));
}

bool OpenNearClass::sendCommand(uint16_t target_short_id, const char *payload) {
    if (!started_ || !payload || !payload[0]) return false;
    size_t n = strlen(payload);
    if (n > ON_GW_MAX_PAYLOAD_LEN) n = ON_GW_MAX_PAYLOAD_LEN;
    const uint8_t *pbytes = reinterpret_cast<const uint8_t *>(payload);
    on_err_t rc = ON_OK;
    on_esp32_node_lock();
    if (target_short_id != ON_SHORT_ID_BROADCAST && s_sdk_node.mesh &&
        on_mesh_lookup_next_hop(s_sdk_node.mesh, target_short_id) != ON_SHORT_ID_UNASSIGNED) {
        rc = on_node_send_mesh_downlink(&s_sdk_node, target_short_id, pbytes, static_cast<uint8_t>(n));
    } else {
        rc = on_node_send_gw_downlink(&s_sdk_node, target_short_id,
                                      ON_GW_MSG_CLOUD_CMD_DOWN, pbytes, static_cast<uint8_t>(n));
    }
    on_esp32_node_unlock();
    return rc == ON_OK;
}

bool OpenNearClass::sendTagCommand(const char *tag, const char *payload) {
    if (!started_ || !tag || !tag[0] || !payload || !payload[0]) return false;
    char buf[ON_GW_MAX_PAYLOAD_LEN + 1u];
    int n = snprintf(buf, sizeof(buf), "CMD_TAG:%s|%s", tag, payload);
    if (n <= 0 || static_cast<size_t>(n) >= sizeof(buf)) return false;
    return execControl(buf);
}

bool OpenNearClass::execControl(const char *control_text) {
    if (!started_ || !control_text || !control_text[0]) return false;
    char saved_tok[32];
    memcpy(saved_tok, s_sdk_gw.cfg.admin_token, sizeof(saved_tok));
    s_sdk_gw.cfg.admin_token[0] = '\0';
    on_err_t rc = on_gw_handle_control_text(&s_sdk_gw, control_text, strlen(control_text), false);
    if (!s_sdk_gw.cfg.admin_token_updated) {
        memcpy(s_sdk_gw.cfg.admin_token, saved_tok, sizeof(saved_tok));
    }
    return rc == ON_OK;
}

bool OpenNearClass::setFence(const char *name, int32_t x_cm, int32_t y_cm, uint16_t radius_cm) {
    if (!name || !name[0]) return false;
    char buf[96];
    int n = snprintf(buf, sizeof(buf), "FENCE:%s|%ld|%ld|%u",
                     name, (long)x_cm, (long)y_cm, (unsigned)radius_cm);
    return (n > 0 && static_cast<size_t>(n) < sizeof(buf)) && execControl(buf);
}

bool OpenNearClass::setRule(uint8_t rule_id, on_loc_rule_trig_t trig, const char *zone,
                            uint8_t csi_threshold, const char *tag, const char *cmd_payload) {
    const char *z = (zone && zone[0]) ? zone : "*";
    const char *t = (tag && tag[0])   ? tag  : "*";
    const char *c = (cmd_payload && cmd_payload[0]) ? cmd_payload : "NOP";
    char buf[112];
    int n = (trig == ON_RULE_TRIG_NONE)
            ? snprintf(buf, sizeof(buf), "RULE:%u|0", (unsigned)rule_id)
            : snprintf(buf, sizeof(buf), "RULE:%u|%u|%s|%u|%s|%s",
                       (unsigned)rule_id, (unsigned)trig, z, (unsigned)csi_threshold, t, c);
    return (n > 0 && static_cast<size_t>(n) < sizeof(buf)) && execControl(buf);
}

bool OpenNearClass::startShowcaseDemo() { return execControl("LOCDEMO:1"); }
bool OpenNearClass::stopShowcaseDemo()  { return execControl("LOCDEMO:0"); }

bool OpenNearClass::setLongRangeMode(uint8_t mode) {
    if (mode > 2u) mode = 2u;
    char buf[16];
    snprintf(buf, sizeof(buf), "LRMODE:%u", (unsigned)mode);
    return execControl(buf);
}

uint8_t OpenNearClass::longRangeMode() const {
    on_esp32_node_lock();
    uint8_t m = s_sdk_node.lr_mode;
    on_esp32_node_unlock();
    return m;
}

on_role_t OpenNearClass::role() const {
    on_esp32_node_lock();
    on_role_t r = s_sdk_node.role;
    on_esp32_node_unlock();
    return r;
}

uint16_t OpenNearClass::shortId() const {
    on_esp32_node_lock();
    uint16_t id = s_sdk_node.short_id;
    on_esp32_node_unlock();
    return id;
}

uint32_t OpenNearClass::superframeSeq() const {
    on_esp32_node_lock();
    uint32_t sf = s_sdk_node.current_sf_seq;
    on_esp32_node_unlock();
    return sf;
}

bool OpenNearClass::isPrimaryGateway() const {
    return role() == ON_ROLE_G;
}

on_node_t   *OpenNearClass::rawNode()    { return &s_sdk_node; }
on_gw_ctx_t *OpenNearClass::rawGateway() { return &s_sdk_gw; }

OpenNearClass OpenNear;

#endif /* ARDUINO */
