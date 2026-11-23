#include "esp32_net_gateway.h"
#include "hal_esp32_raw80211.h"
#include "esp32_ota_flash.h"
#include "on_mqtt.h"
#include <Arduino.h>
#include <string.h>

#ifdef OPENNEAR_ENABLE_WIFI_GW
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#endif

#ifdef OPENNEAR_ENABLE_BLE_GW
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif

static on_gw_ctx_t      *s_gw_ref     = nullptr;
static on_node_t        *s_node_ref   = nullptr;
static SemaphoreHandle_t s_node_mutex = nullptr;

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

#ifdef OPENNEAR_ENABLE_WIFI_GW
static Preferences s_prefs;
static WebServer   s_http_srv(80);
static WiFiClient  s_mqtt_tcp;
static bool        s_http_srv_started   = false;
static bool        s_wifi_was_connected = false;
static uint32_t    s_last_wifi_retry_ms = 0u;
static uint32_t    s_last_mqtt_retry_ms = 0u;
static uint32_t    s_last_mqtt_ping_ms  = 0u;
static uint32_t    s_last_ha_pub_ms     = 0u;

static on_err_t esp32_mqtt_publish_cb(void *ctx, const char *topic,
                                      const char *payload, bool retain) {
    (void)ctx;
    if (!s_mqtt_tcp.connected() || !topic || !payload) return ON_ERR_NOT_READY;
    static uint8_t pkt[640];
    size_t n = on_mqtt_encode_publish(topic, reinterpret_cast<const uint8_t *>(payload),
                                      strlen(payload), retain, pkt, sizeof(pkt));
    if (n == 0u) return ON_ERR_TOO_LONG;
    return (s_mqtt_tcp.write(pkt, n) == n) ? ON_OK : ON_ERR_NOT_READY;
}

static void esp32_mqtt_poll(on_node_t *node, on_gw_ctx_t *gw) {
    if (!gw || WiFi.status() != WL_CONNECTED || gw->cfg.mqtt_host[0] == '\0') {
        if (gw) gw->mqtt_connected = false;
        return;
    }
    if (gw->cfg.mqtt_creds_updated) {
        gw->cfg.mqtt_creds_updated = false;
        if (s_mqtt_tcp.connected()) s_mqtt_tcp.stop();
        if (s_prefs.begin("opennear_gw", false)) {
            s_prefs.putString("mhost", gw->cfg.mqtt_host);
            s_prefs.putUShort("mport", gw->cfg.mqtt_port);
            s_prefs.putString("muser", gw->cfg.mqtt_user);
            s_prefs.putString("mpass", gw->cfg.mqtt_pass);
            s_prefs.end();
        }
    }

    uint32_t now_ms = millis();
    if (!s_mqtt_tcp.connected()) {
        gw->mqtt_connected = false;
        if ((now_ms - s_last_mqtt_retry_ms) < 5000u) return;
        s_last_mqtt_retry_ms = now_ms;
        uint16_t port = gw->cfg.mqtt_port ? gw->cfg.mqtt_port : 1883u;
        if (s_mqtt_tcp.connect(gw->cfg.mqtt_host, port)) {
            uint8_t cbuf[192];
            size_t clen = on_mqtt_encode_connect("opennear_gw", gw->cfg.mqtt_user,
                                                 gw->cfg.mqtt_pass, 30u, cbuf, sizeof(cbuf));
            if (clen > 0u && s_mqtt_tcp.write(cbuf, clen) == clen) {
                uint8_t sbuf[64];
                size_t s1 = on_mqtt_encode_subscribe(1u, "opennear/cmd/#", sbuf, sizeof(sbuf));
                if (s1 > 0u) s_mqtt_tcp.write(sbuf, s1);
                size_t s2 = on_mqtt_encode_subscribe(2u, "opennear/ota/trigger", sbuf, sizeof(sbuf));
                if (s2 > 0u) s_mqtt_tcp.write(sbuf, s2);
                gw->mqtt_connected  = true;
                s_last_mqtt_ping_ms = now_ms;
                s_last_ha_pub_ms    = 0u;
                Serial.printf("[OpenNear MQTT] Connected to %s:%u (HA Discovery + cmd/# armed)\n",
                              gw->cfg.mqtt_host, (unsigned)port);
            }
        }
        return;
    }

    /* Keepalive PINGREQ every 20 s */
    if ((now_ms - s_last_mqtt_ping_ms) > 20000u) {
        s_last_mqtt_ping_ms = now_ms;
        uint8_t ping[2];
        on_mqtt_encode_pingreq(ping);
        s_mqtt_tcp.write(ping, 2u);
    }

    /* Periodically publish Home Assistant MQTT Auto-Discovery + full cluster topology every 15 s */
    if (node && (s_last_ha_pub_ms == 0u || (now_ms - s_last_ha_pub_ms) > 15000u)) {
        s_last_ha_pub_ms = now_ms;
        on_cluster_snapshot_t snap;
        on_node_get_cluster_snapshot(node, &snap);
        static char topic[96];
        static char payload[512];
        for (uint8_t i = 0u; i < snap.active_count; ++i) {
            const on_cluster_node_t *cn = &snap.nodes[i];
            const char *rname = (cn->role == ON_ROLE_G) ? "G" :
                                (cn->role == ON_ROLE_SUB_G ? "SUB_G" : "T");
            if (on_mqtt_format_ha_discovery(cn->short_id, cn->hop_depth, rname,
                                            topic, sizeof(topic), payload, sizeof(payload))) {
                esp32_mqtt_publish_cb(gw, topic, payload, true);
            }
            if (on_mqtt_format_node_state(cn->short_id, cn->parent_id, cn->hop_depth,
                                          cn->last_skew_ns, cn->last_data, cn->last_data_len,
                                          topic, sizeof(topic), payload, sizeof(payload))) {
                esp32_mqtt_publish_cb(gw, topic, payload, false);
            }
        }
    }

    /* Drain inbound MQTT PUBLISH packets (Home Assistant / Cloud commands & OTA triggers) */
    if (s_mqtt_tcp.available() >= 4) {
        static uint8_t rxbuf[384];
        int n = s_mqtt_tcp.read(rxbuf, sizeof(rxbuf));
        if (n >= 4) {
            on_mqtt_pub_view_t pub;
            if (on_mqtt_decode_publish(rxbuf, static_cast<size_t>(n), &pub)) {
                uint16_t target_id = 0u;
                bool is_ota = false;
                if (on_mqtt_parse_cmd_topic(pub.topic, pub.topic_len, &target_id, &is_ota)) {
                    if (is_ota) {
                        char cmd_txt[192];
                        int cl = snprintf(cmd_txt, sizeof(cmd_txt), "OTA:%.*s",
                                          static_cast<int>(pub.payload_len),
                                          reinterpret_cast<const char *>(pub.payload));
                        if (cl > 4) on_gw_handle_control_text(gw, cmd_txt, static_cast<size_t>(cl), false);
                    } else if (node) {
                        uint8_t plen = (pub.payload_len <= ON_GW_MAX_PAYLOAD_LEN)
                                       ? static_cast<uint8_t>(pub.payload_len) : ON_GW_MAX_PAYLOAD_LEN;
                        on_esp32_node_lock();
                        if (node->mesh &&
                            on_mesh_lookup_next_hop(node->mesh, target_id) != ON_SHORT_ID_UNASSIGNED) {
                            on_node_send_mesh_downlink(node, target_id, pub.payload, plen);
                        } else {
                            on_node_send_gw_downlink(node, target_id, ON_GW_MSG_CLOUD_CMD_DOWN,
                                                     pub.payload, plen);
                        }
                        on_esp32_node_unlock();
                    }
                }
            }
        }
    }
}

static void start_local_http_api(void) {
    if (s_http_srv_started) return;
    s_http_srv.on("/api/topology", HTTP_GET, []() {
        static char json[2048];
        if (!s_node_ref) { s_http_srv.send(503, "application/json", "{\"error\":\"no_node\"}"); return; }
        on_esp32_node_lock();
        size_t n = on_node_format_cluster_json(s_node_ref, false, json, sizeof(json));
        on_esp32_node_unlock();
        s_http_srv.send(200, "application/json", (n > 0u) ? json : "{}");
    });
    s_http_srv.on("/api/data", HTTP_GET, []() {
        static char json[2048];
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
        /* Support X-OpenNear-Token header for HTTP /api/cmd when admin_token is configured */
        if (s_gw_ref->cfg.admin_token[0] != '\0' && !body.startsWith("AUTH:")) {
            if (!s_http_srv.hasHeader("X-OpenNear-Token") ||
                s_http_srv.header("X-OpenNear-Token") != String(s_gw_ref->cfg.admin_token)) {
                s_http_srv.send(401, "application/json", "{\"ok\":false,\"error\":\"unauthorized\"}");
                return;
            }
            body = String("AUTH:") + String(s_gw_ref->cfg.admin_token) + "|" + body;
        }
        on_err_t rc = on_gw_handle_control_text(s_gw_ref, body.c_str(), body.length(), false);
        s_http_srv.send((rc == ON_OK) ? 200 : 400, "application/json",
                        (rc == ON_OK) ? "{\"ok\":true}" : "{\"ok\":false}");
    });
    const char *hdr_keys[1] = { "X-OpenNear-Token" };
    s_http_srv.collectHeaders(hdr_keys, 1);
    s_http_srv.begin();
    s_http_srv_started = true;
    Serial.println("[OpenNear GW] HTTP API server started: GET /api/topology, GET /api/data, POST /api/cmd");
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
    auto *gw = static_cast<on_gw_ctx_t *>(ctx);
    if (!ssid || !ssid[0]) return ON_ERR_INVALID_ARG;
    save_wifi_nvs(ssid, pass ? pass : "", gw ? gw->cfg.cloud_url : "");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, pass ? pass : "");
    Serial.printf("[OpenNear GW] Connecting to public Wi-Fi SSID='%s'...\n", ssid);
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
    if (!http.begin(url)) return ON_ERR_NOT_READY;
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

    HTTPClient http;
    http.setConnectTimeout(3000);
    http.setTimeout(5000);
    if (!http.begin(gw->cfg.ota_url)) {
        Serial.println("[OpenNear OTA] HTTP begin failed");
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

class OpenNearBleServerCb : public BLEServerCallbacks {
    void onConnect(BLEServer *srv) override {
        (void)srv;
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
        String nvs_tok   = s_prefs.getString("atoken", "");
        s_prefs.end();
        if (nvs_ssid.length() > 0)  ssid = nvs_ssid;
        if (nvs_pass.length() > 0)  pass = nvs_pass;
        if (nvs_url.length() > 0)   url  = nvs_url;
        if (nvs_mhost.length() > 0) {
            strncpy(gw->cfg.mqtt_host, nvs_mhost.c_str(), sizeof(gw->cfg.mqtt_host) - 1u);
            gw->cfg.mqtt_port = mport;
            strncpy(gw->cfg.mqtt_user, nvs_muser.c_str(), sizeof(gw->cfg.mqtt_user) - 1u);
            strncpy(gw->cfg.mqtt_pass, nvs_mpass.c_str(), sizeof(gw->cfg.mqtt_pass) - 1u);
        }
        if (nvs_tok.length() > 0) {
            on_gw_set_admin_token(gw, nvs_tok.c_str());
        }
    }
#ifdef OPENNEAR_ADMIN_TOKEN
    if (gw->cfg.admin_token[0] == '\0') {
        on_gw_set_admin_token(gw, OPENNEAR_ADMIN_TOKEN);
    }
#endif

    on_gw_set_wifi_creds(gw, ssid.c_str(), pass.c_str(), url.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    if (ssid.length() > 0) {
        WiFi.begin(ssid.c_str(), pass.c_str());
        Serial.printf("[OpenNear GW] Wi-Fi STA starting (SSID='%s')\n", ssid.c_str());
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
#ifdef OPENNEAR_ENABLE_BLE_GW
    BLEDevice::init(ble_name && ble_name[0] ? ble_name : "OpenNear-GW");
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
    Serial.printf("[OpenNear BLE] GATT Bridge advertising as '%s' (Svc 0xFFE0)\n",
                  ble_name && ble_name[0] ? ble_name : "OpenNear-GW");
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
        Serial.printf("[OpenNear GW] Wi-Fi ONLINE ip=%u.%u.%u.%u ch=%u rssi=%d\n",
                      ip4[0], ip4[1], ip4[2], ip4[3], (unsigned)ch, (int)WiFi.RSSI());
    } else if (!connected && s_wifi_was_connected) {
        s_wifi_was_connected = false;
        on_gw_on_wifi_disconnected(gw);
        Serial.println("[OpenNear GW] Wi-Fi disconnected, retrying...");
    } else if (!connected && gw->cfg.wifi_ssid[0] != '\0' &&
               (millis() - s_last_wifi_retry_ms) > 10000u) {
        s_last_wifi_retry_ms = millis();
        WiFi.reconnect();
    }
    if (s_http_srv_started) {
        s_http_srv.handleClient();
    }
    esp32_mqtt_poll(node, gw);

    if (connected && gw->cfg.ota_pull_requested) {
        gw->cfg.ota_pull_requested = false;
        esp32_pull_http_ota(node, gw);
    }

    /* Auto-commit staged cluster OTA once all target nodes verify & report READY */
    if (node && node->role == ON_ROLE_G && node->ota &&
        node->ota->tx.active && !node->ota->tx.commit_sent &&
        on_ota_tx_all_targets_ready(&node->ota->tx, 1u)) {
        on_esp32_node_lock();
        on_node_ota_commit(node, 8u, ON_OTA_FLAG_AUTO_REBOOT);
        on_esp32_node_unlock();
        Serial.println("[OpenNear OTA] All targets READY -> scheduled synchronized cluster commit (+8 SFs)");
    }
#else
    (void)node;
#endif

    if (node && gw->cfg.topo_query_requested) {
        gw->cfg.topo_query_requested = false;
        if (node->role != ON_ROLE_G && !node->cluster_snapshot_valid) {
            on_node_request_cluster_snapshot(node);
        }
        static char topo_json[1024];
        size_t jlen = on_node_format_cluster_json(node, gw->cfg.topo_include_data,
                                                  topo_json, sizeof(topo_json));
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
