#include <Arduino.h>
#include "nvs_flash.h"
#include "esp_system.h"
#include "on_node.h"
#include "hal_esp32_raw80211.h"
#include "act_capture.h"
#include "esp32_ota_flash.h"
#include "esp32_net_gateway.h"

#ifndef OPENNEAR_IS_MASTER
#define OPENNEAR_IS_MASTER 0
#endif

#ifndef OPENNEAR_CAPS
#define OPENNEAR_CAPS 0x03u /* bit0: Backup_ID capable, bit1: Sub-G capable */
#endif

#ifndef OPENNEAR_FW_VERSION
#define OPENNEAR_FW_VERSION 0x00050000u /* v5.0.0 */
#endif

#ifndef OPENNEAR_WIFI_SSID
#define OPENNEAR_WIFI_SSID ""
#endif
#ifndef OPENNEAR_WIFI_PASS
#define OPENNEAR_WIFI_PASS ""
#endif
#ifndef OPENNEAR_CLOUD_URL
#define OPENNEAR_CLOUD_URL ""
#endif

static const uint8_t kDefaultNetMasterKey[ON_SEC_KEY_LEN] = {
    0x4F,0x50,0x45,0x4E,0x4E,0x45,0x41,0x52,
    0x33,0x30,0x2D,0x4B,0x45,0x59,0x21,0x01
};

static on_node_t g_node;
static on_mesh_ctx_t g_mesh;
#ifdef OPENNEAR_ENABLE_OTA
static on_ota_ctx_t g_ota;
static bool s_ota_boot_confirmed = false;
#endif
#if defined(OPENNEAR_ENABLE_WIFI_GW) || defined(OPENNEAR_ENABLE_BLE_GW)
static on_gw_ctx_t g_gw;
#endif
static constexpr uint8_t kWifiBackupMissThreshold = 12u;
static constexpr uint32_t kWifiBackupSilenceMs = 150u;
static constexpr uint32_t kWifiPeerLeaseTimeoutSf = 250u; /* 1 s at the default 4 ms SF */

static void opennear_rt_task(void *arg) {
    (void)arg;
    int64_t last_log_ns = 0;
    uint32_t loop_iters = 0;

    for (;;) {
        on_esp32_node_lock();
        on_esp32_hal_poll_rx(&g_node);
        int64_t now_ns = g_node.hal.now_ns(g_node.hal.user_ctx);
        on_node_step(&g_node, now_ns);
        const int64_t until_deadline_ns = on_node_next_wakeup_local_ns(&g_node, now_ns) - now_ns;
        const bool should_log = (now_ns - last_log_ns >= 1000000000LL) &&
                                (!g_node.sync.locked || until_deadline_ns > 600000LL);
        struct {
            uint32_t sf_seq, harq_retx, act_fire, sync_rx, tdma_tx, tdma_rx, missed, failover, rejoins, leases;
            int      role, hop;
            uint16_t short_id, backup_id;
            uint8_t  my_slot;
            double   rate_ppb;
            int64_t  link_delay_ns;
        } log_s;
        if (should_log) {
            log_s.sf_seq        = g_node.current_sf_seq;
            log_s.role          = (int)g_node.role;
            log_s.short_id      = g_node.short_id;
            log_s.hop           = (g_node.mesh && g_node.mesh->my_hop_depth != 0xFFu)
                                  ? (int)g_node.mesh->my_hop_depth
                                  : (g_node.is_sub_node ? 2 : (g_node.role == ON_ROLE_G ? 0 : 1));
            log_s.my_slot       = g_node.my_slot;
            log_s.rate_ppb      = g_node.sync.rate_ppb;
            log_s.link_delay_ns = g_node.sync.link_delay_ns;
            log_s.harq_retx     = g_node.harq_retx_total;
            log_s.act_fire      = g_node.act_fire_total;
            log_s.sync_rx       = g_node.sync_rx_total;
            log_s.tdma_tx       = g_node.tdma_tx_total;
            log_s.tdma_rx       = g_node.tdma_rx_total;
            log_s.missed        = g_node.missed_sync_count;
            log_s.failover      = g_node.failover_count;
            log_s.backup_id     = g_node.backup_id;
            log_s.rejoins       = g_node.rejoin_count;
            log_s.leases        = g_node.lease_reclaim_count;
        }
        on_esp32_node_unlock();

        /* Network SF can reset on a primary reboot; local time stays monotonic.
         * Defer 1 Hz UART formatting if an ACT/SYNC deadline is within 600 us. */
        if (should_log) {
            last_log_ns = now_ns;
            uint32_t tx_accepted, tx_failed;
            on_esp32_hal_tx_counts(&tx_accepted, &tx_failed);
            Serial.printf("[OpenNear v6.2] SF=%u role=%d id=0x%03X hop=%d slot=%u nco=%.2fppm rtt=%lldns harq=%u acts=%u\n",
                          (unsigned)log_s.sf_seq,
                          log_s.role,
                          (unsigned)log_s.short_id,
                          log_s.hop,
                          (unsigned)log_s.my_slot,
                          log_s.rate_ppb * 1e-3,
                          (long long)log_s.link_delay_ns,
                          (unsigned)log_s.harq_retx,
                          (unsigned)log_s.act_fire);
            Serial.printf("[OpenNear TX] accepted=%u failed=%u\n",
                          (unsigned)tx_accepted, (unsigned)tx_failed);
            Serial.printf("[OpenNear Link] sync_rx=%u data_tx=%u data_rx=%u missed=%u takeovers=%u backup=0x%03X rejoins=%u leases=%u\n",
                          (unsigned)log_s.sync_rx, (unsigned)log_s.tdma_tx,
                          (unsigned)log_s.tdma_rx, (unsigned)log_s.missed,
                          (unsigned)log_s.failover, (unsigned)log_s.backup_id,
                          (unsigned)log_s.rejoins,
                          (unsigned)log_s.leases);
        }
        /* A 1 ms tick delay near a deadline can skip the entire ACT window. */
        if ((++loop_iters & 0x3Fu) == 0u) {
            const int64_t tick_ns = 1000000000LL / configTICK_RATE_HZ;
            on_esp32_node_lock();
            const int64_t now = g_node.hal.now_ns(g_node.hal.user_ctx);
            const int64_t until_wakeup = on_node_next_wakeup_local_ns(&g_node, now) - now;
            const bool can_delay = !g_node.sync.locked || until_wakeup > tick_ns + 300000LL;
            on_esp32_node_unlock();
            if (can_delay) {
                vTaskDelay(1);
                continue;
            }
        }
        taskYIELD();
    }
}

void setup() {
    Serial.setTxBufferSize(1024);
    Serial.begin(115200);
    delay(200);
#ifdef OPENNEAR_ACT_CAPTURE
    on_esp32_act_capture_start();
#endif

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Optional hardware strap: pull GPIO4 LOW at boot to select Primary G coordinator */
    pinMode(ON_ESP32_MASTER_STRAP_PIN, INPUT_PULLUP);
    bool is_master = (OPENNEAR_IS_MASTER != 0) || (digitalRead(ON_ESP32_MASTER_STRAP_PIN) == LOW);

#if defined(OPENNEAR_ENABLE_WIFI_GW) || defined(OPENNEAR_ENABLE_BLE_GW)
    on_esp32_net_gw_preinit_wifi(&g_gw, OPENNEAR_WIFI_SSID, OPENNEAR_WIFI_PASS, OPENNEAR_CLOUD_URL);
#endif

    on_esp32_cfg_t cfg = {
        .channel             = ON_ESP32_DEFAULT_CHANNEL,
        .act_gpio            = ON_ESP32_ACT_LED_GPIO,
#ifdef OPENNEAR_ENABLE_WIFI_GW
        .share_existing_wifi = true
#else
        .share_existing_wifi = false
#endif
    };
    on_hal_ops_t hal;
    if (on_esp32_hal_init(&cfg, &hal) != ON_OK) {
        Serial.println("[OpenNear] HAL init failed!");
        return;
    }

#ifdef OPENNEAR_ACT_TRACE
    if (!on_esp32_hal_trace_init()) Serial.println("[ACTTRACE] queue allocation failed");
#endif
    on_node_init(&g_node, &hal,
                 is_master ? ON_ROLE_G : ON_ROLE_UNJOINED,
                 is_master ? 0x00u : (uint8_t)OPENNEAR_CAPS);
    on_mesh_init(&g_mesh,
                 is_master ? ON_SHORT_ID_G_DEFAULT : ON_SHORT_ID_UNASSIGNED,
                 is_master ? 0u : 0xFFu,
                 ON_SHORT_ID_UNASSIGNED);
    on_node_attach_mesh(&g_node, &g_mesh);
#ifdef OPENNEAR_ENABLE_OTA
    on_ota_storage_ops_t ota_ops;
    on_esp32_ota_storage_init(&ota_ops);
    on_ota_init(&g_ota, OPENNEAR_FW_VERSION, 1u /* ESP32 */, &ota_ops);
    on_node_attach_ota(&g_node, &g_ota);
    Serial.printf("[OpenNear OTA] A/B Dual-Partition OTA armed (fw=0x%08X, deferred boot-confirm)\n",
                  (unsigned)OPENNEAR_FW_VERSION);
#endif
#if defined(OPENNEAR_ENABLE_WIFI_GW) || defined(OPENNEAR_ENABLE_BLE_GW)
    on_esp32_net_gw_start(&g_node, &g_gw, is_master ? "OpenNear-G-GW" : "OpenNear-T-Node");
#endif
#ifdef OPENNEAR_ACT_LOOKAHEAD
    g_node.act_lookahead_enabled = true;
    Serial.println("[OpenNear ACT] Lookahead GPIO demo enabled");
#endif
#ifdef OPENNEAR_LEADERLESS_BOOT
    if (!is_master) {
        on_node_enable_leaderless_boot(&g_node, 1500u);
        Serial.println("[OpenNear Mesh] Leaderless cold-boot auto-election enabled (1500 ms)");
    }
#endif
#ifdef OPENNEAR_ACT_CLOCK_BIAS_US
    g_node.act_clock_enabled = true;
    g_node.act_clock_bias_us = OPENNEAR_ACT_CLOCK_BIAS_US;
    Serial.printf("[OpenNear ACT] Independent action clock, RX floor correction=%u us\n",
                  (unsigned)g_node.act_clock_bias_us);
#endif
    /* Raw Wi-Fi is contended; tolerate a short burst of delayed/missed beacons. */
    g_node.peer_lease_timeout_sf = kWifiPeerLeaseTimeoutSf;
    g_node.backup_miss_threshold = kWifiBackupMissThreshold;
    g_node.backup_min_sync_silence_ns = (int64_t)kWifiBackupSilenceMs * 1000000LL;

    /* Enable v3.0 AES-128-CCM + Beacon Anti-Spoofing with per-boot TRNG salt */
    on_node_enable_security(&g_node, kDefaultNetMasterKey);
    if (is_master) {
        on_node_set_session_salt(&g_node, esp_random());
    }

    Serial.printf("[OpenNear] ESP32-WROOM-32 booted as %s on Wi-Fi ch=%u (54M 0xD0, RTT off, backup after %u missed beacons and %u ms silence)\n",
                  is_master ? "PRIMARY G (0x001)" : "AUTO-JOIN TERMINAL",
                  (unsigned)cfg.channel, (unsigned)kWifiBackupMissThreshold,
                  (unsigned)kWifiBackupSilenceMs);

    /* Initialize recursive mutex before spawning Core 1 real-time task */
    on_esp32_node_lock();
    on_esp32_node_unlock();

    /* Pin OpenNear real-time state machine to Core 1 (isolating Wi-Fi ISR on Core 0) */
    xTaskCreatePinnedToCore(opennear_rt_task, "opennear_rt", 8192, NULL,
                            configMAX_PRIORITIES - 2, NULL, 1);
}

void loop() {
#ifdef OPENNEAR_ACT_TRACE
    on_esp32_act_trace_t act;
    while (on_esp32_hal_trace_read(&act)) {
        Serial.printf("[ACTTRACE] %u,%u,%u,%u,%u,%u,%lld,%lld,%lld,%lld,%u,%u,%lld,%lld,%lld,%lld,%u\n",
                      (unsigned)act.seq, (unsigned)act.act_sf, (unsigned)act.level,
                      (unsigned)act.role, (unsigned)act.sf_us, (unsigned)act.act_offset_us,
                      (long long)act.planned_us, (long long)act.step_us,
                      (long long)act.pre_write_us, (long long)act.post_write_us,
                      (unsigned)act.beacon_sf, (unsigned)act.rx_hw_us,
                      (long long)act.rx_mapped_us, (long long)act.rx_callback_us,
                      (long long)act.rx_dequeue_us, (long long)act.rx_accepted_us,
                      (unsigned)act.scheduled);
    }
#endif
#if defined(OPENNEAR_ENABLE_WIFI_GW) || defined(OPENNEAR_ENABLE_BLE_GW)
    on_esp32_net_gw_poll(&g_node, &g_gw);
#endif
#ifdef OPENNEAR_ENABLE_OTA
    /* Deferred A/B boot confirmation: only cancel rollback after >=10s healthy runtime + sync */
    if (!s_ota_boot_confirmed && millis() >= 10000u) {
        on_esp32_node_lock();
        bool healthy = (g_node.role == ON_ROLE_G) || (g_node.sync_rx_total >= 4u) || (millis() >= 30000u);
        on_esp32_node_unlock();
        if (healthy) {
            on_esp32_ota_confirm_running_app();
            s_ota_boot_confirmed = true;
        }
    }
#endif
    /* Interactive USB Serial query: type "TOPO?" or "DATA?" on ANY node to dump cluster JSON */
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.equalsIgnoreCase("TOPO?") || cmd.equalsIgnoreCase("DATA?")) {
            on_esp32_node_lock();
            if (g_node.role != ON_ROLE_G && !g_node.cluster_snapshot_valid) {
                on_node_request_cluster_snapshot(&g_node);
            }
            static char s_json[3072];
            size_t n = on_node_format_cluster_json(&g_node, cmd.equalsIgnoreCase("DATA?"),
                                                   s_json, sizeof(s_json));
            on_esp32_node_unlock();
            if (n > 0u) Serial.println(s_json);
        }
    }
#ifdef OPENNEAR_ACT_CAPTURE
    on_esp32_act_capture_poll();
#elif defined(OPENNEAR_ACT_TRACE)
    vTaskDelay(pdMS_TO_TICKS(10));
#elif defined(OPENNEAR_ENABLE_WIFI_GW) || defined(OPENNEAR_ENABLE_BLE_GW)
    vTaskDelay(pdMS_TO_TICKS(50));
#else
    vTaskDelay(pdMS_TO_TICKS(1000));
#endif
}
