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
#ifdef OPENNEAR_ENABLE_OTA
static on_ota_ctx_t g_ota;
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
        on_esp32_hal_poll_rx(&g_node);
        int64_t now_ns = g_node.hal.now_ns(g_node.hal.user_ctx);
        on_node_step(&g_node, now_ns);

        /* Network SF can reset on a primary reboot; local time stays monotonic.
         * Defer 1 Hz UART formatting if an ACT/SYNC deadline is within 600 us. */
        const int64_t until_deadline_ns = on_node_next_wakeup_local_ns(&g_node, now_ns) - now_ns;
        if (now_ns - last_log_ns >= 1000000000LL &&
            (!g_node.sync.locked || until_deadline_ns > 600000LL)) {
            last_log_ns = now_ns;
            uint32_t tx_accepted, tx_failed;
            on_esp32_hal_tx_counts(&tx_accepted, &tx_failed);
            Serial.printf("[OpenNear v3.0] SF=%u role=%d id=0x%03X hop=%d slot=%u nco=%.2fppm rtt=%lldns harq=%u acts=%u\n",
                          (unsigned)g_node.current_sf_seq,
                          (int)g_node.role,
                          (unsigned)g_node.short_id,
                          g_node.is_sub_node ? 2 : (g_node.role == ON_ROLE_G ? 0 : 1),
                          (unsigned)g_node.my_slot,
                          g_node.sync.rate_ppb * 1e-3,
                          (long long)g_node.sync.link_delay_ns,
                          (unsigned)g_node.harq_retx_total,
                          (unsigned)g_node.act_fire_total);
            Serial.printf("[OpenNear TX] accepted=%u failed=%u\n",
                          (unsigned)tx_accepted, (unsigned)tx_failed);
            Serial.printf("[OpenNear Link] sync_rx=%u data_tx=%u data_rx=%u missed=%u takeovers=%u backup=0x%03X rejoins=%u leases=%u\n",
                          (unsigned)g_node.sync_rx_total, (unsigned)g_node.tdma_tx_total,
                          (unsigned)g_node.tdma_rx_total, (unsigned)g_node.missed_sync_count,
                          (unsigned)g_node.failover_count, (unsigned)g_node.backup_id,
                          (unsigned)g_node.rejoin_count,
                          (unsigned)g_node.lease_reclaim_count);
        }
        /* A 1 ms tick delay near a deadline can skip the entire ACT window. */
        if ((++loop_iters & 0x3Fu) == 0u) {
            const int64_t tick_ns = 1000000000LL / configTICK_RATE_HZ;
            const int64_t now = g_node.hal.now_ns(g_node.hal.user_ctx);
            const int64_t until_wakeup = on_node_next_wakeup_local_ns(&g_node, now) - now;
            if (!g_node.sync.locked || until_wakeup > tick_ns + 300000LL) {
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
#ifdef OPENNEAR_ENABLE_OTA
    on_esp32_ota_confirm_running_app();
    on_ota_storage_ops_t ota_ops;
    on_esp32_ota_storage_init(&ota_ops);
    on_ota_init(&g_ota, OPENNEAR_FW_VERSION, 1u /* ESP32 */, &ota_ops);
    on_node_attach_ota(&g_node, &g_ota);
    Serial.printf("[OpenNear OTA] A/B Dual-Partition OTA armed (fw=0x%08X)\n",
                  (unsigned)OPENNEAR_FW_VERSION);
#endif
#if defined(OPENNEAR_ENABLE_WIFI_GW) || defined(OPENNEAR_ENABLE_BLE_GW)
    on_esp32_net_gw_start(&g_node, &g_gw, is_master ? "OpenNear-G-GW" : "OpenNear-T-Node");
#endif
#ifdef OPENNEAR_ACT_LOOKAHEAD
    g_node.act_lookahead_enabled = true;
    Serial.println("[OpenNear ACT] Lookahead GPIO demo enabled");
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
