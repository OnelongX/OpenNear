#include <Arduino.h>
#include "nvs_flash.h"
#include "esp_system.h"
#include "on_node.h"
#include "hal_esp32_raw80211.h"

#ifndef OPENNEAR_IS_MASTER
#define OPENNEAR_IS_MASTER 0
#endif

#ifndef OPENNEAR_CAPS
#define OPENNEAR_CAPS 0x03u /* bit0: Backup_ID capable, bit1: Sub-G capable */
#endif

static const uint8_t kDefaultNetMasterKey[ON_SEC_KEY_LEN] = {
    0x4F,0x50,0x45,0x4E,0x4E,0x45,0x41,0x52,
    0x33,0x30,0x2D,0x4B,0x45,0x59,0x21,0x01
};

static on_node_t g_node;

static void opennear_rt_task(void *arg) {
    (void)arg;
    uint32_t last_log_sf = 0;
    uint32_t loop_iters = 0;

    for (;;) {
        on_esp32_hal_poll_rx(&g_node);
        int64_t now_ns = g_node.hal.now_ns(g_node.hal.user_ctx);
        on_node_step(&g_node, now_ns);

        if (g_node.current_sf_seq >= last_log_sf + 250u) {
            last_log_sf = g_node.current_sf_seq;
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
        }
        /* Periodic safe yield to keep FreeRTOS idle task / TWDT healthy */
        if ((++loop_iters & 0x3Fu) == 0u) {
            vTaskDelay(1);
        } else {
            taskYIELD();
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Optional hardware strap: pull GPIO4 LOW at boot to select Primary G coordinator */
    pinMode(ON_ESP32_MASTER_STRAP_PIN, INPUT_PULLUP);
    bool is_master = (OPENNEAR_IS_MASTER != 0) || (digitalRead(ON_ESP32_MASTER_STRAP_PIN) == LOW);

    on_esp32_cfg_t cfg = {
        .channel  = ON_ESP32_DEFAULT_CHANNEL,
        .act_gpio = ON_ESP32_ACT_LED_GPIO
    };
    on_hal_ops_t hal;
    if (on_esp32_hal_init(&cfg, &hal) != ON_OK) {
        Serial.println("[OpenNear] HAL init failed!");
        return;
    }

    on_node_init(&g_node, &hal,
                 is_master ? ON_ROLE_G : ON_ROLE_UNJOINED,
                 is_master ? 0x00u : (uint8_t)OPENNEAR_CAPS);

    /* Enable v3.0 AES-128-CCM + Beacon Anti-Spoofing with per-boot TRNG salt */
    on_node_enable_security(&g_node, kDefaultNetMasterKey);
    if (is_master) {
        on_node_set_session_salt(&g_node, esp_random());
    }

    Serial.printf("[OpenNear] ESP32-WROOM-32 booted as %s on Wi-Fi ch=%u (54M 0xD0 raw carrier)\n",
                  is_master ? "PRIMARY G (0x001)" : "AUTO-JOIN TERMINAL",
                  (unsigned)cfg.channel);

    /* Pin OpenNear real-time state machine to Core 1 (isolating Wi-Fi ISR on Core 0) */
    xTaskCreatePinnedToCore(opennear_rt_task, "opennear_rt", 8192, NULL,
                            configMAX_PRIORITIES - 2, NULL, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
