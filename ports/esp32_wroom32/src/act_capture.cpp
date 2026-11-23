#include "act_capture.h"

#ifdef OPENNEAR_ACT_CAPTURE
#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_timer.h"
#include "freertos/queue.h"

namespace {
constexpr auto kUnit = MCPWM_UNIT_0;
constexpr auto kPrimaryInput = GPIO_NUM_32;
constexpr auto kTerminalInput = GPIO_NUM_33;

struct Edge {
    uint32_t seq;
    uint32_t ticks;
    uint8_t channel;
    bool rising;
};

QueueHandle_t s_queue;
uint32_t s_seq; /* Only the capture interrupt writes this. */
uint32_t s_reported_hz;
int64_t s_last_status_us;
bool s_running;

bool capture_isr(mcpwm_unit_t, mcpwm_capture_channel_id_t channel,
                 const cap_event_data_t *event, void *) {
    const Edge edge{++s_seq, event->cap_value, static_cast<uint8_t>(channel),
                    event->cap_edge == MCPWM_POS_EDGE};
    BaseType_t wake = pdFALSE;
    xQueueSendFromISR(s_queue, &edge, &wake);
    /* Dropped records remain visible as gaps in sequence numbers. */
    return wake == pdTRUE;
}

esp_err_t init_input(mcpwm_io_signals_t signal, gpio_num_t pin) {
    esp_err_t err = mcpwm_gpio_init(kUnit, signal, pin);
    if (err == ESP_OK) err = gpio_set_direction(pin, GPIO_MODE_INPUT);
    if (err == ESP_OK) err = gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);
    return err;
}
}  // namespace

void on_esp32_act_capture_start() {
    s_queue = xQueueCreate(128, sizeof(Edge));
    esp_err_t err = s_queue ? ESP_OK : ESP_ERR_NO_MEM;
    if (err == ESP_OK) err = init_input(MCPWM_CAP_0, kPrimaryInput);
    if (err == ESP_OK) err = init_input(MCPWM_CAP_1, kTerminalInput);
    mcpwm_capture_config_t cfg{};
    cfg.cap_edge = MCPWM_BOTH_EDGE;
    cfg.cap_prescale = 1;
    cfg.capture_cb = capture_isr;
    if (err == ESP_OK) err = mcpwm_capture_enable_channel(kUnit, MCPWM_SELECT_CAP0, &cfg);
    if (err == ESP_OK) {
        err = mcpwm_capture_enable_channel(kUnit, MCPWM_SELECT_CAP1, &cfg);
        if (err != ESP_OK) mcpwm_capture_disable_channel(kUnit, MCPWM_SELECT_CAP0);
    }
    if (err != ESP_OK) {
        Serial.printf("[GPIOCAP] init error=%d\n", static_cast<int>(err));
        return;
    }
    s_reported_hz = getApbFrequency();
    s_running = true;
    Serial.printf("[GPIOCAP] hz=%lu p=32 t=33\n", static_cast<unsigned long>(s_reported_hz));
}

void on_esp32_act_capture_poll() {
    if (!s_running) {
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }
    Edge edge;
    if (xQueueReceive(s_queue, &edge, pdMS_TO_TICKS(20)) == pdTRUE) {
        Serial.printf("[GPIOCAP] %lu,%u,%c,%lu\n", static_cast<unsigned long>(edge.seq),
                      static_cast<unsigned>(edge.channel), edge.rising ? 'R' : 'F',
                      static_cast<unsigned long>(edge.ticks));
    }
    const int64_t now = esp_timer_get_time();
    if (now - s_last_status_us >= 1000000) {
        s_last_status_us = now;
        const uint32_t hz = getApbFrequency();
        if (hz != s_reported_hz) {
            Serial.println("[GPIOCAP] clock changed; capture invalid");
            s_running = false;
            mcpwm_capture_disable_channel(kUnit, MCPWM_SELECT_CAP0);
            mcpwm_capture_disable_channel(kUnit, MCPWM_SELECT_CAP1);
            return;
        }
        Serial.printf("[GPIOCAP] hz=%lu p=32 t=33\n", static_cast<unsigned long>(hz));
    }
}
#endif
