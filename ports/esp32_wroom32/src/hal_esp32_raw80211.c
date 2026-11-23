#include "hal_esp32_raw80211.h"

#include <string.h>
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#ifdef OPENNEAR_ACT_TRACE
#include "freertos/queue.h"
#endif

/* 802.11 Vendor-Specific Action (0xD0) carrier header:
 * 24-byte 802.11 MAC header + 1-byte Category (0x7F) + 3-byte OpenNear OUI ("ON\x01") = 28 bytes */
#define WIFI_ACTION_HDR_LEN   24u
#define ON_VENDOR_TAG_LEN     4u
#define ON_CARRIER_HDR_LEN    (WIFI_ACTION_HDR_LEN + ON_VENDOR_TAG_LEN)
#define ON_RX_RING_SIZE       16u

static const uint8_t kOpenNearVendorTag[ON_VENDOR_TAG_LEN] = { 0x7Fu, 0x4Fu, 0x4Eu, 0x01u };

typedef struct {
    uint16_t len;
    int64_t  rx_local_ns;
#ifdef OPENNEAR_ACT_TRACE
    int64_t rx_callback_us;
    uint32_t rx_hw_us;
#endif
    uint8_t  data[ON_MAX_FRAME_LEN];
} esp32_rx_slot_t;

static esp32_rx_slot_t  s_rx_ring[ON_RX_RING_SIZE];
static volatile uint8_t s_rx_head = 0u;
static volatile uint8_t s_rx_tail = 0u;

static uint8_t          s_mac48[6];
static int              s_act_gpio = ON_ESP32_ACT_LED_GPIO;
static esp_timer_handle_t s_act_timer = NULL;
static volatile uint8_t s_pending_act_level = 0u;
static uint32_t s_tx_accepted = 0u;
static uint32_t s_tx_failed   = 0u;

#ifdef OPENNEAR_ACT_TRACE
/* Single producer: OpenNear RT task. Serial drains from the separate loop task. */
static QueueHandle_t s_trace_queue;
static on_node_t *s_trace_node;
static on_esp32_act_trace_t s_trace_last_beacon;
static uint32_t s_trace_seq;
static uint8_t s_trace_level;

bool on_esp32_hal_trace_init(void) {
    s_trace_queue = xQueueCreate(96, sizeof(on_esp32_act_trace_t));
    return s_trace_queue != NULL;
}

bool on_esp32_hal_trace_read(on_esp32_act_trace_t *trace) {
    return s_trace_queue && xQueueReceive(s_trace_queue, trace, 0) == pdTRUE;
}

static void trace_act(uint32_t sf_seq, uint8_t level, int64_t step_us,
                      int64_t pre_write_us, int64_t post_write_us) {
    const on_node_t *node = s_trace_node;
    on_esp32_act_trace_t trace = s_trace_last_beacon;
    trace.seq = ++s_trace_seq;
    trace.act_sf = sf_seq;
    trace.level = level;
    trace.role = (uint8_t)node->role;
    trace.sf_us = node->sf_cfg.t_sf_us;
    trace.act_offset_us = node->sf_cfg.act_fire_off_us;
    int64_t target_net_ns = node->current_sf_start_net_ns + 1000LL * trace.act_offset_us;
    trace.scheduled = node->last_act_was_scheduled;
    trace.planned_us = (trace.scheduled ? node->last_act_deadline_local_ns
                       : on_sync_net_to_local(&node->sync, target_net_ns)) / 1000LL;
    trace.step_us = step_us;
    trace.pre_write_us = pre_write_us;
    trace.post_write_us = post_write_us;
    xQueueSend(s_trace_queue, &trace, 0);
}
#endif

static void IRAM_ATTR act_timer_cb(void *arg) {
    (void)arg;
    gpio_set_level((gpio_num_t)s_act_gpio, s_pending_act_level ? 1 : 0);
}

/* Safe 32-bit 1 us rx_ctrl.timestamp -> 64-bit monotonic nanosecond unwrapper */
static portMUX_TYPE     s_ts_mux = portMUX_INITIALIZER_UNLOCKED;
static bool             s_ts_inited = false;
static uint32_t         s_last_ts32_us = 0u;
static uint64_t         s_ts_high_us = 0u;
static int64_t          s_rx_to_esp_offset_us = 0;

static int64_t unwrap_rx_timestamp_ns(uint32_t rx_ts32_us) {
    int64_t now_esp_us = esp_timer_get_time();
    uint64_t full_rx_us;

    portENTER_CRITICAL_ISR(&s_ts_mux);
    if (!s_ts_inited) {
        s_ts_inited = true;
        s_last_ts32_us = rx_ts32_us;
        s_ts_high_us = 0u;
        s_rx_to_esp_offset_us = now_esp_us - (int64_t)rx_ts32_us;
    } else {
        /* Handle 71.5-minute 32-bit unsigned rollover cleanly */
        if (rx_ts32_us < s_last_ts32_us && (s_last_ts32_us - rx_ts32_us) > 0x80000000UL) {
            s_ts_high_us += (1ULL << 32);
        }
        s_last_ts32_us = rx_ts32_us;
    }
    full_rx_us = s_ts_high_us + (uint64_t)rx_ts32_us;
    int64_t aligned_us = (int64_t)full_rx_us + s_rx_to_esp_offset_us;
    /*
     * Guard against baseband TSF counter jump (> 2 ms divergence from esp_timer),
     * and track minimum callback latency offset when a faster callback path is observed.
     */
    int64_t drift_us = now_esp_us - aligned_us;
    if (drift_us > 2000LL || drift_us < -2000LL) {
        s_rx_to_esp_offset_us = now_esp_us - (int64_t)full_rx_us;
        aligned_us = now_esp_us;
    } else if (drift_us < 0) {
        s_rx_to_esp_offset_us = now_esp_us - (int64_t)full_rx_us;
        aligned_us = now_esp_us;
    }
    portEXIT_CRITICAL_ISR(&s_ts_mux);

    return aligned_us * 1000LL;
}

static void IRAM_ATTR promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT || !buf) return;

    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    uint32_t sig_len = pkt->rx_ctrl.sig_len;
    if (sig_len <= ON_CARRIER_HDR_LEN + 4u) return; /* exclude 4-byte 802.11 FCS */

    const uint8_t *p = pkt->payload;
    /* Check 802.11 Management Action subtype (0xD0) + OpenNear Vendor Tag */
    if (p[0] != 0xD0u) return;
    if (memcmp(p + WIFI_ACTION_HDR_LEN, kOpenNearVendorTag, ON_VENDOR_TAG_LEN) != 0) return;

    uint32_t on_len = sig_len - ON_CARRIER_HDR_LEN - 4u;
    if (on_len < ON_MAC_HDR_LEN + ON_PAYLOAD_CRC_LEN || on_len > ON_MAX_FRAME_LEN) return;

    uint8_t next_head = (uint8_t)((s_rx_head + 1u) % ON_RX_RING_SIZE);
    if (next_head == s_rx_tail) return; /* ring full */

    s_rx_ring[s_rx_head].len = (uint16_t)on_len;
#ifdef OPENNEAR_ACT_TRACE
    s_rx_ring[s_rx_head].rx_callback_us = esp_timer_get_time();
    s_rx_ring[s_rx_head].rx_hw_us = pkt->rx_ctrl.timestamp;
#endif
    s_rx_ring[s_rx_head].rx_local_ns = unwrap_rx_timestamp_ns(pkt->rx_ctrl.timestamp);
    memcpy(s_rx_ring[s_rx_head].data, p + ON_CARRIER_HDR_LEN, on_len);
    s_rx_head = next_head;
}

static int64_t esp32_now_ns(void *ctx) {
    (void)ctx;
    return esp_timer_get_time() * 1000LL;
}

static on_err_t esp32_tx_raw(void *ctx, const uint8_t *frame, size_t len, int64_t tx_at_local_ns) {
    (void)ctx;
    if (!frame || len > ON_MAX_FRAME_LEN) return ON_ERR_INVALID_ARG;

    /* Busy-spin micro-align if tx_at_local_ns is slightly ahead (< 500 us) */
    if (tx_at_local_ns > 0) {
        int64_t target_us = tx_at_local_ns / 1000LL;
        int64_t now_us = esp_timer_get_time();
        if (target_us > now_us && (target_us - now_us) < 500LL) {
            while (esp_timer_get_time() < target_us) {
                /* spin for sub-microsecond TX launch alignment */
            }
        }
    }

    uint8_t raw80211[ON_CARRIER_HDR_LEN + ON_MAX_FRAME_LEN];
    memset(raw80211, 0, ON_CARRIER_HDR_LEN);

    /* 802.11 Management Action Frame (0xD0) Broadcast */
    raw80211[0] = 0xD0u; /* Frame Control: Management / Action */
    raw80211[1] = 0x00u;
    memset(raw80211 + 4, 0xFF, 6u);           /* Addr1: Broadcast DA */
    memcpy(raw80211 + 10, s_mac48, 6u);       /* Addr2: Source SA */
    memset(raw80211 + 16, 0xFF, 6u);          /* Addr3: Broadcast BSSID */
    memcpy(raw80211 + WIFI_ACTION_HDR_LEN, kOpenNearVendorTag, ON_VENDOR_TAG_LEN);
    memcpy(raw80211 + ON_CARRIER_HDR_LEN, frame, len);

    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, raw80211,
                                      (int)(ON_CARRIER_HDR_LEN + len), false);
    if (err == ESP_OK) {
        ++s_tx_accepted;
    } else {
        ++s_tx_failed;
    }
    /* Driver acceptance does not reveal antenna departure time. */
    return (err == ESP_OK) ? ON_OK : ON_ERR_NOT_READY;
}

static void esp32_radio_set_rx(void *ctx, bool enable) {
    (void)ctx;
    (void)enable;
}

static void esp32_radio_sleep(void *ctx) {
    (void)ctx;
}

static void esp32_radio_set_channel(void *ctx, uint8_t ch_idx) {
    (void)ctx;
    /* Map 16 logical TSCH channels across non-overlapping 2.4 GHz Wi-Fi channels 1, 6, 11 */
    static const uint8_t kWifiCh[3] = { 1u, 6u, 11u };
    esp_wifi_set_channel(kWifiCh[ch_idx % 3u], WIFI_SECOND_CHAN_NONE);
}

static void esp32_on_act_trigger(void *ctx, uint32_t sf_seq, uint8_t act_cmd, int64_t local_ns) {
    (void)ctx;
    (void)sf_seq;
    s_pending_act_level = (act_cmd & 0x01u);
    int64_t now_us = esp_timer_get_time();
    int64_t target_us = local_ns / 1000LL;
    if (s_act_timer && target_us > now_us + 5LL && (target_us - now_us) < 2000LL) {
        /* Zero-task-jitter hardware esp_timer one-shot callback */
        esp_timer_stop(s_act_timer);
        esp_timer_start_once(s_act_timer, (uint64_t)(target_us - now_us));
    } else {
#ifdef OPENNEAR_ACT_TRACE
        if (s_trace_queue && s_trace_node && s_pending_act_level != s_trace_level) {
            int64_t pre_write_us = esp_timer_get_time();
            gpio_set_level((gpio_num_t)s_act_gpio, s_pending_act_level ? 1 : 0);
            int64_t post_write_us = esp_timer_get_time();
            s_trace_level = s_pending_act_level;
            trace_act(sf_seq, s_trace_level, target_us, pre_write_us, post_write_us);
            return;
        }
#endif
        gpio_set_level((gpio_num_t)s_act_gpio, s_pending_act_level ? 1 : 0);
    }
}

static void esp32_get_mac48(void *ctx, uint8_t mac_out[6]) {
    (void)ctx;
    memcpy(mac_out, s_mac48, 6u);
}

on_err_t on_esp32_hal_init(const on_esp32_cfg_t *cfg, on_hal_ops_t *hal_out) {
    if (!hal_out) return ON_ERR_INVALID_ARG;
    uint8_t ch = (cfg && cfg->channel >= 1u && cfg->channel <= 13u)
               ? cfg->channel : ON_ESP32_DEFAULT_CHANNEL;
    s_act_gpio = (cfg && cfg->act_gpio >= 0) ? cfg->act_gpio : ON_ESP32_ACT_LED_GPIO;

    /* Configure synchronized ACT LED GPIO + hardware microsecond esp_timer */
    gpio_config_t io = {0};
    io.pin_bit_mask = (1ULL << s_act_gpio);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level((gpio_num_t)s_act_gpio, 0);

    esp_timer_create_args_t targs = {0};
    targs.callback = act_timer_cb;
    targs.name     = "on_act";
    esp_timer_create(&targs, &s_act_timer);

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&wcfg) != ESP_OK) return ON_ERR_NOT_READY;
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);

    /* Lock raw 802.11 TX PHY rate to 54 Mbps BEFORE esp_wifi_start() */
    esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);

    if (esp_wifi_start() != ESP_OK) return ON_ERR_NOT_READY;
    if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) return ON_ERR_NOT_READY;
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    esp_wifi_get_mac(WIFI_IF_STA, s_mac48);

    wifi_promiscuous_filter_t filt;
    filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);
    esp_wifi_set_promiscuous(true);

    memset(hal_out, 0, sizeof(*hal_out));
    hal_out->now_ns         = esp32_now_ns;
    hal_out->tx_raw         = esp32_tx_raw;
    hal_out->radio_set_rx   = esp32_radio_set_rx;
    hal_out->radio_sleep    = esp32_radio_sleep;
    hal_out->radio_set_channel = esp32_radio_set_channel;
    hal_out->on_act_trigger = esp32_on_act_trigger;
    hal_out->get_mac48      = esp32_get_mac48;
    hal_out->tx_timing_best_effort = true;
    return ON_OK;
}

void on_esp32_hal_tx_counts(uint32_t *accepted, uint32_t *failed) {
    *accepted = s_tx_accepted;
    *failed   = s_tx_failed;
}

void on_esp32_hal_poll_rx(on_node_t *node) {
#ifdef OPENNEAR_ACT_TRACE
    s_trace_node = node;
#endif
    while (s_rx_tail != s_rx_head) {
        const esp32_rx_slot_t *slot = &s_rx_ring[s_rx_tail];
#ifdef OPENNEAR_ACT_TRACE
        uint32_t sync_before = node->sync_rx_total;
        int64_t dequeued_us = esp_timer_get_time();
#endif
        on_node_on_rx_raw(node, slot->data, slot->len, slot->rx_local_ns);
#ifdef OPENNEAR_ACT_TRACE
        if (node->sync_rx_total != sync_before) {
            s_trace_last_beacon.rx_accepted_us = esp_timer_get_time();
            s_trace_last_beacon.beacon_sf = node->current_sf_seq;
            s_trace_last_beacon.rx_hw_us = slot->rx_hw_us;
            s_trace_last_beacon.rx_mapped_us = slot->rx_local_ns / 1000LL;
            s_trace_last_beacon.rx_callback_us = slot->rx_callback_us;
            s_trace_last_beacon.rx_dequeue_us = dequeued_us;
        }
#endif
        s_rx_tail = (uint8_t)((s_rx_tail + 1u) % ON_RX_RING_SIZE);
    }
}
