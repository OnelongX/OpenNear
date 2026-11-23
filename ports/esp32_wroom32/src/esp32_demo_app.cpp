#include "esp32_demo_app.h"
#include "esp32_net_gateway.h"

#ifdef OPENNEAR_ENABLE_DEMO_APP
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    DEMO_MODE_SYNC   = 0, /* Standard 1 Hz superframe-locked sync */
    DEMO_MODE_STROBE = 1, /* 8 Hz high-speed synchronized flash */
    DEMO_MODE_PULSE  = 2  /* Global sf_seq double-beat rhythm */
} demo_mode_t;

static volatile uint8_t  s_demo_mode      = DEMO_MODE_SYNC;
static volatile uint8_t  s_relay_state    = 0u;
static volatile uint32_t s_ping_until_ms  = 0u;
static uint32_t          s_last_telem_ms  = 0u;

static char demo_mode_char(uint8_t m) {
    switch (m) {
        case DEMO_MODE_STROBE: return 'F'; /* Fast Strobe */
        case DEMO_MODE_PULSE:  return 'P'; /* Pulse Rhythm */
        default:               return 'S'; /* Sync */
    }
}

void on_esp32_demo_init(void) {
    pinMode(OPENNEAR_DEMO_RELAY_GPIO, OUTPUT);
    digitalWrite(OPENNEAR_DEMO_RELAY_GPIO, LOW);
    analogReadResolution(12);
    Serial.printf("[OpenNear Demo] Sync Showcase armed (relay=GPIO%d adc=GPIO%d, cmds: MODE:SYNC|STROBE|PULSE, RELAY:1|0, PING)\n",
                  (int)OPENNEAR_DEMO_RELAY_GPIO, (int)OPENNEAR_DEMO_ADC_GPIO);
}

void on_esp32_demo_on_cmd(const uint8_t *payload, size_t len) {
    if (!payload || len == 0u) return;
    char buf[48];
    size_t n = (len < sizeof(buf) - 1u) ? len : (sizeof(buf) - 1u);
    memcpy(buf, payload, n);
    buf[n] = '\0';
    while (n > 0u && (buf[n - 1u] == '\r' || buf[n - 1u] == '\n' || buf[n - 1u] == ' ')) {
        buf[--n] = '\0';
    }

    if (strcmp(buf, "MODE:SYNC") == 0) {
        s_demo_mode = DEMO_MODE_SYNC;
    } else if (strcmp(buf, "MODE:STROBE") == 0) {
        s_demo_mode = DEMO_MODE_STROBE;
    } else if (strcmp(buf, "MODE:PULSE") == 0) {
        s_demo_mode = DEMO_MODE_PULSE;
    } else if (strcmp(buf, "RELAY:1") == 0 || strcmp(buf, "MOTOR_ON") == 0 || strcmp(buf, "LED_1") == 0) {
        s_relay_state = 1u;
        digitalWrite(OPENNEAR_DEMO_RELAY_GPIO, HIGH);
    } else if (strcmp(buf, "RELAY:0") == 0 || strcmp(buf, "MOTOR_OFF") == 0 || strcmp(buf, "LED_0") == 0) {
        s_relay_state = 0u;
        digitalWrite(OPENNEAR_DEMO_RELAY_GPIO, LOW);
    } else if (strcmp(buf, "PING") == 0) {
        s_ping_until_ms = millis() + 600u;
    }
    Serial.printf("[OpenNear Demo] Applied cmd '%s' -> mode=%c relay=%u\n",
                  buf, demo_mode_char(s_demo_mode), (unsigned)s_relay_state);
}

uint8_t on_esp32_demo_act_level(uint32_t sf_seq, uint8_t default_level) {
    if (s_ping_until_ms != 0u && static_cast<int32_t>(millis() - s_ping_until_ms) < 0) {
        return (uint8_t)((sf_seq >> 3) & 1u); /* Rapid 15 Hz ping confirmation blink */
    }
    switch (s_demo_mode) {
        case DEMO_MODE_STROBE:
            /* 4 ms * 16 = 64 ms half-period -> crisp ~7.8 Hz synchronized strobe */
            return (uint8_t)((sf_seq >> 4) & 1u);
        case DEMO_MODE_PULSE: {
            /* 1-second (250 SF) heartbeat double-pulse locked strictly to global sf_seq */
            uint32_t ph = sf_seq % 250u;
            return (ph < 20u || (ph >= 45u && ph < 65u)) ? 1u : 0u;
        }
        default:
            return default_level;
    }
}

void on_esp32_demo_poll(on_node_t *node) {
    if (!node) return;
    uint32_t now_ms = millis();
    if (s_last_telem_ms != 0u && (now_ms - s_last_telem_ms) < 500u) return;
    s_last_telem_ms = now_ms;

    /* Sample analog input (mV approx) + pack compact 7-byte ASCII telemetry into TDMA_DATA */
    int raw = analogRead(OPENNEAR_DEMO_ADC_GPIO);
    unsigned mv = static_cast<unsigned>((raw * 3300) / 4095);
    char telem[12];
    int len = snprintf(telem, sizeof(telem), "%c%u:%04u",
                       demo_mode_char(s_demo_mode),
                       (unsigned)s_relay_state,
                       mv > 9999u ? 9999u : mv);
    if (len > 0) {
        if (len > 8) len = 8;
        on_esp32_node_lock();
        on_node_set_tx_payload(node, reinterpret_cast<const uint8_t *>(telem), static_cast<uint8_t>(len));
        on_esp32_node_unlock();
    }
}

#else

void on_esp32_demo_init(void) {}
void on_esp32_demo_on_cmd(const uint8_t *payload, size_t len) { (void)payload; (void)len; }
void on_esp32_demo_poll(on_node_t *node) { (void)node; }
uint8_t on_esp32_demo_act_level(uint32_t sf_seq, uint8_t default_level) {
    (void)sf_seq;
    return default_level;
}

#endif /* OPENNEAR_ENABLE_DEMO_APP */
