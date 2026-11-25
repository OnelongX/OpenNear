/*
 * OpenNear BSP / Radio Hardware Porting Template (v7.3.0)
 *
 * Copy this file into ports/<your_mcu>/ and implement the 7 callbacks below
 * to run the entire OpenNear L1~L4 protocol stack (42-suite verified pure C11)
 * on STM32WB, ESP32-S3/C3/C6, nRF5340, Zephyr RTOS, or custom SDR/FPGA boards.
 */

#include <opennear_core.h>
#include <string.h>

/* 1. Monotonic local hardware timer in nanoseconds (e.g. 1 MHz/16 MHz timer) */
static int64_t bsp_now_ns(void *ctx) {
    (void)ctx;
    /* TODO: return hardware_timer_us() * 1000LL; */
    return 0;
}

/* 2. Transmit a raw OpenNear frame (up to 128 B) at tx_at_local_ns (or ASAP if <= 0) */
static on_err_t bsp_tx_raw(void *ctx, const uint8_t *frame, size_t len, int64_t tx_at_local_ns) {
    (void)ctx; (void)frame; (void)len; (void)tx_at_local_ns;
    /* TODO: load packet into radio FIFO and trigger TX */
    return ON_OK;
}

/* 3. Enable or disable radio receiver window (for low-power TDMA duty cycling) */
static void bsp_radio_set_rx(void *ctx, bool enable) {
    (void)ctx; (void)enable;
}

/* 4. Place radio into sleep/idle state between superframe windows */
static void bsp_radio_sleep(void *ctx) {
    (void)ctx;
}

/* 5. Tune 2.4 GHz radio to logical TSCH channel index (0..15) */
static void bsp_radio_set_channel(void *ctx, uint8_t ch_idx) {
    (void)ctx; (void)ch_idx;
}

/* 6. Synchronized superframe ACT output trigger (GPIO / LED / PWM / Sensor latch) */
static void bsp_on_act_trigger(void *ctx, uint32_t sf_seq, uint8_t act_cmd, int64_t local_ns) {
    (void)ctx; (void)sf_seq; (void)act_cmd; (void)local_ns;
    /* TODO: drive hardware GPIO/LED at local_ns */
}

/* 7. Return permanent 6-byte factory hardware MAC48 / EUI-48 */
static void bsp_get_mac48(void *ctx, uint8_t mac_out[6]) {
    (void)ctx;
    static const uint8_t kDemoMac[6] = { 0x02u, 0x4Fu, 0x4Eu, 0x00u, 0x00u, 0x01u };
    memcpy(mac_out, kDemoMac, 6u);
}

void opennear_bsp_fill_hal(on_hal_ops_t *hal_out, void *user_ctx) {
    memset(hal_out, 0, sizeof(*hal_out));
    hal_out->user_ctx              = user_ctx;
    hal_out->now_ns                = bsp_now_ns;
    hal_out->tx_raw                = bsp_tx_raw;
    hal_out->radio_set_rx          = bsp_radio_set_rx;
    hal_out->radio_sleep           = bsp_radio_sleep;
    hal_out->radio_set_channel     = bsp_radio_set_channel;
    hal_out->on_act_trigger        = bsp_on_act_trigger;
    hal_out->get_mac48             = bsp_get_mac48;
    /* Set true if radio TX is driver-queued (e.g. Wi-Fi); false for bare-metal PPI */
    hal_out->tx_timing_best_effort = false;
}

/*
 * Feed received frames from your radio RX ISR / poll loop:
 *   on_node_on_rx_raw_rssi(&node, rx_buf, rx_len, rx_timestamp_ns, rssi_dbm);
 * Advance the protocol state machine from your RT task / main loop:
 *   on_node_step(&node, bsp_now_ns(NULL));
 */
