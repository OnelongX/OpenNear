#ifndef OPENNEAR_HAL_NRF52_RADIO_PPI_H
#define OPENNEAR_HAL_NRF52_RADIO_PPI_H

#include "on_hal.h"
#include "on_node.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Route A: Nordic nRF52840 / nRF54 Bare-Metal NRF_RADIO + PPI Driver (Zero BLE!)
 *
 * Hardware Zero-CPU-Latency Datapath:
 *   - BLE SoftDevice completely disabled; direct NRF_RADIO 2 Mbps GFSK proprietary mode.
 *   - Transmits OpenNear 6-byte Short-ID MAC frames natively on air (no 802.11 / BLE header).
 *   - PPI CH0: RADIO->EVENTS_ADDRESS  --> TIMER0->TASKS_CAPTURE[0]  (< 1 us HW RX capture)
 *   - PPI CH1: TIMER0->EVENTS_COMPARE[1] --> RADIO->TASKS_TXEN      (deterministic HW TX)
 *   - PPI CH2: TIMER0->EVENTS_COMPARE[2] --> GPIOTE->TASKS_OUT[0]   (sub-us ACT LED/GPIO)
 */

#define ON_NRF52_RF_CHANNEL_MHZ   2440u /* 2400 + 40 MHz */
#define ON_NRF52_ACCESS_ADDR      0x4F4E5359UL /* "ONSY" OpenNear sync word */
#define ON_NRF52_ACT_LED_PIN      13u   /* nRF52840-DK LED1 (P0.13) */

typedef struct {
    uint8_t  rf_freq_offset_mhz; /* 0..100 -> 2400 + offset MHz */
    uint8_t  act_gpio_pin;
    int8_t   tx_power_dbm;       /* 0, +4, +8 dBm */
} on_nrf52_cfg_t;

on_err_t on_nrf52_hal_init(const on_nrf52_cfg_t *cfg, on_hal_ops_t *hal_out);

/* Call from RADIO_IRQHandler() when EVENTS_END fires */
void     on_nrf52_radio_isr(on_node_t *node);

/* Program hardware PPI CH2 to toggle ACT GPIO at exact TIMER0 compare tick (sub-us jitter) */
void     on_nrf52_arm_hw_act_ns(int64_t target_local_ns);

/* v7.3 700m+ Long-Range PHY Control: switches NRF_RADIO between 2M GFSK (-93 dBm) and
 * 125 Kbps Coded S=8 FEC (-103 dBm) @ +8 dBm PA (111 dB native link budget -> 700m+ LOS) */
void     on_nrf52_set_long_range_mode(bool enable);

/*
 * v3.0 Dual-Clock Micro-Amp Deep Sleep Handoff (RTC0 32.768 kHz <-> HFCLK TIMER0 16 MHz):
 * Shuts down RADIO + 64 MHz HFCLK during idle TDMA sleep (>90% of superframe, ~1.5 uA),
 * sleeps via WFE until 400 us before wakeup_local_ns, then restarts HFCLK for sub-us lock!
 */
void     on_nrf52_deep_sleep_until_ns(int64_t wakeup_local_ns);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_HAL_NRF52_RADIO_PPI_H */
