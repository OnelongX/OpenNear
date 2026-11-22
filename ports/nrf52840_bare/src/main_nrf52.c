#include "on_node.h"
#include "hal_nrf52_radio_ppi.h"

#ifndef OPENNEAR_IS_MASTER
#define OPENNEAR_IS_MASTER 0
#endif

static on_node_t g_nrf_node;

/* Link directly to ARM Cortex-M4 vector table entry for RADIO_IRQHandler */
void RADIO_IRQHandler(void) {
    on_nrf52_radio_isr(&g_nrf_node);
}

int main(void) {
    on_nrf52_cfg_t cfg = {
        .rf_freq_offset_mhz = 40u,
        .act_gpio_pin       = ON_NRF52_ACT_LED_PIN,
        .tx_power_dbm       = 4
    };
    on_hal_ops_t hal;
    if (on_nrf52_hal_init(&cfg, &hal) != ON_OK) {
        for (;;) { }
    }

    on_node_init(&g_nrf_node, &hal,
                 (OPENNEAR_IS_MASTER != 0) ? ON_ROLE_G : ON_ROLE_UNJOINED,
                 (OPENNEAR_IS_MASTER != 0) ? 0x00u : 0x03u);

    for (;;) {
        int64_t now_ns = hal.now_ns(hal.user_ctx);
        on_node_step(&g_nrf_node, now_ns);

        /* Between active TDMA slots, shut down RADIO + 64 MHz HFCLK via RTC0 dual-clock sleep */
        if (!on_node_is_rx_open(&g_nrf_node, now_ns) && g_nrf_node.role != ON_ROLE_G) {
            int64_t next_wake_ns = on_node_next_wakeup_local_ns(&g_nrf_node, now_ns);
            hal.radio_sleep(hal.user_ctx);
            on_nrf52_deep_sleep_until_ns(next_wake_ns);
        }
    }
}
