#include "hal_nrf52_radio_ppi.h"
#include "on_afh.h"
#include <string.h>

/*
 * Exact nRF52840 Product Specification v1.7 MMIO Register Offsets.
 * Compiles standalone or alongside Nordic MDK (nrf52840.h).
 */
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

/* CLOCK (0x40000000) */
#define NRF_CLOCK_BASE              0x40000000UL
#define CLOCK_TASKS_HFCLKSTART      REG32(NRF_CLOCK_BASE + 0x000UL)
#define CLOCK_TASKS_HFCLKSTOP       REG32(NRF_CLOCK_BASE + 0x004UL)
#define CLOCK_TASKS_LFCLKSTART      REG32(NRF_CLOCK_BASE + 0x008UL)
#define CLOCK_EVENTS_HFCLKSTARTED   REG32(NRF_CLOCK_BASE + 0x100UL)
#define CLOCK_EVENTS_LFCLKSTARTED   REG32(NRF_CLOCK_BASE + 0x104UL)
#define CLOCK_LFCLKSRC              REG32(NRF_CLOCK_BASE + 0x518UL)

/* RTC0 (0x4000B000): 32.768 kHz Ultra-Low-Power Sleep Counter (~1.5 uA!) */
#define NRF_RTC0_BASE               0x4000B000UL
#define RTC0_TASKS_START            REG32(NRF_RTC0_BASE + 0x000UL)
#define RTC0_EVENTS_COMPARE0        REG32(NRF_RTC0_BASE + 0x140UL)
#define RTC0_INTENSET               REG32(NRF_RTC0_BASE + 0x304UL)
#define RTC0_EVTENSET               REG32(NRF_RTC0_BASE + 0x344UL)
#define RTC0_COUNTER                REG32(NRF_RTC0_BASE + 0x504UL)
#define RTC0_PRESCALER              REG32(NRF_RTC0_BASE + 0x508UL)
#define RTC0_CC0                    REG32(NRF_RTC0_BASE + 0x540UL)

/* RADIO (0x40001000) */
#define NRF_RADIO_BASE              0x40001000UL
#define RADIO_TASKS_TXEN            (NRF_RADIO_BASE + 0x000UL)
#define RADIO_TASKS_RXEN            (NRF_RADIO_BASE + 0x004UL)
#define RADIO_TASKS_START           (NRF_RADIO_BASE + 0x008UL)
#define RADIO_TASKS_DISABLE         (NRF_RADIO_BASE + 0x010UL)
#define RADIO_EVENTS_READY          (NRF_RADIO_BASE + 0x100UL)
#define RADIO_EVENTS_ADDRESS        (NRF_RADIO_BASE + 0x104UL)
#define RADIO_EVENTS_END            (NRF_RADIO_BASE + 0x10CUL)
#define RADIO_EVENTS_DISABLED       (NRF_RADIO_BASE + 0x110UL)
#define RADIO_SHORTS                REG32(NRF_RADIO_BASE + 0x200UL)
#define RADIO_INTENSET              REG32(NRF_RADIO_BASE + 0x304UL)
#define RADIO_CRCSTATUS             REG32(NRF_RADIO_BASE + 0x400UL)
#define RADIO_PACKETPTR             REG32(NRF_RADIO_BASE + 0x504UL)
#define RADIO_FREQUENCY             REG32(NRF_RADIO_BASE + 0x508UL)
#define RADIO_TXPOWER               REG32(NRF_RADIO_BASE + 0x50CUL)
#define RADIO_MODE                  REG32(NRF_RADIO_BASE + 0x510UL)
#define RADIO_PCNF0                 REG32(NRF_RADIO_BASE + 0x514UL)
#define RADIO_PCNF1                 REG32(NRF_RADIO_BASE + 0x518UL)
#define RADIO_BASE0                 REG32(NRF_RADIO_BASE + 0x51CUL)
#define RADIO_PREFIX0               REG32(NRF_RADIO_BASE + 0x524UL)
#define RADIO_TXADDRESS             REG32(NRF_RADIO_BASE + 0x52CUL)
#define RADIO_RXADDRESSES           REG32(NRF_RADIO_BASE + 0x530UL)
#define RADIO_CRCCNF                REG32(NRF_RADIO_BASE + 0x534UL)
#define RADIO_CRCPOLY               REG32(NRF_RADIO_BASE + 0x538UL)
#define RADIO_CRCINIT               REG32(NRF_RADIO_BASE + 0x53CUL)
#define RADIO_MODECNF0              REG32(NRF_RADIO_BASE + 0x650UL)

/* GPIOTE (0x40006000) */
#define NRF_GPIOTE_BASE             0x40006000UL
#define GPIOTE_TASKS_OUT0           (NRF_GPIOTE_BASE + 0x000UL)
#define GPIOTE_CONFIG0              REG32(NRF_GPIOTE_BASE + 0x510UL)

/* TIMER0 (0x40008000): 16 MHz / 2^0 = 16 MHz (62.5 ns hardware tick resolution!) */
#define NRF_TIMER0_BASE             0x40008000UL
#define TIMER0_TASKS_START          REG32(NRF_TIMER0_BASE + 0x000UL)
#define TIMER0_TASKS_CAPTURE0       (NRF_TIMER0_BASE + 0x040UL)
#define TIMER0_TASKS_CAPTURE3       REG32(NRF_TIMER0_BASE + 0x04CUL)
#define TIMER0_EVENTS_COMPARE1      (NRF_TIMER0_BASE + 0x144UL)
#define TIMER0_EVENTS_COMPARE2      (NRF_TIMER0_BASE + 0x148UL)
#define TIMER0_MODE                 REG32(NRF_TIMER0_BASE + 0x504UL)
#define TIMER0_BITMODE              REG32(NRF_TIMER0_BASE + 0x508UL)
#define TIMER0_PRESCALER            REG32(NRF_TIMER0_BASE + 0x510UL)
#define TIMER0_CC0                  REG32(NRF_TIMER0_BASE + 0x540UL)
#define TIMER0_CC1                  REG32(NRF_TIMER0_BASE + 0x544UL)
#define TIMER0_CC2                  REG32(NRF_TIMER0_BASE + 0x548UL)
#define TIMER0_CC3                  REG32(NRF_TIMER0_BASE + 0x54CUL)

/* PPI (0x4001F000): Programmable Peripheral Interconnect (Zero CPU Latency!) */
#define NRF_PPI_BASE                0x4001F000UL
#define PPI_CHENSET                 REG32(NRF_PPI_BASE + 0x504UL)
#define PPI_CH_EEP(n)               REG32(NRF_PPI_BASE + 0x510UL + (n) * 8UL)
#define PPI_CH_TEP(n)               REG32(NRF_PPI_BASE + 0x514UL + (n) * 8UL)

/* FICR Factory 48-bit Device Address */
#define NRF_FICR_DEVICEADDR0        REG32(0x100000A4UL)
#define NRF_FICR_DEVICEADDR1        REG32(0x100000A8UL)

/* DMA Packet Buffer: [0] = LENGTH byte, [1..N] = Native OpenNear 6B-header Frame */
static uint8_t s_dma_pkt[1u + ON_MAX_FRAME_LEN] __attribute__((aligned(4)));
static uint64_t s_timer_high_ticks = 0u;
static uint32_t s_timer_last_ticks = 0u;

/* Convert 16 MHz TIMER0 ticks (62.5 ns/tick) -> monotonic 64-bit nanoseconds */
static int64_t ticks16m_to_ns(uint32_t ticks32) {
    if (ticks32 < s_timer_last_ticks && (s_timer_last_ticks - ticks32) > 0x80000000UL) {
        s_timer_high_ticks += (1ULL << 32);
    }
    s_timer_last_ticks = ticks32;
    uint64_t full = s_timer_high_ticks + (uint64_t)ticks32;
    /* 1 tick = 62.5 ns = 125 / 2 ns */
    return (int64_t)((full * 125ULL) / 2ULL);
}

static uint32_t ns_to_ticks16m(int64_t local_ns) {
    uint64_t ticks = ((uint64_t)local_ns * 2ULL) / 125ULL;
    return (uint32_t)(ticks & 0xFFFFFFFFULL);
}

static int64_t nrf52_now_ns(void *ctx) {
    (void)ctx;
    TIMER0_TASKS_CAPTURE3 = 1UL;
    return ticks16m_to_ns(TIMER0_CC3);
}

static on_err_t nrf52_tx_raw(void *ctx, const uint8_t *frame, size_t len, int64_t tx_at_local_ns) {
    (void)ctx;
    if (!frame || len > ON_MAX_FRAME_LEN) return ON_ERR_INVALID_ARG;

    /* Copy native OpenNear 6B-header frame directly into DMA buffer */
    s_dma_pkt[0] = (uint8_t)len;
    memcpy(&s_dma_pkt[1], frame, len);

    REG32(RADIO_TASKS_DISABLE) = 1UL;
    while (REG32(RADIO_EVENTS_DISABLED) == 0UL) { }
    REG32(RADIO_EVENTS_DISABLED) = 0UL;
    REG32(RADIO_EVENTS_END)      = 0UL;

    RADIO_PACKETPTR = (uint32_t)(uintptr_t)s_dma_pkt;

    if (tx_at_local_ns > 0) {
        /* Hardware-timed TX via PPI CH1: TIMER0->COMPARE[1] -> RADIO->TASKS_TXEN
         * Subtract 40 us fast ramp-up time (MODECNF0.RU = 1) */
        int64_t ramp_ns = tx_at_local_ns - 40000LL;
        TIMER0_CC1 = ns_to_ticks16m(ramp_ns > 0 ? ramp_ns : 0);
        REG32(TIMER0_EVENTS_COMPARE1) = 0UL;
        PPI_CHENSET = (1UL << 1);
    } else {
        REG32(RADIO_TASKS_TXEN) = 1UL;
    }
    return ON_OK;
}

static void nrf52_radio_set_rx(void *ctx, bool enable) {
    (void)ctx;
    if (enable) {
        RADIO_PACKETPTR = (uint32_t)(uintptr_t)s_dma_pkt;
        REG32(RADIO_EVENTS_END) = 0UL;
        REG32(RADIO_TASKS_RXEN) = 1UL;
    } else {
        REG32(RADIO_TASKS_DISABLE) = 1UL;
    }
}

static void nrf52_radio_sleep(void *ctx) {
    nrf52_radio_set_rx(ctx, false);
}

static void nrf52_radio_set_channel(void *ctx, uint8_t ch_idx) {
    (void)ctx;
    /* Direct 2 MHz narrowband frequency hop in NRF_RADIO->FREQUENCY (2400 + offset MHz) */
    RADIO_FREQUENCY = (uint32_t)on_afh_ch_to_rf_offset_mhz(ch_idx);
}

static void nrf52_on_act_trigger(void *ctx, uint32_t sf_seq, uint8_t act_cmd, int64_t local_ns) {
    (void)ctx;
    (void)sf_seq;
    (void)act_cmd;
    /* Arm hardware PPI CH2 for zero-jitter GPIOTE toggle */
    on_nrf52_arm_hw_act_ns(local_ns);
}

static void nrf52_get_mac48(void *ctx, uint8_t mac_out[6]) {
    (void)ctx;
    uint32_t lo = NRF_FICR_DEVICEADDR0;
    uint32_t hi = NRF_FICR_DEVICEADDR1;
    mac_out[0] = (uint8_t)(lo & 0xFFu);
    mac_out[1] = (uint8_t)((lo >> 8) & 0xFFu);
    mac_out[2] = (uint8_t)((lo >> 16) & 0xFFu);
    mac_out[3] = (uint8_t)((lo >> 24) & 0xFFu);
    mac_out[4] = (uint8_t)(hi & 0xFFu);
    mac_out[5] = (uint8_t)((hi >> 8) & 0xFFu);
}

void on_nrf52_arm_hw_act_ns(int64_t target_local_ns) {
    TIMER0_CC2 = ns_to_ticks16m(target_local_ns);
    REG32(TIMER0_EVENTS_COMPARE2) = 0UL;
    PPI_CHENSET = (1UL << 2);
}

void on_nrf52_deep_sleep_until_ns(int64_t wakeup_local_ns) {
    int64_t now_ns = nrf52_now_ns(NULL);
    /* Only drop HFCLK if sleep duration > 800 us (400 us HFCLK startup margin + 400 us sleep) */
    int64_t sleep_ns = wakeup_local_ns - now_ns - 400000LL;
    if (sleep_ns <= 400000LL) return;

    /* 1. Convert sleep_ns -> 32.768 kHz RTC0 ticks (1 tick = 30,517.578 ns) */
    uint32_t sleep_rtc_ticks = (uint32_t)(((uint64_t)sleep_ns * 32768ULL) / 1000000000ULL);
    if (sleep_rtc_ticks < 2u) return;

    uint32_t rtc_now = RTC0_COUNTER;
    RTC0_CC0 = (rtc_now + sleep_rtc_ticks) & 0x00FFFFFFUL;
    RTC0_EVENTS_COMPARE0 = 0UL;

    /* 2. Power down RADIO & 64 MHz HFCLK crystal -> system enters ~1.5 uA System ON sleep */
    REG32(RADIO_TASKS_DISABLE) = 1UL;
    CLOCK_TASKS_HFCLKSTOP = 1UL;

    while (RTC0_EVENTS_COMPARE0 == 0UL) {
#if defined(__arm__) || defined(__thumb__)
        __asm volatile ("wfe");
#else
        break;
#endif
    }
    RTC0_EVENTS_COMPARE0 = 0UL;

    /* 3. Wake 64 MHz HFCLK crystal ahead of SYNC/MAP & advance high-res tick accumulator */
    CLOCK_EVENTS_HFCLKSTARTED = 0UL;
    CLOCK_TASKS_HFCLKSTART    = 1UL;
    while (CLOCK_EVENTS_HFCLKSTARTED == 0UL) { }

    /* Compensate TIMER0 elapsed time during HFCLK-off period using exact 32.768 kHz delta */
    uint32_t rtc_elapsed = (RTC0_COUNTER - rtc_now) & 0x00FFFFFFUL;
    uint64_t equiv_16m_ticks = ((uint64_t)rtc_elapsed * 16000000ULL) / 32768ULL;
    s_timer_high_ticks += equiv_16m_ticks;
}

on_err_t on_nrf52_hal_init(const on_nrf52_cfg_t *cfg, on_hal_ops_t *hal_out) {
    if (!hal_out) return ON_ERR_INVALID_ARG;

    uint8_t freq_off = cfg ? cfg->rf_freq_offset_mhz : 40u;
    uint8_t act_pin  = cfg ? cfg->act_gpio_pin : ON_NRF52_ACT_LED_PIN;

    /* 1. Start external 32.768 kHz LFCLK + RTC0 (micro-amp sleep timebase) & 64 MHz HFCLK */
    CLOCK_LFCLKSRC            = 1UL; /* External 32.768 kHz XTAL */
    CLOCK_EVENTS_LFCLKSTARTED = 0UL;
    CLOCK_TASKS_LFCLKSTART    = 1UL;
    while (CLOCK_EVENTS_LFCLKSTARTED == 0UL) { }
    RTC0_PRESCALER = 0UL; /* 32.768 kHz */
    RTC0_EVTENSET  = (1UL << 16); /* COMPARE0 event for WFE wakeup */
    RTC0_INTENSET  = (1UL << 16);
    RTC0_TASKS_START = 1UL;

    CLOCK_EVENTS_HFCLKSTARTED = 0UL;
    CLOCK_TASKS_HFCLKSTART    = 1UL;
    while (CLOCK_EVENTS_HFCLKSTARTED == 0UL) { }

    /* 2. Configure TIMER0 at full 16 MHz (62.5 ns per tick!), 32-bit free-running */
    TIMER0_MODE      = 0UL; /* Timer mode */
    TIMER0_BITMODE   = 3UL; /* 32-bit */
    TIMER0_PRESCALER = 0UL; /* 16 MHz / 2^0 = 16 MHz */
    TIMER0_TASKS_START = 1UL;

    /* 3. Configure GPIOTE Channel 0 in Task Toggle mode for sub-us ACT LED/GPIO */
    GPIOTE_CONFIG0 = (3UL << 0)               /* Mode = Task */
                   | ((uint32_t)act_pin << 8) /* PSEL */
                   | (3UL << 16);             /* Polarity = Toggle */

    /* 4. Configure NRF_RADIO in bare-metal 2 Mbps GFSK proprietary mode (NO BLE!) */
    RADIO_MODE      = 4UL; /* Ble_2Mbit = 2 Mbps GFSK with 16-bit fast preamble (8 us!) */
    RADIO_MODECNF0  = 1UL; /* Fast ramp-up (40 us instead of 130 us) */
    RADIO_FREQUENCY = freq_off;
    RADIO_TXPOWER   = (uint32_t)(int32_t)(cfg ? cfg->tx_power_dbm : 0);

    /* Direct OpenNear frame format: 8-bit LENGTH + payload up to ON_MAX_FRAME_LEN */
    RADIO_PCNF0 = (8UL << 0); /* LFLEN = 8 bits, S0LEN = 0, S1LEN = 0 */
    RADIO_PCNF1 = ((uint32_t)ON_MAX_FRAME_LEN << 0) /* MAXLEN */
                | (0UL << 8)                        /* STATLEN = 0 */
                | (3UL << 16)                       /* BALEN = 3 (4-byte access address) */
                | (0UL << 24)                       /* Little-endian */
                | (1UL << 25);                      /* Hardware whitening ON */

    RADIO_BASE0       = (ON_NRF52_ACCESS_ADDR << 8);
    RADIO_PREFIX0     = (ON_NRF52_ACCESS_ADDR >> 24) & 0xFFUL;
    RADIO_TXADDRESS   = 0UL;
    RADIO_RXADDRESSES = 1UL;

    /* Hardware 24-bit CRC */
    RADIO_CRCCNF  = 3UL;
    RADIO_CRCPOLY = 0x00065BUL;
    RADIO_CRCINIT = 0x555555UL;

    /* Hardware shortcuts: READY->START, END->DISABLE */
    RADIO_SHORTS = (1UL << 0) | (1UL << 1);
    RADIO_INTENSET = (1UL << 3); /* END interrupt */

    /* 5. Wire Hardware PPI Channels (ZERO CPU INTERVENTION!) */
    /* PPI CH0: RADIO->EVENTS_ADDRESS -> TIMER0->TASKS_CAPTURE[0] (62.5 ns hardware RX capture!) */
    PPI_CH_EEP(0) = RADIO_EVENTS_ADDRESS;
    PPI_CH_TEP(0) = TIMER0_TASKS_CAPTURE0;
    /* PPI CH1: TIMER0->EVENTS_COMPARE[1] -> RADIO->TASKS_TXEN */
    PPI_CH_EEP(1) = TIMER0_EVENTS_COMPARE1;
    PPI_CH_TEP(1) = RADIO_TASKS_TXEN;
    /* PPI CH2: TIMER0->EVENTS_COMPARE[2] -> GPIOTE->TASKS_OUT[0] */
    PPI_CH_EEP(2) = TIMER0_EVENTS_COMPARE2;
    PPI_CH_TEP(2) = GPIOTE_TASKS_OUT0;

    PPI_CHENSET = (1UL << 0); /* Keep CH0 always enabled for instant RX timestamp capture */

    memset(hal_out, 0, sizeof(*hal_out));
    hal_out->now_ns         = nrf52_now_ns;
    hal_out->tx_raw         = nrf52_tx_raw;
    hal_out->radio_set_rx      = nrf52_radio_set_rx;
    hal_out->radio_sleep       = nrf52_radio_sleep;
    hal_out->radio_set_channel = nrf52_radio_set_channel;
    hal_out->on_act_trigger = nrf52_on_act_trigger;
    hal_out->get_mac48      = nrf52_get_mac48;
    return ON_OK;
}

void on_nrf52_radio_isr(on_node_t *node) {
    if (REG32(RADIO_EVENTS_END) == 0UL) return;
    REG32(RADIO_EVENTS_END) = 0UL;

    if (RADIO_CRCSTATUS == 1UL && s_dma_pkt[0] >= ON_MAC_HDR_LEN + ON_PAYLOAD_CRC_LEN) {
        /* Read exact 62.5 ns hardware timestamp captured by PPI CH0 at preamble ADDRESS match! */
        int64_t hw_rx_ns = ticks16m_to_ns(TIMER0_CC0);
        on_node_on_rx_raw(node, &s_dma_pkt[1], s_dma_pkt[0], hw_rx_ns);
    }
}
