#include "on_superframe.h"

void on_sf_cfg_default(on_sf_cfg_t *cfg) {
    if (!cfg) return;
    cfg->t_sf_us         = ON_DEFAULT_TSF_US;
    cfg->sync_end_us     = ON_DEFAULT_SYNC_END_US;
    cfg->tdma_start_us   = ON_DEFAULT_TDMA_START_US;
    cfg->slot_len_us     = ON_DEFAULT_SLOT_LEN_US;
    cfg->max_slots       = ON_DEFAULT_MAX_SLOTS;
    cfg->sub_g_start_us  = ON_DEFAULT_SUBG_START_US;
    cfg->sub_g_len_us    = ON_DEFAULT_SUBG_LEN_US;
    cfg->cra_start_us    = ON_DEFAULT_CRA_START_US;
    cfg->cra_end_us      = ON_DEFAULT_CRA_END_US;
    cfg->act_start_us    = ON_DEFAULT_ACT_START_US;
    cfg->act_fire_off_us = ON_DEFAULT_ACT_FIRE_US;
}

on_err_t on_sf_cfg_validate(const on_sf_cfg_t *cfg) {
    if (!cfg) return ON_ERR_INVALID_ARG;
    if (cfg->t_sf_us < 500u || cfg->t_sf_us > 20000u) return ON_ERR_INVALID_ARG;
    if (cfg->sync_end_us > cfg->tdma_start_us) return ON_ERR_INVALID_ARG;
    uint32_t tdma_end = (uint32_t)cfg->tdma_start_us + (uint32_t)cfg->max_slots * cfg->slot_len_us;
    if (tdma_end > cfg->sub_g_start_us) return ON_ERR_INVALID_ARG;
    if ((uint32_t)cfg->sub_g_start_us + cfg->sub_g_len_us > cfg->cra_start_us) return ON_ERR_INVALID_ARG;
    if (cfg->cra_start_us >= cfg->cra_end_us || cfg->cra_end_us > cfg->act_start_us) return ON_ERR_INVALID_ARG;
    if (cfg->act_fire_off_us < cfg->act_start_us || cfg->act_fire_off_us >= cfg->t_sf_us) return ON_ERR_INVALID_ARG;
    return ON_OK;
}

on_phase_t on_sf_phase_at(const on_sf_cfg_t *cfg, uint32_t offset_us) {
    if (offset_us < cfg->sync_end_us) {
        return ON_PHASE_SYNC_MAP;
    }
    if (offset_us >= cfg->sub_g_start_us &&
        offset_us < (uint32_t)cfg->sub_g_start_us + cfg->sub_g_len_us) {
        return ON_PHASE_SUB_G;
    }
    if (offset_us < cfg->cra_start_us) {
        return ON_PHASE_D_TDMA;
    }
    if (offset_us < cfg->act_start_us) {
        return ON_PHASE_C_RA;
    }
    return ON_PHASE_ACT_QUIET;
}

uint32_t on_sf_slot_start_us(const on_sf_cfg_t *cfg, uint8_t slot_idx) {
    return (uint32_t)cfg->tdma_start_us + (uint32_t)slot_idx * cfg->slot_len_us;
}

uint32_t on_sf_slot_end_us(const on_sf_cfg_t *cfg, uint8_t slot_idx) {
    return on_sf_slot_start_us(cfg, slot_idx) + cfg->slot_len_us;
}

uint32_t on_sf_subg_sync_start_us(const on_sf_cfg_t *cfg) {
    return (uint32_t)cfg->sub_g_start_us;
}

uint32_t on_sf_subg_slot_start_us(const on_sf_cfg_t *cfg, uint8_t sub_slot_idx) {
    uint32_t sub_sync_us = (cfg && cfg->t_sf_us >= 20000u) ? 1000u : ON_DEFAULT_SUBG_SYNC_US;
    return (uint32_t)cfg->sub_g_start_us + sub_sync_us +
           (uint32_t)sub_slot_idx * cfg->slot_len_us;
}

uint32_t on_sf_subg_cra_start_us(const on_sf_cfg_t *cfg) {
    uint32_t cra_off = (cfg && cfg->t_sf_us >= 20000u) ? 4600u : ON_DEFAULT_SUBG_CRA_OFF;
    return (uint32_t)cfg->sub_g_start_us + cra_off;
}

uint32_t on_sf_cra_slot_start_us(const on_sf_cfg_t *cfg, uint8_t micro_slot) {
    uint32_t half = (uint32_t)(cfg->cra_end_us - cfg->cra_start_us) / 2u;
    return (uint32_t)cfg->cra_start_us + (micro_slot ? half : 0u);
}

on_err_t on_sf_cfg_set_gear(on_sf_cfg_t *cfg, on_sf_gear_t gear) {
    if (!cfg) return ON_ERR_INVALID_ARG;
    switch (gear) {
        case ON_SF_GEAR_FAST_500US:
            /* 500 us (2 kHz) ultra-fast HID/control superframe (4M QPSK / 20M OFDM) */
            cfg->t_sf_us         = 500u;
            cfg->sync_end_us     = 60u;
            cfg->tdma_start_us   = 60u;
            cfg->slot_len_us     = 50u;
            cfg->max_slots       = 6u;   /* 60..360 us */
            cfg->sub_g_start_us  = 360u;
            cfg->sub_g_len_us    = 0u;   /* 1-hop low-latency direct mode */
            cfg->cra_start_us    = 360u;
            cfg->cra_end_us      = 440u;
            cfg->act_start_us    = 440u;
            cfg->act_fire_off_us = 470u;
            return on_sf_cfg_validate(cfg);

        case ON_SF_GEAR_AUDIO_1MS:
            /* 1,000 us (1 kHz) low-latency spatial audio / industrial PLC superframe */
            cfg->t_sf_us         = 1000u;
            cfg->sync_end_us     = 100u;
            cfg->tdma_start_us   = 100u;
            cfg->slot_len_us     = 80u;
            cfg->max_slots       = 8u;   /* 100..740 us */
            cfg->sub_g_start_us  = 740u;
            cfg->sub_g_len_us    = 0u;
            cfg->cra_start_us    = 750u;
            cfg->cra_end_us      = 900u;
            cfg->act_start_us    = 900u;
            cfg->act_fire_off_us = 950u;
            return on_sf_cfg_validate(cfg);

        case ON_SF_GEAR_LONGRANGE_20MS:
            /* 20,000 us (50 Hz) 700m+ Long-Range 4-phase superframe (1M DSSS / 250K LR + CA-Polar):
             * Preserves all 11 1-hop D-TDMA slots (900 us each) + 4 Sub-G 2-hop slots (900 us each)! */
            cfg->t_sf_us         = 20000u;
            cfg->sync_end_us     = 1200u;
            cfg->tdma_start_us   = 1200u;
            cfg->slot_len_us     = 900u;
            cfg->max_slots       = 11u;   /* 1200..11100 us */
            cfg->sub_g_start_us  = 11200u;/* 11200..16800 us (1000 us Sub-SYNC + 4x900 us Sub-TDMA + 1000 us Sub-CRA) */
            cfg->sub_g_len_us    = 5600u;
            cfg->cra_start_us    = 17000u;/* 17000..18800 us (2 x 900 us C-RA contention slots) */
            cfg->cra_end_us      = 18800u;
            cfg->act_start_us    = 18800u;
            cfg->act_fire_off_us = 19200u;
            return on_sf_cfg_validate(cfg);

        case ON_SF_GEAR_MESH_4MS:
        default:
            on_sf_cfg_default(cfg);
            return on_sf_cfg_validate(cfg);
    }
}

uint8_t on_sf_max_contiguous_slots(const on_sf_cfg_t *cfg,
                                   uint16_t tdma_bitmap,
                                   uint8_t own_slot,
                                   uint8_t max_merge) {
    if (!cfg || own_slot >= cfg->max_slots || max_merge == 0u) {
        return 0u;
    }
    uint8_t merged = 1u;
    for (uint8_t s = (uint8_t)(own_slot + 1u);
         s < cfg->max_slots && merged < max_merge;
         ++s) {
        /* If subsequent slot `s` is allocated to another peer, stop merging */
        if ((tdma_bitmap & (uint16_t)(1u << s)) != 0u) {
            break;
        }
        merged++;
    }
    return merged;
}

uint32_t on_sf_superslot_air_budget_us(const on_sf_cfg_t *cfg,
                                       uint8_t merged_slots) {
    if (!cfg || merged_slots == 0u) return 0u;
    uint32_t raw_us = (uint32_t)merged_slots * (uint32_t)cfg->slot_len_us;
    return (raw_us > 20u) ? (raw_us - 20u) : raw_us; /* 20 us tail guard */
}
