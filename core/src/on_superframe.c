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
    if (cfg->t_sf_us < 1000u || cfg->t_sf_us > 20000u) return ON_ERR_INVALID_ARG;
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
    return (uint32_t)cfg->sub_g_start_us + ON_DEFAULT_SUBG_SYNC_US +
           (uint32_t)sub_slot_idx * cfg->slot_len_us;
}

uint32_t on_sf_subg_cra_start_us(const on_sf_cfg_t *cfg) {
    return (uint32_t)cfg->sub_g_start_us + ON_DEFAULT_SUBG_CRA_OFF;
}

uint32_t on_sf_cra_slot_start_us(const on_sf_cfg_t *cfg, uint8_t micro_slot) {
    uint32_t half = (uint32_t)(cfg->cra_end_us - cfg->cra_start_us) / 2u;
    return (uint32_t)cfg->cra_start_us + (micro_slot ? half : 0u);
}
