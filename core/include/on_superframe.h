#ifndef OPENNEAR_SUPERFRAME_H
#define OPENNEAR_SUPERFRAME_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear L2 4-Phase Dynamic Superframe Schedule Descriptor
 *
 * |<------------------- T_SF (1,000 ~ 20,000 us, default 4,000 us) ------------------->|
 * | SYNC/MAP (0..120) | D-TDMA (120..3200, incl. Sub-G 2000..3000) | C-RA | ACT/QUIET  |
 */
typedef struct {
    uint16_t t_sf_us;
    uint16_t sync_end_us;
    uint16_t tdma_start_us;
    uint16_t slot_len_us;
    uint8_t  max_slots;
    uint16_t sub_g_start_us;
    uint16_t sub_g_len_us;
    uint16_t cra_start_us;
    uint16_t cra_end_us;
    uint16_t act_start_us;
    uint16_t act_fire_off_us;
} on_sf_cfg_t;

void       on_sf_cfg_default(on_sf_cfg_t *cfg);
on_err_t   on_sf_cfg_validate(const on_sf_cfg_t *cfg);

/* Query which 4-phase segment an offset_us (0 .. t_sf_us-1) falls in */
on_phase_t on_sf_phase_at(const on_sf_cfg_t *cfg, uint32_t offset_us);

/* Primary D-TDMA slot start/end offset in microseconds */
uint32_t   on_sf_slot_start_us(const on_sf_cfg_t *cfg, uint8_t slot_idx);
uint32_t   on_sf_slot_end_us(const on_sf_cfg_t *cfg, uint8_t slot_idx);

/* Carved Sub-G child slot timing (guaranteed non-overlapping with primary D-TDMA) */
uint32_t   on_sf_subg_sync_start_us(const on_sf_cfg_t *cfg);
uint32_t   on_sf_subg_slot_start_us(const on_sf_cfg_t *cfg, uint8_t sub_slot_idx);
uint32_t   on_sf_subg_cra_start_us(const on_sf_cfg_t *cfg);

/* C-RA micro-slot timing (micro_slot = 0 or 1) */
uint32_t   on_sf_cra_slot_start_us(const on_sf_cfg_t *cfg, uint8_t micro_slot);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_SUPERFRAME_H */
