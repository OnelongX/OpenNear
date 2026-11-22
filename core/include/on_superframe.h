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

/*
 * v4.0 Multi-Gear Dynamic Superframe Profiles (NearLink SLB/SLE multi-rate scheduling):
 * - ON_SF_GEAR_FAST_500US : 500 us (2,000 Hz) ultra-low-latency 2k/4k esports HID / control
 * - ON_SF_GEAR_AUDIO_1MS  : 1,000 us (1,000 Hz) lossless spatial audio & industrial PLC
 * - ON_SF_GEAR_MESH_4MS   : 4,000 us (250 Hz) full 4-phase multi-hop Sub-G & low-power sensor
 */
typedef enum {
    ON_SF_GEAR_FAST_500US = 0,
    ON_SF_GEAR_AUDIO_1MS  = 1,
    ON_SF_GEAR_MESH_4MS   = 2
} on_sf_gear_t;

on_err_t   on_sf_cfg_set_gear(on_sf_cfg_t *cfg, on_sf_gear_t gear);

/*
 * v4.0 Dynamic Super-Slot Aggregation:
 * Checks how many contiguous idle D-TDMA slots starting at `own_slot` can be merged into
 * a single burst Super-Slot (up to `max_merge`) without colliding with allocated peers.
 */
uint8_t    on_sf_max_contiguous_slots(const on_sf_cfg_t *cfg,
                                      uint16_t tdma_bitmap,
                                      uint8_t own_slot,
                                      uint8_t max_merge);

uint32_t   on_sf_superslot_air_budget_us(const on_sf_cfg_t *cfg,
                                         uint8_t merged_slots);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_SUPERFRAME_H */
