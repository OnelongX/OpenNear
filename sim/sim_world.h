#ifndef OPENNEAR_SIM_WORLD_H
#define OPENNEAR_SIM_WORLD_H

#include "on_node.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_MAX_NODES        12u
#define SIM_MAX_IN_FLIGHT    64u
#define SIM_MAX_SF_HISTORY   256u

typedef struct {
    bool     active;
    uint32_t src_idx;
    uint8_t  ch_idx;
    uint8_t  frame[ON_MAX_FRAME_LEN];
    size_t   len;
    int64_t  tx_start_phys_ns;
    int64_t  tx_end_phys_ns;
    uint8_t  frame_type;
    bool     is_sub_g;
} sim_air_pkt_t;

typedef struct {
    uint32_t sf_seq;
    uint32_t fired_mask;
    int64_t  fire_phys_ns[SIM_MAX_NODES];
    uint8_t  fire_cmd[SIM_MAX_NODES];
    int64_t  max_skew_ns;
    bool     cmd_consistent;
} sim_act_record_t;

typedef struct {
    uint32_t       num_nodes;
    int64_t        phys_now_ns;
    uint64_t       rng_state;

    /* Nodes and physical oscillator parameters */
    on_node_t      nodes[SIM_MAX_NODES];
    char           names[SIM_MAX_NODES][16];
    bool           alive[SIM_MAX_NODES];
    double         drift_ppm[SIM_MAX_NODES];
    int64_t        boot_offset_ns[SIM_MAX_NODES];
    int64_t        hw_jitter_bound_ns;

    /* RF channel matrix & per-node active TSCH channel */
    bool           can_hear[SIM_MAX_NODES][SIM_MAX_NODES];
    int64_t        link_delay_ns[SIM_MAX_NODES][SIM_MAX_NODES];
    uint8_t        cur_ch[SIM_MAX_NODES];

    /* Air packets queue */
    sim_air_pkt_t  air[SIM_MAX_IN_FLIGHT];

    /* Fault, interference, Wi-Fi jammer & impulse-drop injection */
    int32_t        blackout_node_idx; /* -1 = all nodes, >=0 = specific node RX blackout */
    uint32_t       drop_sync_start_sf;
    uint32_t       drop_sync_end_sf;
    int32_t        kill_node_idx;
    uint32_t       kill_at_sf;
    bool           kill_executed;
    uint16_t       jammed_ch_mask;    /* 16-bit mask of Wi-Fi jammed logical channels */
    uint32_t       jammed_drops;
    int32_t        impulse_drop_node_idx;
    uint32_t       impulse_drop_sf;

    /* Captured encrypted wire packet for live replay/tamper attack tests */
    bool           captured_sec_pkt;
    uint8_t        cap_frame[ON_MAX_FRAME_LEN];
    size_t         cap_len;
    uint32_t       cap_src_idx;

    /* Metrics */
    uint32_t       total_tx_packets;
    uint32_t       total_rx_packets;
    uint32_t       tdma_subg_collisions;
    uint32_t       cra_collisions;
    uint32_t       sync_drops_injected;
    uint32_t       failover_sf_seq;
    uint16_t       failover_new_g_id;
    bool           alert_received_by_g;
    bool           edge_relay_received_by_g;

    /* Per-superframe ACT synchronization measurements in true physical time */
    sim_act_record_t act_hist[SIM_MAX_SF_HISTORY];
    uint32_t         act_hist_count;

    /* Per-superframe AFH channel history & PCAP capture output */
    uint8_t          sf_ch_hist[SIM_MAX_SF_HISTORY];
    uint16_t         sf_map_hist[SIM_MAX_SF_HISTORY];
    FILE            *pcap_fp;
    uint32_t         pcap_pkts;

    bool           verbose;
} sim_world_t;

void     sim_world_init(sim_world_t *w, uint64_t seed);
void     sim_world_setup_10node_demo(sim_world_t *w, bool pre_registered_fast);
int64_t  sim_phys_to_local_ns(const sim_world_t *w, uint32_t idx, int64_t phys_ns);
int64_t  sim_local_to_phys_ns(const sim_world_t *w, uint32_t idx, int64_t local_ns);
void     sim_world_run_us(sim_world_t *w, int64_t duration_us, int64_t step_us);
void     sim_world_print_report(const sim_world_t *w);
bool     sim_world_export_csv(const sim_world_t *w, const char *csv_path);
bool     sim_world_start_pcap(sim_world_t *w, const char *pcap_path);
void     sim_world_stop_pcap(sim_world_t *w);
bool     sim_world_export_dashboard_js(const sim_world_t *w, const char *js_path);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_SIM_WORLD_H */
