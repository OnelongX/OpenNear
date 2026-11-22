#include "sim_world.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
    double      drift_scale  = 1.0;
    int64_t     hw_jitter_ns = 35;
    uint32_t    blackout_sfs = 10u;
    uint16_t    jam_mask     = 0x0000u;
    const char *kill_mode    = "g"; /* "g", "subg", or "none" */

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--drift-scale") == 0 && i + 1 < argc) {
            drift_scale = atof(argv[++i]);
        } else if (strcmp(argv[i], "--hw-jitter-ns") == 0 && i + 1 < argc) {
            hw_jitter_ns = (int64_t)atoll(argv[++i]);
        } else if (strcmp(argv[i], "--blackout-sfs") == 0 && i + 1 < argc) {
            blackout_sfs = (uint32_t)strtoul(argv[++i], NULL, 10);
        } else if (strcmp(argv[i], "--jam-mask") == 0 && i + 1 < argc) {
            jam_mask = (uint16_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "--kill-mode") == 0 && i + 1 < argc) {
            kill_mode = argv[++i];
        }
    }

    sim_world_t w;
    sim_world_init(&w, 0x20261122ULL);
    w.verbose = true;

    printf("=== OpenNear 10-Node Full Lifecycle Demonstration ===\n");
    printf("Config: DriftScale=%.2fx | HwJitter=+/-%lld ns | Blackout=%u SFs | JamMask=0x%04X | KillMode=%s\n",
           drift_scale, (long long)hw_jitter_ns, blackout_sfs, jam_mask, kill_mode);
    printf("Topology: G + T1(Backup_ID) + T2(Sub-G proxy) + T3..T7 (1-hop) + T8,T9 (wall-blocked 2-hop)\n");
    printf("Phases:\n");
    printf("  [SF 0..34]  Dynamic C-RA Join + RTT Calibration + 10-Node Sub-us ACT Sync\n");
    printf("  [SF 35..%u] %u-Frame Primary SYNC Blackout (NCO TC-PLL Coasting Holdover)\n",
           35u + (blackout_sfs > 0u ? blackout_sfs - 1u : 0u), blackout_sfs);
    printf("  [SF 45..64] Recovery + Emergency C-RA Alert + Sub-G Edge Telemetry Relay\n");
    printf("  [SF 65..95] Fault Injection (%s) -> Self-Healing & Sub-us Sync\n\n", kill_mode);

    sim_world_setup_10node_demo(&w, false);
    w.hw_jitter_bound_ns = hw_jitter_ns;
    for (uint32_t i = 0; i < w.num_nodes; ++i) {
        w.drift_ppm[i] *= drift_scale;
    }
    if (strcmp(kill_mode, "subg") == 0) {
        /* Enable T3 (idx=3) as standby Sub-G capable and reachable to wall-blocked T8/T9 */
        w.nodes[3].capabilities |= 0x02u;
        w.can_hear[3][8] = w.can_hear[8][3] = true;
        w.link_delay_ns[3][8] = w.link_delay_ns[8][3] = 380;
        w.can_hear[3][9] = w.can_hear[9][3] = true;
        w.link_delay_ns[3][9] = w.link_delay_ns[9][3] = 400;
    }

    sim_world_start_pcap(&w, "opennear_capture.pcap");

    /* Phase 1: 0..34 SF (139 ms, inside SF=34 ACT/QUIET before SF=35 pre-tune) -> C-RA join + RTT + lock */
    w.blackout_node_idx  = (blackout_sfs > 0u) ? 4 : -1; /* Shield T4 during blackout */
    w.drop_sync_start_sf = 35u;
    w.drop_sync_end_sf   = (blackout_sfs > 0u) ? (35u + blackout_sfs - 1u) : 34u;
    sim_world_run_us(&w, 139000, 10);

    /* Optional Wi-Fi jammer turns on cleanly before SF=35 boundary after all 10 nodes joined */
    if (jam_mask != 0u) {
        for (uint32_t i = 0; i < w.num_nodes; ++i) {
            on_node_enable_afh(&w.nodes[i], true);
        }
        w.jammed_ch_mask = jam_mask;
    }

    /* Phase 2: 35..44 SF (41 ms) -> SYNC blackout holdover & AFH reaction */
    sim_world_run_us(&w, 41000, 10);

    /* Phase 3: 45..105 SF -> trigger C-RA emergency alert on T5 + fault injection */
    on_node_trigger_alert(&w.nodes[5], 0xA5u);
    int64_t phase3_us = 200000;
    if (strcmp(kill_mode, "g") == 0) {
        w.kill_node_idx = 0;
        w.kill_at_sf    = 65u;
    } else if (strcmp(kill_mode, "subg") == 0) {
        w.kill_node_idx = 2;
        w.kill_at_sf    = 45u;
        phase3_us       = 260000; /* Run to SF=110 so T3 promotion at SF=75 + T8/T9 re-attach completes */
    } else {
        w.kill_node_idx = -1;
    }
    sim_world_run_us(&w, phase3_us, 10);

    sim_world_print_report(&w);

    /* Summarize skew across key phases */
    int64_t steady_max = 0, holdover_max = 0, post_failover_max = 0;
    uint32_t ho_end = (blackout_sfs > 0u) ? (35u + blackout_sfs - 1u) : 44u;
    for (uint32_t i = 0; i < w.act_hist_count; ++i) {
        const sim_act_record_t *r = &w.act_hist[i];
        if (r->sf_seq >= 25u && r->sf_seq <= 34u) {
            if (r->max_skew_ns > steady_max) steady_max = r->max_skew_ns;
        } else if (r->sf_seq >= 35u && r->sf_seq <= ho_end) {
            if (r->max_skew_ns > holdover_max) holdover_max = r->max_skew_ns;
        } else if (r->sf_seq >= 72u && r->sf_seq <= 90u) {
            if (r->max_skew_ns > post_failover_max) post_failover_max = r->max_skew_ns;
        }
    }

    printf("\nKey Measured Physical-Time ACT Sync Skew (All Alive Nodes incl. 2-Hop Wall-Blocked):\n");
    printf("  Steady-State (SF 25..34, 10 nodes):        max skew = %lld ns (%.3f us)\n",
           (long long)steady_max, (double)steady_max * 1e-3);
    printf("  10-Frame Blackout Holdover (SF 35..44):    max skew = %lld ns (%.3f us)\n",
           (long long)holdover_max, (double)holdover_max * 1e-3);
    printf("  Post Backup_ID Failover (SF 72..90):       max skew = %lld ns (%.3f us)\n",
           (long long)post_failover_max, (double)post_failover_max * 1e-3);
    printf("  Emergency C-RA Alert Delivered to G:       %s\n", w.alert_received_by_g ? "YES" : "NO");
    printf("  Wall-Blocked Edge Telemetry Relayed to G:  %s\n", w.edge_relay_received_by_g ? "YES" : "NO");

    sim_world_stop_pcap(&w);
    if (sim_world_export_csv(&w, "opennear_sim_trace.csv")) {
        printf("  Exported timeline CSV:  opennear_sim_trace.csv\n");
    }
    if (sim_world_export_dashboard_js(&w, "../tools/opennear_dashboard_data.js")) {
        printf("  Exported dashboard JS:  ../tools/opennear_dashboard_data.js\n");
    }
    printf("  Exported Wireshark PCAP: opennear_capture.pcap (%u frames, DLT_USER0)\n", w.pcap_pkts);
    return 0;
}
