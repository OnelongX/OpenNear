#include "sim_world.h"
#include <stdio.h>

int main(void) {
    sim_world_t w;
    sim_world_init(&w, 0x20261122ULL);
    w.verbose = true;

    printf("=== OpenNear 10-Node Full Lifecycle Demonstration ===\n");
    printf("Topology: G + T1(Backup_ID) + T2(Sub-G proxy) + T3..T7 (1-hop) + T8,T9 (wall-blocked 2-hop)\n");
    printf("Phases:\n");
    printf("  [SF 0..34]  Dynamic C-RA Join + RTT Calibration + 10-Node Sub-us ACT Sync\n");
    printf("  [SF 35..44] 10-Frame Primary SYNC Blackout (NCO TC-PLL Coasting Holdover)\n");
    printf("  [SF 45..64] Recovery + Emergency C-RA Alert + Sub-G Edge Telemetry Relay\n");
    printf("  [SF 65..95] Sudden Primary G Power Loss -> T1 Backup_ID Zero-Vote Failover at SF+4\n\n");

    sim_world_setup_10node_demo(&w, false);
    sim_world_start_pcap(&w, "opennear_capture.pcap");

    /* Phase 1: 0..34 SF (140 ms) -> C-RA join + RTT + lock */
    w.blackout_node_idx  = 3; /* Shield T3 for 10 consecutive SYNC frames to verify NCO holdover */
    w.drop_sync_start_sf = 35u;
    w.drop_sync_end_sf   = 44u;
    sim_world_run_us(&w, 140000, 10);

    /* Phase 2: 35..44 SF (40 ms) -> 10-frame SYNC blackout holdover */
    sim_world_run_us(&w, 40000, 10);

    /* Phase 3: 45..64 SF (80 ms) -> trigger C-RA emergency alert on T5 */
    on_node_trigger_alert(&w.nodes[5], 0xA5u);
    w.kill_node_idx = 0;
    w.kill_at_sf    = 65u;
    sim_world_run_us(&w, 200000, 10);

    sim_world_print_report(&w);

    /* Summarize skew across key phases */
    int64_t steady_max = 0, holdover_max = 0, post_failover_max = 0;
    for (uint32_t i = 0; i < w.act_hist_count; ++i) {
        const sim_act_record_t *r = &w.act_hist[i];
        if (r->sf_seq >= 25u && r->sf_seq <= 34u) {
            if (r->max_skew_ns > steady_max) steady_max = r->max_skew_ns;
        } else if (r->sf_seq >= 35u && r->sf_seq <= 44u) {
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
