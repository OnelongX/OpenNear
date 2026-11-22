#include "on_types.h"
#include "on_frame.h"
#include "on_polar.h"
#include "on_sync.h"
#include "on_superframe.h"
#include "on_sec.h"
#include "on_afh.h"
#include "on_amc.h"
#include "on_sar.h"
#include "on_node.h"
#include "../sim/sim_world.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define EXPECT(cond, msg) do { \
    if (cond) { \
        g_tests_passed++; \
    } else { \
        g_tests_failed++; \
        fprintf(stderr, "  [FAIL] %s:%d: %s\n", __FILE__, __LINE__, msg); \
    } \
} while (0)

/* Deterministic Box-Muller AWGN helper */
static uint64_t g_rng = 0xCAFEBABE12345678ULL;
static double rand_uniform01(void) {
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 7;
    g_rng ^= g_rng << 17;
    return ((double)(g_rng & 0xFFFFFFFFULL) + 1.0) / 4294967297.0;
}

static double rand_gauss(void) {
    double u1 = rand_uniform01();
    double u2 = rand_uniform01();
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

/* 1. Test 6-Byte Compact MAC Header + CRC8 fast drop + CRC16 payload integrity */
static void test_6byte_header_and_frame(void) {
    printf("[TEST 1] 6-Byte Short-ID MAC Header & Frame Codec...\n");
    on_mac_hdr_t in_hdr;
    in_hdr.version    = ON_PROTO_VERSION;
    in_hdr.type       = ON_FRAME_TDMA_DATA;
    in_hdr.sec_en     = true;
    in_hdr.is_sub_g   = true;
    in_hdr.src_id     = 0xABCu;
    in_hdr.dst_id     = 0xDEFu;
    in_hdr.sf_seq_low = 0x0Du;
    in_hdr.slot_idx   = 0x07u;

    uint8_t raw6[ON_MAC_HDR_LEN];
    EXPECT(on_hdr_pack(&in_hdr, raw6) == ON_OK, "on_hdr_pack returns ON_OK");
    EXPECT(sizeof(raw6) == 6u, "MAC header is strictly 6 bytes");

    on_mac_hdr_t out_hdr;
    EXPECT(on_hdr_unpack(raw6, &out_hdr) == ON_OK, "on_hdr_unpack succeeds");
    EXPECT(out_hdr.version    == in_hdr.version,    "version preserved");
    EXPECT(out_hdr.type       == in_hdr.type,       "type preserved");
    EXPECT(out_hdr.sec_en     == in_hdr.sec_en,     "sec_en preserved");
    EXPECT(out_hdr.is_sub_g   == in_hdr.is_sub_g,   "is_sub_g preserved");
    EXPECT(out_hdr.src_id     == 0xABCu,            "12-bit src_id preserved");
    EXPECT(out_hdr.dst_id     == 0xDEFu,            "12-bit dst_id preserved");
    EXPECT(out_hdr.sf_seq_low == 0x0Du,             "sf_seq_low preserved");
    EXPECT(out_hdr.slot_idx   == 0x07u,             "slot_idx preserved");

    /* Flip 1 bit in header -> must fail Hdr_CRC8 immediately */
    raw6[2] ^= 0x08u;
    EXPECT(on_hdr_unpack(raw6, &out_hdr) == ON_ERR_HDR_CRC, "corrupted header rejected by Hdr_CRC8");

    /* Full frame round-trip */
    const uint8_t payload[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    uint8_t frame[ON_MAX_FRAME_LEN];
    size_t flen = 0u;
    EXPECT(on_frame_pack(&in_hdr, payload, sizeof(payload), frame, sizeof(frame), &flen) == ON_OK,
           "on_frame_pack succeeds");
    EXPECT(flen == 6u + 5u + 2u, "frame length = 6B hdr + 5B payload + 2B CRC16");

    const uint8_t *pptr = NULL;
    size_t plen = 0u;
    EXPECT(on_frame_unpack(frame, flen, &out_hdr, &pptr, &plen) == ON_OK, "on_frame_unpack succeeds");
    EXPECT(plen == 5u && memcmp(pptr, payload, 5u) == 0, "payload bytes match");

    /* Flip 1 bit in payload -> must fail CRC16 */
    frame[7] ^= 0x01u;
    EXPECT(on_frame_unpack(frame, flen, &out_hdr, &pptr, &plen) == ON_ERR_PAY_CRC,
           "corrupted payload rejected by CRC16");
}

/* 2. Test L1 Short CA-Polar (N=128, K=48) over clean + noisy AWGN channel */
static void test_ca_polar_control_plane(void) {
    printf("[TEST 2] L1 Short CA-Polar (N=128, K=48, CA-SCL L=8) Encode/Decode...\n");
    const uint8_t msg[ON_POLAR_MSG_BYTES] = {0x5A, 0xC3, 0x99, 0x01, 0xEF};
    uint8_t coded[ON_POLAR_N];
    EXPECT(on_polar_encode_block(msg, coded) == ON_OK, "CA-Polar encode OK");

    /* Clean channel test */
    float llr[ON_POLAR_N];
    for (uint32_t i = 0; i < ON_POLAR_N; ++i) {
        llr[i] = coded[i] ? -6.0f : +6.0f;
    }
    uint8_t rec[ON_POLAR_MSG_BYTES];
    EXPECT(on_polar_decode_block(llr, rec) == ON_OK, "CA-Polar clean decode OK");
    EXPECT(memcmp(msg, rec, ON_POLAR_MSG_BYTES) == 0, "CA-Polar recovered bytes match");

    /* Noisy AWGN channel test at SNR_symbol = 0.0 dB (Eb/N0 ~ 4.26 dB for rate 0.375) */
    double snr_lin = 1.0; /* 0 dB Es/N0 */
    double sigma = sqrt(1.0 / (2.0 * snr_lin));
    uint32_t trials = 100u;
    uint32_t ok_cnt = 0u;
    for (uint32_t t = 0; t < trials; ++t) {
        uint8_t tmsg[ON_POLAR_MSG_BYTES];
        for (int b = 0; b < 5; ++b) tmsg[b] = (uint8_t)((t * 37u + b * 19u) & 0xFFu);
        on_polar_encode_block(tmsg, coded);
        for (uint32_t i = 0; i < ON_POLAR_N; ++i) {
            double sym = coded[i] ? -1.0 : +1.0;
            double rx = sym + sigma * rand_gauss();
            llr[i] = (float)(2.0 * rx / (sigma * sigma));
        }
        if (on_polar_decode_block(llr, rec) == ON_OK && memcmp(tmsg, rec, 5u) == 0) {
            ok_cnt++;
        }
    }
    printf("    -> CA-Polar (N=128, rate=0.375) @ Es/N0=0dB: %u/%u blocks recovered\n", ok_cnt, trials);
    EXPECT(ok_cnt >= 95u, "CA-Polar recovers >= 95% blocks at 0 dB Es/N0");
}

/* 3. Test L3 RTT 4-Timestamp Calibration & TC-PLL / NCO Convergence + 10-Frame Coasting */
static void test_sync_rtt_and_nco_holdover(void) {
    printf("[TEST 3] L3 RTT 4-Timestamp Calibration + NCO TC-PLL + 10-Frame Coasting...\n");
    on_sync_ctx_t sync;
    on_sync_init(&sync);

    const double true_drift_ppm = +22.5;
    const int64_t true_link_delay_ns = 430;
    const int64_t boot_off_ns = 7654321;

    /* Run 15 superframes of SYNC beacons (4 ms each) */
    for (int sf = 0; sf < 15; ++sf) {
        int64_t t_tx_net = (int64_t)sf * 4000000LL;
        int64_t t_rx_phys = t_tx_net + true_link_delay_ns;
        int64_t t_rx_loc  = boot_off_ns + (int64_t)llround((double)t_rx_phys * (1.0 + true_drift_ppm * 1e-6));
        on_sync_on_beacon(&sync, t_tx_net, t_rx_loc);

        if (sf == 4) {
            /* Bidirectional 4-timestamp RTT exchange */
            int64_t t1_phys = t_tx_net + 3200000LL;
            int64_t t2_phys = t1_phys + true_link_delay_ns;
            int64_t t3_phys = t2_phys + 25000LL;
            int64_t t4_phys = t3_phys + true_link_delay_ns;

            int64_t t1_loc = boot_off_ns + (int64_t)llround((double)t1_phys * (1.0 + true_drift_ppm * 1e-6));
            int64_t t4_loc = boot_off_ns + (int64_t)llround((double)t4_phys * (1.0 + true_drift_ppm * 1e-6));
            int64_t tau_est = 0;
            EXPECT(on_sync_on_rtt(&sync, t1_loc, t2_phys, t3_phys, t4_loc, &tau_est) == ON_OK,
                   "RTT calibration succeeds");
            EXPECT(llabs(tau_est - true_link_delay_ns) <= 5, "RTT delay within 5 ns of true 430 ns");
        }
    }

    /* Check NCO frequency estimate (should be ~ -22,500 ppb to cancel +22.5 ppm fast local crystal) */
    double rate_err_ppm = fabs(sync.rate_ppb * 1e-3 + true_drift_ppm);
    printf("    -> NCO rate=%.3f ppm (true cancel=%.3f ppm), RTT=%lld ns\n",
           sync.rate_ppb * 1e-3, -true_drift_ppm, (long long)sync.link_delay_ns);
    EXPECT(rate_err_ppm < 0.5, "NCO frequency locked within 0.5 ppm");

    /* Now coast 10 consecutive missed superframes (40 ms total blackout) */
    int64_t t_coast_net_true = 25LL * 4000000LL;
    int64_t t_coast_loc = boot_off_ns + (int64_t)llround((double)t_coast_net_true * (1.0 + true_drift_ppm * 1e-6));
    int64_t t_coast_est = on_sync_local_to_net(&sync, t_coast_loc);
    int64_t coast_err   = llabs(t_coast_est - t_coast_net_true);
    printf("    -> After 10 dropped superframes (40 ms holdover): skew = %lld ns\n", (long long)coast_err);
    EXPECT(coast_err < 200, "10-frame holdover skew < 200 ns");
}

/* 4. End-to-End 10-Node Multi-Hop + Sub-G + Holdover + Backup_ID Failover Integration Test */
static void test_10node_multihop_and_failover(void) {
    printf("[TEST 4] End-to-End 10-Node Sub-us Sync, Zero-Collision Sub-G, Holdover & Backup_ID Failover...\n");
    sim_world_t w;
    sim_world_init(&w, 0x11223344ULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, false);

    /* Phase 1: Join + RTT + Sync (SF 0..34) */
    w.blackout_node_idx  = 3; /* Node T3 experiences 10-frame RF blackout (SF 35..44) */
    w.drop_sync_start_sf = 35u;
    w.drop_sync_end_sf   = 44u;
    sim_world_run_us(&w, 140000, 10);

    /* Verify all 10 nodes joined (including wall-blocked T8 & T9 via Sub-G T2!) */
    uint32_t joined_cnt = 0u;
    for (uint32_t i = 0; i < w.num_nodes; ++i) {
        if (w.nodes[i].role != ON_ROLE_UNJOINED && w.nodes[i].short_id != ON_SHORT_ID_UNASSIGNED) {
            joined_cnt++;
        }
    }
    EXPECT(joined_cnt == 10u, "All 10 nodes (incl. wall-blocked T8 & T9) joined dynamically via C-RA");
    EXPECT(w.nodes[2].role == ON_ROLE_SUB_G, "T2 promoted to Sub-G relay proxy");
    EXPECT(w.nodes[8].is_sub_node && w.nodes[9].is_sub_node, "T8 & T9 attached as 2-hop Sub-G edge nodes");
    EXPECT(w.tdma_subg_collisions == 0u, "Zero RF collisions in D-TDMA & Sub-G carved window");

    /* Phase 2: 10-frame SYNC blackout holdover (SF 35..44) */
    sim_world_run_us(&w, 40000, 10);
    EXPECT(w.sync_drops_injected == 10u, "Exactly 10 primary SYNC frames dropped during interference window");

    /* Phase 3: Recovery + C-RA Emergency Alert + Sudden G Kill -> Backup_ID Takeover */
    on_node_trigger_alert(&w.nodes[5], 0x99u);
    w.kill_node_idx = 0;
    w.kill_at_sf    = 65u;
    sim_world_run_us(&w, 180000, 10);

    EXPECT(w.alert_received_by_g, "Emergency C-RA alert from T5 delivered to G");
    EXPECT(w.edge_relay_received_by_g, "Wall-blocked 2-hop edge telemetry relayed via Sub-G T2 to G");
    EXPECT(w.failover_new_g_id == w.nodes[1].short_id, "T1 (Backup_ID) took over as Primary G");
    EXPECT(w.nodes[1].role == ON_ROLE_G, "T1 role is now ON_ROLE_G");
    EXPECT(w.failover_sf_seq == 69u, "Zero-vote failover happened exactly on 4th frame (SF 65 + 4 = 69)");
    EXPECT(w.tdma_subg_collisions == 0u, "Still ZERO D-TDMA / Sub-G RF collisions through failover");

    /* Verify true physical-time ACT skew across all alive nodes */
    int64_t max_steady = 0, max_holdover = 0, max_post_fail = 0;
    bool all_cmd_ok = true;
    for (uint32_t i = 0; i < w.act_hist_count; ++i) {
        const sim_act_record_t *r = &w.act_hist[i];
        if (r->sf_seq >= 25u && r->sf_seq <= 34u) {
            if (r->max_skew_ns > max_steady) max_steady = r->max_skew_ns;
            if (!r->cmd_consistent) all_cmd_ok = false;
        } else if (r->sf_seq >= 35u && r->sf_seq <= 44u) {
            if (r->max_skew_ns > max_holdover) max_holdover = r->max_skew_ns;
        } else if (r->sf_seq >= 72u && r->sf_seq <= 88u) {
            if (r->max_skew_ns > max_post_fail) max_post_fail = r->max_skew_ns;
            if (!r->cmd_consistent) all_cmd_ok = false;
        }
    }

    printf("    -> 10-Node Steady Skew = %lld ns | 10-SF Holdover = %lld ns | Post-Failover = %lld ns\n",
           (long long)max_steady, (long long)max_holdover, (long long)max_post_fail);
    EXPECT(max_steady > 0 && max_steady < 1000,    "Steady-state 10-node (incl. 2-hop) skew < 1.0 us");
    EXPECT(max_holdover > 0 && max_holdover < 1500,"10-frame blackout holdover skew < 1.5 us");
    EXPECT(max_post_fail > 0 && max_post_fail < 1000,"Post-failover 9-node skew < 1.0 us");
    EXPECT(all_cmd_ok, "Synchronized ACT LED command consistent across all nodes");

    /* Verify ultra-low-power duty cycle (1-hop & 2-hop terminals > 88% deep sleep) */
    double t6_duty = on_node_duty_cycle_pct(&w.nodes[6]);
    double t8_duty = on_node_duty_cycle_pct(&w.nodes[8]);
    printf("    -> Radio duty cycle: 1-hop T6 = %.2f%% | 2-hop T8 = %.2f%%\n", t6_duty, t8_duty);
    EXPECT(t6_duty < 10.0, "1-hop terminal T6 radio duty cycle < 10% (>90% deep sleep)");
    EXPECT(t8_duty < 10.0, "2-hop edge T8 radio duty cycle < 10% (>90% deep sleep)");
}

/* 5. Test AES-128 (NIST FIPS 197) + AES-128-CCM AEAD + Live Replay & Forgery Rejection */
static void test_aes128_ccm_security_and_antireplay(void) {
    printf("[TEST 5] AES-128 (FIPS 197) + AES-128-CCM AEAD + Live Replay/Forgery Rejection...\n");

    /* Official NIST FIPS 197 Appendix B test vector */
    const uint8_t fips_key[16] = {
        0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c
    };
    const uint8_t fips_in[16] = {
        0x32,0x43,0xf6,0xa8,0x88,0x5a,0x30,0x8d,0x31,0x31,0x98,0xa2,0xe0,0x37,0x07,0x34
    };
    const uint8_t fips_exp[16] = {
        0x39,0x25,0x84,0x1d,0x02,0xdc,0x09,0xfb,0xdc,0x11,0x85,0x97,0x19,0x6a,0x0b,0x32
    };
    uint8_t fips_out[16];
    on_aes128_encrypt_block(fips_key, fips_in, fips_out);
    EXPECT(memcmp(fips_out, fips_exp, 16u) == 0, "AES-128 matches NIST FIPS 197 Appendix B vector");

    /* AES-128-CCM unit round-trip + AAD tampering detection */
    uint8_t nonce[ON_SEC_NONCE_LEN] = { 1,2,3,4,5,6,7,8,9,10,11,12,13 };
    uint8_t aad[6] = { 0x4A, 0x00, 0x20, 0x01, 0x34, 0x99 };
    const uint8_t plain[10] = { 0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xA0 };
    uint8_t cipher[10], dec[10], mic[ON_SEC_MIC_LEN];

    EXPECT(on_ccm_encrypt(fips_key, nonce, aad, 6u, plain, 10u, cipher, mic) == ON_OK,
           "AES-128-CCM encrypt OK");
    EXPECT(memcmp(cipher, plain, 10u) != 0, "Ciphertext differs from plaintext");
    EXPECT(on_ccm_decrypt(fips_key, nonce, aad, 6u, cipher, 10u, mic, dec) == ON_OK,
           "AES-128-CCM decrypt OK");
    EXPECT(memcmp(dec, plain, 10u) == 0, "Decrypted plaintext matches original");

    /* Tamper 1 bit in 6-byte MAC header AAD -> MUST fail MIC */
    aad[1] ^= 0x01u;
    EXPECT(on_ccm_decrypt(fips_key, nonce, aad, 6u, cipher, 10u, mic, dec) == ON_ERR_DECODE_FAIL,
           "1-bit AAD header tamper rejected by CCM MIC");

    /* Live 10-node encrypted network + active replay & header forgery injection */
    sim_world_t w;
    sim_world_init(&w, 0x5EC02026ULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, true);
    for (uint32_t i = 0; i < w.num_nodes; ++i) {
        on_node_enable_security(&w.nodes[i], fips_key);
    }

    sim_world_run_us(&w, 80000, 10); /* 20 superframes */
    EXPECT(w.nodes[0].tdma_rx_total >= 100u, "Encrypted 10-node TDMA uplinks decrypted & accepted by G");
    EXPECT(w.captured_sec_pkt, "Captured live encrypted TDMA_DATA frame on wire");
    EXPECT(w.nodes[0].sec.auth_failures == 0u, "Zero auth failures during clean encrypted operation");
    EXPECT(w.nodes[0].sec.replay_drops == 0u, "Zero false replay drops during clean operation");

    /* Attack 1: Replay freshly captured valid encrypted packet directly into G */
    int64_t g_now = sim_phys_to_local_ns(&w, 0u, w.phys_now_ns);
    on_node_on_rx_raw(&w.nodes[0], w.cap_frame, w.cap_len, g_now);
    EXPECT(w.nodes[0].sec.replay_drops == 1u,
           "Replayed encrypted packet strictly rejected by monotonic anti-replay window");

    /* Attack 2: Forge a packet with tampered Short-ID (even after recomputing valid CRC8 + CRC16!) */
    uint8_t forged[ON_MAX_FRAME_LEN];
    memcpy(forged, w.cap_frame, w.cap_len);
    forged[1] ^= 0x01u; /* tamper src_id */
    forged[5] = on_crc8(forged, 5u); /* attacker recomputes valid Hdr_CRC8! */
    uint32_t auth_before = w.nodes[0].sec.auth_failures;
    on_node_on_rx_raw(&w.nodes[0], forged, w.cap_len, g_now);
    EXPECT(w.nodes[0].sec.auth_failures == auth_before + 1u,
           "Re-CRC8 forged MAC header rejected by AES-128-CCM AAD authentication");
    printf("    -> Encrypted RX=%u | ReplayDrops=%u | AuthFailures(attack)=%u\n",
           w.nodes[0].tdma_rx_total, w.nodes[0].sec.replay_drops, w.nodes[0].sec.auth_failures);
}

/* 6. Test TSCH Adaptive Frequency Hopping (AFH) under Wi-Fi CH6 Co-Channel Jammer */
static void test_tsch_afh_wifi_jammer_blacklisting(void) {
    printf("[TEST 6] TSCH Adaptive Frequency Hopping (AFH) + Live Wi-Fi CH6 Jammer Blacklisting...\n");
    sim_world_t w;
    sim_world_init(&w, 0xAF42026ULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, true);
    for (uint32_t i = 0; i < w.num_nodes; ++i) {
        on_node_enable_afh(&w.nodes[i], true);
    }

    /* Warm up 15 superframes cleanly so all 10 nodes (incl. 2-hop T8/T9) lock & join */
    sim_world_run_us(&w, 60000, 10);

    /* Turn on Wi-Fi CH6 20 MHz jammer wiping out logical channels 4, 5, 6, 7 */
    w.jammed_ch_mask = (uint16_t)((1u << 4) | (1u << 5) | (1u << 6) | (1u << 7));
    sim_world_run_us(&w, 440000, 10); /* 110 superframes (~440 ms) for detection & full blacklist */

    uint16_t g_map = w.nodes[0].afh.channel_map;
    uint16_t t4_map = w.nodes[4].afh.channel_map;
    uint16_t t8_map = w.nodes[8].afh.channel_map;
    uint32_t drops_before_clean_window = w.jammed_drops;

    /* Run another 25 superframes (100 ms) AFTER blacklisting -> must suffer ZERO jammed drops! */
    sim_world_run_us(&w, 100000, 10);
    uint32_t drops_after_blacklist = w.jammed_drops - drops_before_clean_window;

    int64_t post_afh_max_skew = 0;
    for (uint32_t i = 0; i < w.act_hist_count; ++i) {
        if (w.act_hist[i].sf_seq >= 125u && w.act_hist[i].sf_seq <= 145u) {
            if (w.act_hist[i].max_skew_ns > post_afh_max_skew) {
                post_afh_max_skew = w.act_hist[i].max_skew_ns;
            }
        }
    }

    printf("    -> G map=0x%04X | T4 map=0x%04X | 2-Hop T8 map=0x%04X | Post-AFH JammedDrops=%u | Skew=%lld ns\n",
           g_map, t4_map, t8_map, drops_after_blacklist, (long long)post_afh_max_skew);

    EXPECT((g_map & w.jammed_ch_mask) == 0u, "Primary G blacklisted all 4 Wi-Fi jammed channels (CH4..CH7)");
    EXPECT(t4_map == g_map, "1-hop T4 synchronized exact AFH channel_map via SYNC/MAP");
    EXPECT(t8_map == g_map, "2-hop wall-blocked T8 synchronized exact AFH channel_map via Sub-G");
    EXPECT(drops_after_blacklist == 0u, "ZERO jammed packets after AFH migrated network to clean channels");
    EXPECT(post_afh_max_skew > 0 && post_afh_max_skew < 1000,
           "10-node sub-us ACT sync maintained (< 1.0 us) across TSCH frequency hopping");
}

/* 7. Test Piggyback Group-ACK Slot-Local Fast HARQ Recovery */
static void test_piggyback_group_ack_fast_harq(void) {
    printf("[TEST 7] Piggyback Group-ACK Bitmap & Slot-Local Fast HARQ Recovery...\n");
    sim_world_t w;
    sim_world_init(&w, 0x4A902026ULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, true);

    /* Inject a single impulse RF drop on T4's D-TDMA uplink at SF=15 */
    w.impulse_drop_node_idx = 4;
    w.impulse_drop_sf       = 15u;
    sim_world_run_us(&w, 100000, 10); /* 25 superframes */

    printf("    -> T4 HARQ Retransmissions=%u | HARQ Recovered=%u\n",
           w.nodes[4].harq_retx_total, w.nodes[4].harq_recovered_total);
    EXPECT(w.nodes[4].harq_retx_total >= 1u, "T4 detected missing Group-ACK bit and triggered HARQ retry");
    EXPECT(w.nodes[4].harq_recovered_total >= 1u, "T4 HARQ retransmission confirmed ACKed on next superframe");
}

/* 8. Test v3.0 Beacon Anti-Spoofing (Rogue G Rejection) + Session Salt + Online Key Rotation */
static void test_v3_beacon_auth_and_key_rotation(void) {
    printf("[TEST 8] v3.0 Beacon Anti-Spoofing (CBC-MAC) + Session Salt + Live Key Rotation...\n");
    const uint8_t master_key[16] = {
        0x10,0x21,0x32,0x43,0x54,0x65,0x76,0x87,0x98,0xA9,0xBA,0xCB,0xDC,0xED,0xFE,0x0F
    };
    sim_world_t w;
    sim_world_init(&w, 0xB3AC02026ULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, true);
    for (uint32_t i = 0; i < w.num_nodes; ++i) {
        on_node_enable_security(&w.nodes[i], master_key);
    }
    on_node_set_session_salt(&w.nodes[0], 0xABCD1234UL);
    sim_world_run_us(&w, 40000, 10); /* 10 superframes */

    EXPECT(w.nodes[4].sec.session_salt == 0xABCD1234UL, "T4 adopted G's per-boot session_salt");
    EXPECT(w.nodes[4].sec.beacon_auth_drops == 0u, "Zero false beacon drops during clean run");

    /* Inject a spoofed Rogue G SYNC_MAP beacon into T4 with forged act_cmd & bad beacon_mac */
    on_sync_payload_t spoof;
    memset(&spoof, 0, sizeof(spoof));
    spoof.sf_seq       = w.nodes[4].current_sf_seq + 1u;
    spoof.t_tx_net_ns  = w.nodes[4].current_sf_start_net_ns + 4000000LL;
    spoof.t_sf_us      = ON_DEFAULT_TSF_US;
    spoof.session_salt = 0xABCD1234UL;
    spoof.beacon_mac[0] = 0xDEu; spoof.beacon_mac[1] = 0xADu;
    uint8_t sp_buf[ON_SYNC_PAYLOAD_WIRE_LEN];
    size_t sp_len = 0u;
    on_sync_payload_pack(&spoof, sp_buf, sizeof(sp_buf), &sp_len);

    on_mac_hdr_t shdr = { ON_PROTO_VERSION, ON_FRAME_SYNC_MAP, false, false,
                          ON_SHORT_ID_G_DEFAULT, ON_SHORT_ID_BROADCAST,
                          (uint8_t)(spoof.sf_seq & 0x0Fu), 0u };
    uint8_t wire[ON_MAX_FRAME_LEN];
    size_t wlen = 0u;
    on_frame_pack(&shdr, sp_buf, sp_len, wire, sizeof(wire), &wlen);

    int64_t t4_now = sim_phys_to_local_ns(&w, 4u, w.phys_now_ns);
    on_node_on_rx_raw(&w.nodes[4], wire, wlen, t4_now);
    EXPECT(w.nodes[4].sec.beacon_auth_drops == 1u, "Spoofed Rogue G beacon rejected by 32-bit CBC-MAC");

    /* Live online key epoch rotation: rotate on G -> all nodes derive new session key seamlessly */
    uint32_t rx_before = w.nodes[0].tdma_rx_total;
    on_node_rotate_key_epoch(&w.nodes[0]);
    sim_world_run_us(&w, 40000, 10);
    EXPECT(w.nodes[4].sec.key_epoch == 1u, "T4 seamlessly rotated to key_epoch=1");
    EXPECT(w.nodes[0].tdma_rx_total > rx_before + 40u, "Encrypted TDMA traffic continued smoothly post-rotation");
    EXPECT(w.nodes[0].sec.auth_failures == 0u, "Zero decryption failures during live key rotation");
    printf("    -> BeaconSpoofDrops=%u | RotatedEpoch=%u | PostRotRX=%u\n",
           w.nodes[4].sec.beacon_auth_drops, w.nodes[4].sec.key_epoch,
           w.nodes[0].tdma_rx_total - rx_before);
}

/* 9. Test v3.0 Peer Lease Expiry GC + Sub-G Relay Failover + 2-Hop Edge Re-Attach */
static void test_v3_lease_gc_and_subg_failover(void) {
    printf("[TEST 9] v3.0 Peer Lease GC + Sub-G Proxy Failover + 2-Hop Edge Re-Attach...\n");
    sim_world_t w;
    sim_world_init(&w, 0x50B62026ULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, true);

    /* Make T3 also Sub-G capable & reachable by wall-blocked edge nodes T8, T9 as standby relay */
    w.nodes[3].capabilities |= 0x02u;
    w.nodes[0].peers[2].capabilities |= 0x02u; /* T3 is peer index 2 (T1=0, T2=1, T3=2) */
    w.can_hear[3][8] = w.can_hear[8][3] = true; w.link_delay_ns[3][8] = w.link_delay_ns[8][3] = 360;
    w.can_hear[3][9] = w.can_hear[9][3] = true; w.link_delay_ns[3][9] = w.link_delay_ns[9][3] = 380;

    /* Run 20 SFs so T8 & T9 join via original Sub-G T2 */
    sim_world_run_us(&w, 80000, 10);
    EXPECT(w.nodes[8].role == ON_ROLE_T && w.nodes[8].parent_id == w.nodes[2].short_id,
           "T8 initially attached behind Sub-G proxy T2");

    /* Abruptly kill Sub-G proxy T2! */
    w.alive[2] = false;
    sim_world_run_us(&w, 260000, 10); /* 65 SFs: T2 lease expires -> G promotes T3 -> T8 re-joins T3 */

    printf("    -> LeaseReclaims=%u | SubGFailovers=%u | NewSubG=0x%03X | T8 Parent=0x%03X\n",
           w.nodes[0].lease_reclaim_count, w.nodes[0].subg_failover_count,
           w.nodes[0].sub_g_id, w.nodes[8].parent_id);
    EXPECT(w.nodes[0].lease_reclaim_count >= 1u, "Primary G reclaimed dead T2 slot via Lease GC");
    EXPECT(w.nodes[0].subg_failover_count >= 1u, "Primary G triggered automatic Sub-G re-election");
    EXPECT(w.nodes[3].role == ON_ROLE_SUB_G, "T3 automatically promoted to new Sub-G relay proxy");
    EXPECT(w.nodes[8].role == ON_ROLE_T && w.nodes[8].parent_id == w.nodes[3].short_id,
           "Wall-blocked 2-hop T8 detached from dead T2 and re-attached to new Sub-G T3");
}

/* 10. Test v3.0 AFH Cold-Boot Unjoined Channel Discovery Scanner */
static void test_v3_afh_cold_boot_scan(void) {
    printf("[TEST 10] v3.0 AFH Cold-Boot Channel Discovery Scanner (starting on jammed CH5)...\n");
    sim_world_t w;
    sim_world_init(&w, 0x5CA42026ULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, true);
    /* Enable AFH on G + T1..T6, blacklist CH5 on G; keep T7 unjoined starting on dead CH5 */
    for (uint32_t i = 0; i < w.num_nodes; ++i) {
        on_node_enable_afh(&w.nodes[i], true);
    }
    w.jammed_ch_mask = (uint16_t)(1u << 5);
    on_afh_apply_map(&w.nodes[0].afh, (uint16_t)(~w.jammed_ch_mask));
    for (uint32_t i = 1; i <= 6; ++i) on_afh_apply_map(&w.nodes[i].afh, (uint16_t)(~w.jammed_ch_mask));

    /* Reset T7 to unjoined cold boot parked on jammed CH5 */
    w.nodes[7].role        = ON_ROLE_UNJOINED;
    w.nodes[7].short_id    = ON_SHORT_ID_UNASSIGNED;
    w.nodes[7].sync.locked = false;
    w.nodes[7].scan_ch_idx = 5u;
    w.cur_ch[7]            = 5u;

    sim_world_run_us(&w, 320000, 10); /* 80 SFs -> scanner sweeps off CH5, locks, joins via C-RA */
    printf("    -> Cold-boot T7 locked=%d role=%d short_id=0x%03X\n",
           w.nodes[7].sync.locked ? 1 : 0, (int)w.nodes[7].role, w.nodes[7].short_id);
    EXPECT(w.nodes[7].sync.locked, "Cold-boot T7 swept off jammed CH5 and locked to AFH SYNC/MAP");
    EXPECT(w.nodes[7].role == ON_ROLE_T && w.nodes[7].short_id != ON_SHORT_ID_UNASSIGNED,
           "Cold-boot T7 completed C-RA join under active TSCH frequency hopping");
}

/* 11. Test v3.0 Fixed-Point Q4.4 CA-Polar + AMC Dual-Waveform + SAR/QoS Preemption */
static void test_v3_fixed_polar_amc_and_sar_qos(void) {
    printf("[TEST 11] v3.0 Fixed-Point Q4.4 CA-Polar + AMC Hysteresis + SAR/Priority-QoS...\n");

    /* 11a. Fixed-point Q4.4 CA-Polar (zero float) */
    const uint8_t pmsg[ON_POLAR_MSG_BYTES] = { 0xA1, 0xB2, 0xC3, 0xD4, 0xE5 };
    uint8_t coded[ON_POLAR_N];
    int16_t llr_q4[ON_POLAR_N];
    uint8_t prec[ON_POLAR_MSG_BYTES];
    on_polar_encode_block(pmsg, coded);
    for (uint32_t i = 0; i < ON_POLAR_N; ++i) {
        llr_q4[i] = coded[i] ? (int16_t)-48 : (int16_t)+48;
    }
    EXPECT(on_polar_decode_block_fixed(llr_q4, prec) == ON_OK && memcmp(pmsg, prec, 5u) == 0,
           "Fixed-point Q4.4 CA-Polar decodes cleanly");

    /* 11b. AMC Dual-Waveform Hysteresis */
    on_amc_ctx_t amc;
    on_amc_init(&amc, ON_PHY_LE_MSK_2M);
    for (int i = 0; i < 12; ++i) on_amc_update(&amc, 12.0f, true);
    EXPECT(amc.current_mode == ON_PHY_LE_QPSK_4M, "AMC upgrades 2M MSK -> 4M QPSK at 12 dB");
    for (int i = 0; i < 12; ++i) on_amc_update(&amc, 22.0f, true);
    EXPECT(amc.current_mode == ON_PHY_HS_OFDM_20M, "AMC upgrades 4M QPSK -> 20M HS OFDM at 22 dB");
    for (int i = 0; i < 6; ++i) on_amc_update(&amc, 2.0f, false);
    EXPECT(amc.current_mode == ON_PHY_LE_MSK_2M, "AMC rapidly downgrades to robust 2M MSK on fade");

    /* 11c. SAR 300-byte OTA fragmentation + Priority-0 RT control preemption */
    on_sar_tx_t stx;
    on_sar_rx_t srx_ota, srx_rt;
    on_sar_tx_init(&stx);
    on_sar_rx_init(&srx_ota);
    on_sar_rx_init(&srx_rt);

    uint8_t ota_blob[300];
    for (size_t i = 0; i < sizeof(ota_blob); ++i) ota_blob[i] = (uint8_t)((i * 13u + 7u) & 0xFFu);
    const uint8_t rt_cmd[4] = { 0xFF, 0x00, 0xAA, 0x55 };

    /* Enqueue low-priority OTA FIRST, then high-priority RT control command */
    EXPECT(on_sar_tx_enqueue(&stx, ON_QOS_BULK_OTA, ota_blob, sizeof(ota_blob), NULL) == ON_OK,
           "SAR enqueued 300-byte OTA block (5 fragments)");
    EXPECT(on_sar_tx_enqueue(&stx, ON_QOS_CTRL_RT, rt_cmd, sizeof(rt_cmd), NULL) == ON_OK,
           "SAR enqueued Priority-0 real-time control command");

    uint8_t frag[80], rec_buf[ON_SAR_MAX_MSG_LEN];
    size_t flen = 0u, rlen = 0u;
    on_qos_prio_t first_prio = ON_QOS_BULK_OTA;
    EXPECT(on_sar_tx_pop(&stx, frag, sizeof(frag), &flen, &first_prio), "Pop first fragment");
    EXPECT(first_prio == ON_QOS_CTRL_RT, "Priority-0 RT control strictly preempted queued OTA bulk fragments");
    EXPECT(on_sar_rx_push(&srx_rt, frag, flen, rec_buf, sizeof(rec_buf), &rlen) &&
           rlen == 4u && memcmp(rec_buf, rt_cmd, 4u) == 0,
           "RT control payload reassembled intact");

    bool ota_done = false;
    on_qos_prio_t p;
    while (on_sar_tx_pop(&stx, frag, sizeof(frag), &flen, &p)) {
        if (on_sar_rx_push(&srx_ota, frag, flen, rec_buf, sizeof(rec_buf), &rlen)) {
            ota_done = true;
        }
    }
    EXPECT(ota_done && rlen == sizeof(ota_blob) && memcmp(rec_buf, ota_blob, sizeof(ota_blob)) == 0,
           "300-byte OTA payload reassembled bit-exact across 5 SAR fragments");
}

/* 12. Deep Audit Regression: Sub-G Multi-Edge FIFO + Sub-Slot Lease GC + Backup_ID Dynamic Mirror + Auth Join */
static void test_v3_deep_audit_regressions(void) {
    printf("[TEST 12] Deep Audit Regressions: Sub-G FIFO + Sub-Slot GC + Backup_ID Mirror + Auth Join...\n");

    const uint8_t net_key[16] = {
        0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99
    };
    const uint8_t wrong_key[16] = {
        0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10
    };

    sim_world_t w;
    sim_world_init(&w, 0xA0D172026ULL);
    w.verbose = false;
    /* Start with 100% dynamic C-RA join (pre_registered_fast = false) + full network security */
    sim_world_setup_10node_demo(&w, false);
    for (uint32_t i = 0; i < w.num_nodes; ++i) {
        on_node_enable_security(&w.nodes[i], net_key);
    }
    on_node_set_session_salt(&w.nodes[0], 0x2026F00DUL);

    /* Run 25 SFs so all 10 nodes (1-hop T1..T7 and 2-hop T8..T9) join dynamically under AES-128-CCM */
    sim_world_run_us(&w, 100000, 10);
    EXPECT(w.nodes[8].role == ON_ROLE_T && w.nodes[9].role == ON_ROLE_T,
           "Both 2-hop edge nodes T8 & T9 joined via encrypted Sub-C-RA");

    /* Verify Sub-G 4-slot FIFO relay ring queued & drained telemetry without losing T8 or T9 */
    EXPECT(w.nodes[2].peers[0].rx_packets >= 5u && w.nodes[2].peers[1].rx_packets >= 5u,
           "Sub-G T2 received multiple uplinks from both T8 (sub-slot 0) and T9 (sub-slot 1)");

    /* Verify Backup_ID (T1_BK, node 1) passively mirrored 1-hop peers during its periodic snoop window */
    uint32_t t1_mirrored_peers = 0u;
    bool mirrored_subg_cap = false;
    for (uint32_t i = 0; i < ON_MAX_NODES_TABLE; ++i) {
        if (w.nodes[1].peers[i].active) {
            t1_mirrored_peers++;
            if (w.nodes[1].peers[i].capabilities & 0x02u) mirrored_subg_cap = true;
        }
    }
    EXPECT(t1_mirrored_peers >= 5u, "Dynamic Backup_ID (T1_BK) passively mirrored >= 5 active 1-hop peers");
    EXPECT(mirrored_subg_cap, "Dynamic Backup_ID mirrored T2_SG's Sub-G capability bit from TDMA flags");

    /* Kill 2-hop edge node T9 -> verify Sub-G T2 reclaims T9's sub-slot via Sub-G Lease GC! */
    w.alive[9] = false;
    sim_world_run_us(&w, 160000, 10); /* 40 SFs > ON_PEER_LEASE_TIMEOUT_SF (30) */
    EXPECT(w.nodes[2].lease_reclaim_count >= 1u,
           "Sub-G T2 reclaimed dead 2-hop edge node T9's sub-slot via Sub-Slot Lease GC");
    EXPECT((w.nodes[2].sub_tdma_bitmap & (1u << 1)) == 0u,
           "Sub-G T2 cleared sub-slot 1 bit in sub_tdma_bitmap after T9 lease expiry");

    /* Inject unauthorized JOIN_REQ encrypted with wrong_key -> G MUST reject via CCM MIC! */
    on_node_t rogue;
    on_hal_ops_t dummy_hal;
    memset(&dummy_hal, 0, sizeof(dummy_hal));
    on_node_init(&rogue, &dummy_hal, ON_ROLE_UNJOINED, 0u);
    on_node_enable_security(&rogue, wrong_key);
    on_node_set_session_salt(&rogue, 0x2026F00DUL);

    on_join_req_t jreq = { {0xDE,0xAD,0xBE,0xEF,0x00,0x99}, 0u, 0u };
    uint8_t jpay[ON_JOIN_REQ_WIRE_LEN], jsec[32], jwire[ON_MAX_FRAME_LEN], aad6[6], nonce[13];
    size_t jlen = 0u, wlen = 0u;
    on_join_req_pack(&jreq, jpay, sizeof(jpay), &jlen);
    on_mac_hdr_t jhdr = { ON_PROTO_VERSION, ON_FRAME_JOIN_REQ, true, false,
                          ON_SHORT_ID_UNASSIGNED, ON_SHORT_ID_G_DEFAULT,
                          (uint8_t)(w.nodes[0].current_sf_seq & 0x0Fu), 0u };
    on_hdr_pack(&jhdr, aad6);
    on_sec_build_nonce_salted(&jhdr, w.nodes[0].current_sf_seq, rogue.sec.session_salt, nonce);
    on_ccm_encrypt(rogue.sec.key, nonce, aad6, 6u, jpay, jlen, jsec, jsec + jlen);
    on_frame_pack(&jhdr, jsec, jlen + ON_SEC_MIC_LEN, jwire, sizeof(jwire), &wlen);

    uint32_t auth_before = w.nodes[0].sec.auth_failures;
    on_node_on_rx_raw(&w.nodes[0], jwire, wlen, sim_phys_to_local_ns(&w, 0u, w.phys_now_ns));
    EXPECT(w.nodes[0].sec.auth_failures == auth_before + 1u,
           "Unauthorized JOIN_REQ with wrong key rejected by Primary G");
    printf("    -> BackupMirroredPeers=%u | SubGLeaseReclaims=%u | RogueJoinRejected=YES\n",
           t1_mirrored_peers, w.nodes[2].lease_reclaim_count);
}

int main(void) {
    printf("============================================================\n");
    printf("       OpenNear v3.0 Core Protocol Automated Test Suite     \n");
    printf("============================================================\n");

    test_6byte_header_and_frame();
    test_ca_polar_control_plane();
    test_sync_rtt_and_nco_holdover();
    test_10node_multihop_and_failover();
    test_aes128_ccm_security_and_antireplay();
    test_tsch_afh_wifi_jammer_blacklisting();
    test_piggyback_group_ack_fast_harq();
    test_v3_beacon_auth_and_key_rotation();
    test_v3_lease_gc_and_subg_failover();
    test_v3_afh_cold_boot_scan();
    test_v3_fixed_polar_amc_and_sar_qos();
    test_v3_deep_audit_regressions();

    printf("\n------------------------------------------------------------\n");
    printf("SUMMARY: %d passed, %d failed\n", g_tests_passed, g_tests_failed);
    printf("------------------------------------------------------------\n");
    return (g_tests_failed == 0) ? 0 : 1;
}
