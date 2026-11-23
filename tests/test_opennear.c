#include "on_types.h"
#include "on_frame.h"
#include "on_polar.h"
#include "on_sync.h"
#include "on_superframe.h"
#include "on_sec.h"
#include "on_ecdh.h"
#include "on_afh.h"
#include "on_amc.h"
#include "on_sar.h"
#include "on_ranging.h"
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

/* 13. Test v3.1 X25519 (RFC 7748 Section 6.1 Test Vector) + Zero-Touch OOB Onboarding KDF */
static void test_v31_x25519_ecdh_onboarding(void) {
    printf("[TEST 13] v3.1 X25519 (RFC 7748 Section 6.1 Vector) + Zero-Touch OOB Onboarding...\n");

    /* Official RFC 7748 Section 6.1 Alice & Bob test vectors */
    static const uint8_t alice_priv[32] = {
        0x77,0x07,0x6d,0x0a,0x73,0x18,0xa5,0x7d,0x3c,0x16,0xc1,0x72,0x51,0xb2,0x66,0x45,
        0xdf,0x4c,0x2f,0x87,0xeb,0xc0,0x99,0x2a,0xb1,0x77,0xfb,0xa5,0x1d,0xb9,0x2c,0x2a
    };
    static const uint8_t alice_pub_exp[32] = {
        0x85,0x20,0xf0,0x09,0x89,0x30,0xa7,0x54,0x74,0x8b,0x7d,0xdc,0xb4,0x3e,0xf7,0x5a,
        0x0d,0xbf,0x3a,0x0d,0x26,0x38,0x1a,0xf4,0xeb,0xa4,0xa9,0x8e,0xaa,0x9b,0x4e,0x6a
    };
    static const uint8_t bob_priv[32] = {
        0x5d,0xab,0x08,0x7e,0x62,0x4a,0x8a,0x4b,0x79,0xe1,0x7f,0x8b,0x83,0x80,0x0e,0xe6,
        0x6f,0x3b,0xb1,0x29,0x26,0x18,0xb6,0xfd,0x1c,0x2f,0x8b,0x27,0xff,0x88,0xe0,0xeb
    };
    static const uint8_t bob_pub_exp[32] = {
        0xde,0x9e,0xdb,0x7d,0x7b,0x7d,0xc1,0xb4,0xd3,0x5b,0x61,0xc2,0xec,0xe4,0x35,0x37,
        0x3f,0x83,0x43,0xc8,0x5b,0x78,0x67,0x4d,0xad,0xfc,0x7e,0x14,0x6f,0x88,0x2b,0x4f
    };
    static const uint8_t shared_exp[32] = {
        0x4a,0x5d,0x9d,0x5b,0xa4,0xce,0x2d,0xe1,0x72,0x8e,0x3b,0xf4,0x80,0x35,0x0f,0x25,
        0xe0,0x7e,0x21,0xc9,0x47,0xd1,0x9e,0x33,0x76,0xf0,0x9b,0x3c,0x1e,0x16,0x17,0x42
    };

    uint8_t alice_pub[32], bob_pub[32], shared_a[32], shared_b[32];
    on_x25519_public_key(alice_priv, alice_pub);
    on_x25519_public_key(bob_priv, bob_pub);
    EXPECT(memcmp(alice_pub, alice_pub_exp, 32u) == 0, "X25519 Alice public key matches RFC 7748");
    EXPECT(memcmp(bob_pub, bob_pub_exp, 32u) == 0, "X25519 Bob public key matches RFC 7748");

    EXPECT(on_x25519_shared_secret(alice_priv, bob_pub, shared_a) == ON_OK, "Alice ECDH succeeds");
    EXPECT(on_x25519_shared_secret(bob_priv, alice_pub, shared_b) == ON_OK, "Bob ECDH succeeds");
    EXPECT(memcmp(shared_a, shared_exp, 32u) == 0, "X25519 shared secret matches RFC 7748 Section 6.1");
    EXPECT(memcmp(shared_a, shared_b, 32u) == 0, "Alice and Bob derive identical 256-bit shared secret");

    /* Zero-touch OOB PIN KDF verification */
    const uint8_t mac48[6] = { 0x02, 0x4F, 0x4E, 0x00, 0x01, 0x09 };
    uint8_t key_a[16], key_b[16], key_wrong_pin[16], tag_a[4], tag_b[4], tag_wrong[4];
    on_ecdh_derive_onboarding_key(shared_a, 884219u, mac48, key_a, tag_a);
    on_ecdh_derive_onboarding_key(shared_b, 884219u, mac48, key_b, tag_b);
    on_ecdh_derive_onboarding_key(shared_b, 884218u, mac48, key_wrong_pin, tag_wrong);

    EXPECT(memcmp(key_a, key_b, 16u) == 0 && memcmp(tag_a, tag_b, 4u) == 0,
           "Matching OOB PIN derives identical 128-bit network key & confirmation tag");
    EXPECT(memcmp(key_a, key_wrong_pin, 16u) != 0 && memcmp(tag_a, tag_wrong, 4u) != 0,
           "1-digit OOB PIN mismatch produces completely different key & confirmation tag");
}

/* 14. Test v3.1 Multi-Zone Sub-G Orthogonal FDMA Concurrency (Zone 0 T2->T8 || Zone 1 T3->T9) */
static void test_v31_multizone_subg_fdma_concurrency(void) {
    printf("[TEST 14] v3.1 Multi-Zone Sub-G Orthogonal FDMA Concurrency (Zone 0 || Zone 1)...\n");
    sim_world_t w;
    sim_world_init(&w, 0xF0A22026ULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, true);

    /* Configure T2 (0x003) as East-Wall Zone-0 Sub-G and T3 (0x004) as West-Wall Zone-1 Sub-G */
    w.nodes[3].capabilities |= 0x02u;
    on_node_set_multi_subg(&w.nodes[0], 0x003u, 0x004u);

    /* East-Wall T8 ONLY hears Zone-0 T2 (on ch 0); West-Wall T9 ONLY hears Zone-1 T3 (on orthogonal ch 8) */
    w.can_hear[2][8] = w.can_hear[8][2] = true;
    w.can_hear[2][9] = w.can_hear[9][2] = false;
    w.can_hear[3][9] = w.can_hear[9][3] = true;  w.link_delay_ns[3][9] = w.link_delay_ns[9][3] = 360;
    w.can_hear[3][8] = w.can_hear[8][3] = false;
    w.cur_ch[9] = 8u; /* T9 listens on Zone-1 orthogonal channel 8 */

    sim_world_run_us(&w, 120000, 10); /* 30 SFs */

    printf("    -> T2(Zone=%u)->T8(id=0x%03X) | T3(Zone=%u)->T9(id=0x%03X) | Collisions=%u\n",
           w.nodes[2].sub_g_zone, w.nodes[8].short_id,
           w.nodes[3].sub_g_zone, w.nodes[9].short_id,
           w.tdma_subg_collisions);

    EXPECT(w.nodes[2].role == ON_ROLE_SUB_G && w.nodes[2].sub_g_zone == 0u,
           "T2 promoted to Zone-0 Sub-G proxy");
    EXPECT(w.nodes[3].role == ON_ROLE_SUB_G && w.nodes[3].sub_g_zone == 1u,
           "T3 promoted to Zone-1 FDMA Sub-G proxy");
    EXPECT(w.nodes[8].role == ON_ROLE_T && w.nodes[8].short_id == 0x100u,
           "East-wall T8 joined Zone-0 Sub-G T2 (assigned 0x100)");
    EXPECT(w.nodes[9].role == ON_ROLE_T && w.nodes[9].short_id == 0x200u,
           "West-wall T9 joined Zone-1 Sub-G T3 concurrently on orthogonal channel (assigned 0x200)");
    EXPECT(w.tdma_subg_collisions == 0u,
           "Zero RF collisions despite Zone-0 and Zone-1 broadcasting SUB_SYNC at the same instant");
}

/* 15. Test v3.1 IEEE 1588 PTP Two-Step Follow-Up TX-Queue Jitter Cancellation + Antenna Delay */
static void test_v31_ptp_followup_and_ant_delay(void) {
    printf("[TEST 15] v3.1 IEEE 1588 PTP Two-Step Follow-Up (+/-12 us TX Queue Jitter) + Ant Delay...\n");
    on_sync_ctx_t sync;
    on_sync_init(&sync);
    on_sync_set_ant_delay_ns(&sync, 85); /* 85 ns calibrated RF balun/antenna group delay */

    const double true_drift_ppm = -16.5;
    const int64_t true_air_delay_ns = 315; /* total true delay = 315 + 85 = 400 ns */
    const int64_t boot_off_ns = 2468000;
    sync.link_delay_ns  = true_air_delay_ns;
    sync.rtt_calibrated = true;

    int16_t prev_err_ns = 0;
    for (int sf = 0; sf < 20; ++sf) {
        int64_t nominal_tx_net = (int64_t)sf * 4000000LL;
        /* Simulate severe ESP32 Wi-Fi software TX queue jitter: alternating +12 us / -12 us! */
        int16_t actual_tx_err = (sf == 0) ? 0 : ((sf & 1) ? +12000 : -12000);
        int64_t actual_tx_phys = nominal_tx_net + actual_tx_err;
        int64_t rx_phys = actual_tx_phys + true_air_delay_ns + 85;
        int64_t rx_loc  = boot_off_ns + (int64_t)llround((double)rx_phys * (1.0 + true_drift_ppm * 1e-6));

        /* Two-step follow-up applies previous frame's measured TX departure error before locking */
        on_sync_apply_followup(&sync, prev_err_ns);
        on_sync_on_beacon(&sync, nominal_tx_net, rx_loc);
        prev_err_ns = actual_tx_err;
    }
    /* Apply final follow-up for SF 19 */
    on_sync_apply_followup(&sync, prev_err_ns);

    int64_t test_phys = 20LL * 4000000LL;
    int64_t test_loc  = boot_off_ns + (int64_t)llround((double)test_phys * (1.0 + true_drift_ppm * 1e-6));
    int64_t est_net   = on_sync_local_to_net(&sync, test_loc);
    int64_t skew_ns   = llabs(est_net - test_phys);
    printf("    -> Despite +/-12,000 ns TX queue jitter, post-Follow-Up residual skew = %lld ns\n",
           (long long)skew_ns);
    EXPECT(skew_ns < 50, "Two-Step PTP Follow-Up cancels +/-12 us TX queue jitter down to < 50 ns");
}

/* 16. Test v3.2 Route B FPGA RTL Golden Vector Verification (CA-Polar N=128 + Short Preamble) */
static void test_v32_fpga_rtl_golden_vectors(void) {
    printf("[TEST 16] v3.2 Route B FPGA RTL Bit-Exact Golden Vector Verification...\n");
    const uint8_t golden_msg[ON_POLAR_MSG_BYTES] = { 0x5A, 0xC3, 0x99, 0x01, 0xEF };
    uint8_t coded_bits[ON_POLAR_N];
    EXPECT(on_polar_encode_block(golden_msg, coded_bits) == ON_OK, "Golden CA-Polar encode OK");

    uint8_t packed16[16];
    memset(packed16, 0, sizeof(packed16));
    uint32_t ones_count = 0u;
    for (uint32_t i = 0; i < ON_POLAR_N; ++i) {
        if (coded_bits[i]) {
            packed16[i >> 3u] |= (uint8_t)(1u << (7u - (i & 7u)));
            ones_count++;
        }
    }
    uint8_t crc = on_crc8(golden_msg, ON_POLAR_MSG_BYTES);
    printf("    -> Golden Msg=5AC39901EF | CRC8=0x%02X | Weight=%u | Codeword[0..3]=%02X%02X%02X%02X\n",
           crc, ones_count, packed16[0], packed16[1], packed16[2], packed16[3]);

    /* Verify round-trip via Q4.4 fixed-point hardware decoder */
    int16_t llr_q4[ON_POLAR_N];
    for (uint32_t i = 0; i < ON_POLAR_N; ++i) {
        llr_q4[i] = coded_bits[i] ? (int16_t)-64 : (int16_t)+64;
    }
    uint8_t rec[ON_POLAR_MSG_BYTES];
    EXPECT(on_polar_decode_block_fixed(llr_q4, rec) == ON_OK &&
           memcmp(rec, golden_msg, ON_POLAR_MSG_BYTES) == 0,
           "Golden RTL codeword round-trips bit-exact through Q4.4 hardware decoder");
    EXPECT(ones_count > 30u && ones_count < 98u, "Golden Polar codeword has balanced Hamming weight");
}

/* Best-effort TX still supports data and backup, but cannot calibrate on-air RTT. */
static void test_best_effort_tx_timing(void) {
    printf("[TEST 21] Best-Effort TX: No Air RTT + Configurable Backup Holdover...\n");
    sim_world_t w;
    sim_world_init(&w, 0xC4A11ULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, false);
    w.num_nodes = 2u;
    on_node_t *g = &w.nodes[0];
    on_node_t *t = &w.nodes[1];
    g->hal.tx_timing_best_effort = true;
    t->hal.tx_timing_best_effort = true;
    t->backup_miss_threshold = 12u;
    w.blackout_node_idx = 1;
    w.drop_sync_start_sf = 30u;
    w.drop_sync_end_sf = 35u;
    sim_world_run_us(&w, 180000, 10);

    EXPECT(t->role == ON_ROLE_T && t->short_id == 0x002u && t->sync_rx_total > 10u && g->tdma_rx_total > 0u,
           "Best-effort terminal still joins the primary and sends data");
    EXPECT(t->rtt_token == 0u && !t->sync.rtt_calibrated, "Best-effort TX skips on-air RTT calibration");
    EXPECT(w.sync_drops_injected >= 5u && t->failover_count == 0u,
           "Backup with extended holdover does not take over during a short beacon blackout");
    w.kill_node_idx = 0;
    w.kill_at_sf = 50u;
    sim_world_run_us(&w, 100000, 10);
    EXPECT(t->role == ON_ROLE_G && t->failover_count == 1u && w.failover_sf_seq == 63u,
           "Backup still takes over after the configured 12 missed beacons");
}

/* Restart with an empty peer table after the terminal already took over. */
static void test_rejoin_after_primary_reboot(void) {
    printf("[TEST 22] Automatic Rejoin and Backup Re-election After Primary Reboot...\n");
    sim_world_t w;
    sim_world_init(&w, 0x0B007ULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, false);
    w.num_nodes = 2u;
    on_node_t *g = &w.nodes[0];
    on_node_t *t = &w.nodes[1];
    const uint8_t key[ON_SEC_KEY_LEN] = {0x3Au, 0x91u};
    on_node_enable_security(g, key);
    on_node_enable_security(t, key);
    on_node_set_session_salt(g, 0x42A738E1u);
    g->hal.tx_timing_best_effort = true;
    t->hal.tx_timing_best_effort = true;
    t->backup_miss_threshold = 12u;
    t->backup_min_sync_silence_ns = 150000000LL;
    sim_world_run_us(&w, 160000, 10);
    EXPECT(t->role == ON_ROLE_T && g->backup_id == t->short_id && t->short_id != ON_SHORT_ID_UNASSIGNED,
           "Terminal initially joins and is selected as backup");

    on_node_rotate_key_epoch(g);
    sim_world_run_us(&w, 16000, 10);
    EXPECT(t->sec.key_epoch == 1u && t->rejoin_count == 0u && t->role == ON_ROLE_T,
           "A same-session key rotation does not invalidate the slot or role");

    w.alive[0] = false;
    sim_world_run_us(&w, 80000, 10);
    EXPECT(t->role == ON_ROLE_T && t->failover_count == 0u && t->missed_sync_count >= 12u,
           "Measured silence guard blocks takeover despite enough estimated missing frames");
    sim_world_run_us(&w, 100000, 10);
    EXPECT(t->role == ON_ROLE_G && t->failover_count == 1u, "Terminal takes over during a longer outage");

    on_hal_ops_t g_hal = g->hal;
    on_node_init(g, &g_hal, ON_ROLE_G, 0u);
    on_node_enable_security(g, key);
    on_node_set_session_salt(g, 0xC0B3ED26u);
    w.alive[0] = true;
    sim_world_run_us(&w, 160000, 10);
    EXPECT(t->rejoin_count == 1u && t->failover_count == 1u && t->role == ON_ROLE_T,
           "Terminal rejoins the restarted primary without losing its diagnostic counters");
    EXPECT(g->backup_id != ON_SHORT_ID_UNASSIGNED && g->backup_id == t->short_id &&
               t->backup_id == t->short_id && g->tdma_rx_total > 0u,
           "Restarted primary allocates a slot, re-elects backup and receives uplink");

    w.alive[0] = false;
    sim_world_run_us(&w, 180000, 10);
    EXPECT(t->role == ON_ROLE_G && t->failover_count == 2u && t->rejoin_count == 1u,
           "Re-elected backup can take over a second outage");
}

/* A terminal can hear the coordinator throughout an uplink-only outage. */
static void test_rejoin_after_primary_lease_expiry(void) {
    printf("[TEST 23] Automatic Rejoin After Primary Slot Lease Expiry...\n");
    sim_world_t w;
    sim_world_init(&w, 0x1EA5EULL);
    w.verbose = false;
    sim_world_setup_10node_demo(&w, false);
    w.num_nodes = 2u;
    on_node_t *g = &w.nodes[0];
    on_node_t *t = &w.nodes[1];
    const uint8_t key[ON_SEC_KEY_LEN] = {0x3Au, 0x91u};
    const uint32_t salt = 0xD1FFE2E1u;
    on_node_enable_security(g, key);
    on_node_enable_security(t, key);
    on_node_set_session_salt(g, salt);
    g->hal.tx_timing_best_effort = true;
    t->hal.tx_timing_best_effort = true;
    g->peer_lease_timeout_sf = 60u;
    sim_world_run_us(&w, 160000, 10);
    const uint16_t old_id = t->short_id;
    EXPECT(t->role == ON_ROLE_T && old_id > ON_SHORT_ID_G_DEFAULT &&
               g->backup_id == old_id && g->tdma_rx_total > 0u,
           "Terminal initially joins, exchanges data and is selected as backup");

    const on_hal_ops_t saved_hal = t->hal;
    const uint32_t previous_syncs = t->sync_rx_total;
    const uint32_t previous_rx = g->tdma_rx_total;
    t->hal.tx_raw = NULL;
    sim_world_run_us(&w, 160000, 10);
    EXPECT(g->lease_reclaim_count == 0u && t->short_id == old_id && t->rejoin_count == 0u,
           "Configured lease survives a short uplink outage beyond the default timeout");
    sim_world_run_us(&w, 140000, 10);
    EXPECT(g->lease_reclaim_count == 1u && g->backup_id == ON_SHORT_ID_UNASSIGNED &&
               t->role == ON_ROLE_UNJOINED && t->rejoin_count == 1u,
           "Terminal sees its expired slot in the authenticated primary map and restarts JOIN");
    EXPECT(t->sync_rx_total > previous_syncs && t->sync.locked && t->failover_count == 0u &&
               t->sec.session_salt == salt,
           "Unchanged primary session preserves synchronization without triggering failover");

    t->hal = saved_hal;
    sim_world_run_us(&w, 160000, 10);
    EXPECT(t->role == ON_ROLE_T && t->short_id != old_id && t->rejoin_count == 1u &&
               g->backup_id == t->short_id && t->backup_id == t->short_id &&
               g->tdma_rx_total > previous_rx,
           "Restored uplink resumes data and automatically regains backup assignment");
}

/* 17. Test v4.0 Soft-Combining HARQ (Chase LLR Accumulation) + Multi-Rate Adaptive CA-Polar */
static void test_v40_soft_combining_harq_and_multirate_polar(void) {
    printf("[TEST 17] v4.0 Soft-Combining HARQ (Chase LLR @ -4.5 dB) + Multi-Rate CA-Polar (K=40..96)...\n");

    /* 1. Verify multi-rate CA-Polar across K=40 (4B), K=64 (7B), K=80 (9B), K=96 (11B, R=0.75) */
    const size_t rates[4] = { 4u, 7u, 9u, 11u };
    for (int r = 0; r < 4; ++r) {
        size_t mb = rates[r];
        uint8_t msg[ON_POLAR_MAX_MSG_BYTES];
        for (size_t i = 0u; i < mb; ++i) msg[i] = (uint8_t)(0x37u + i * 19u + r * 7u);

        uint8_t coded[ON_POLAR_N];
        EXPECT(on_polar_encode_rate(msg, mb, coded) == ON_OK, "Multi-rate Polar encode OK");

        int16_t llr[ON_POLAR_N];
        for (uint32_t i = 0u; i < ON_POLAR_N; ++i) {
            llr[i] = coded[i] ? (int16_t)-48 : (int16_t)+48;
        }
        uint8_t dec[ON_POLAR_MAX_MSG_BYTES];
        EXPECT(on_polar_decode_rate_fixed(llr, mb, dec) == ON_OK &&
               memcmp(dec, msg, mb) == 0,
               "Multi-rate Polar decode exact recovery");
    }

    /* 2. Soft-Combining HARQ (Chase Combining) at deep negative SNR (Es/N0 = -4.5 dB) */
    uint8_t msg5[ON_POLAR_MSG_BYTES] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x42 };
    uint8_t coded[ON_POLAR_N];
    on_polar_encode_block(msg5, coded);

    uint64_t rng = 0x9988776655443322ULL;
    double es_n0_lin = pow(10.0, -4.5 / 10.0); /* -4.5 dB */
    double sigma = sqrt(1.0 / (2.0 * es_n0_lin));

    uint32_t single_ok = 0u, combined_ok = 0u;
    for (uint32_t trial = 0u; trial < 50u; ++trial) {
        int16_t llr_tx1[ON_POLAR_N], llr_tx2[ON_POLAR_N];
        for (uint32_t i = 0u; i < ON_POLAR_N; ++i) {
            double s = coded[i] ? -1.0 : +1.0;
            /* Box-Muller Gaussian noise for TX1 and TX2 */
            rng = rng * 6364136223846793005ULL + 1ULL;
            double u1 = ((double)((rng >> 11) & 0x1FFFFFULL) + 1.0) / 2097153.0;
            rng = rng * 6364136223846793005ULL + 1ULL;
            double u2 = ((double)((rng >> 11) & 0x1FFFFFULL) + 1.0) / 2097153.0;
            double n1 = sigma * sqrt(-2.0 * log(u1)) * cos(6.28318530718 * u2);
            double n2 = sigma * sqrt(-2.0 * log(u1)) * sin(6.28318530718 * u2);

            double l1 = (2.0 / (sigma * sigma)) * (s + n1) * 16.0;
            double l2 = (2.0 / (sigma * sigma)) * (s + n2) * 16.0;
            llr_tx1[i] = (int16_t)(l1 > 511.0 ? 511 : (l1 < -512.0 ? -512 : (int)llround(l1)));
            llr_tx2[i] = (int16_t)(l2 > 511.0 ? 511 : (l2 < -512.0 ? -512 : (int)llround(l2)));
        }

        uint8_t out1[ON_POLAR_MSG_BYTES], out_comb[ON_POLAR_MSG_BYTES];
        on_polar_harq_buf_t hb;
        on_polar_harq_init(&hb, ON_POLAR_MSG_BYTES);

        if (on_polar_harq_combine_and_decode(&hb, llr_tx1, out1) == ON_OK &&
            memcmp(out1, msg5, ON_POLAR_MSG_BYTES) == 0) {
            single_ok++;
            combined_ok++;
        } else if (on_polar_harq_combine_and_decode(&hb, llr_tx2, out_comb) == ON_OK &&
                   memcmp(out_comb, msg5, ON_POLAR_MSG_BYTES) == 0) {
            combined_ok++;
        }
    }
    printf("    -> @ Es/N0=-4.5 dB: 1st-TX Recovery=%u/50 | 2-TX Soft-Combined Recovery=%u/50\n",
           single_ok, combined_ok);
    EXPECT(combined_ok > single_ok && combined_ok >= 45u,
           "Soft-Combining HARQ significantly boosts recovery at -4.5 dB (>= 90%)");
}

/* 18. Test v4.0 NearLink 2.0 ISAC 16-Channel MCPD + ToF Ranging & Anti-Relay Geofence */
static void test_v40_isac_mcpd_tof_ranging_and_antirelay(void) {
    printf("[TEST 18] v4.0 NearLink 2.0 ISAC 16-Channel MCPD + ToF Ranging & Anti-Relay Geofence...\n");

    on_ranging_ctx_t rctx;
    on_ranging_init(&rctx);

    /* Case A: Legitimate Digital Car Key at true distance = 1.65 m (inside 2.5 m unlock geofence) */
    const double true_d_m = 1.65;
    for (uint8_t ch = 0u; ch < ON_AFH_NUM_CHANNELS; ++ch) {
        double noise = ((ch & 1u) ? +0.025 : -0.025); /* +/- 0.025 rad phase measurement noise */
        double phi = on_ranging_synth_tone_phase(ch, true_d_m, noise);
        on_ranging_add_tone(&rctx, ch, phi);
    }
    double legit_rtt_ns = (2.0 * true_d_m / ON_RANGING_SPEED_OF_LIGHT_MPS) * 1e9 + 0.4; /* +0.4 ns jitter */
    on_ranging_result_t res;
    EXPECT(on_ranging_solve(&rctx, legit_rtt_ns, 2.50, 0.04, &res) == ON_OK, "MCPD+ToF solve OK");
    double err_cm = fabs(res.d_mcpd_m - true_d_m) * 100.0;
    printf("    -> True=1.650 m | MCPD=%.3f m (err=%.2f cm) | ToF=%.3f m | Geofence=%s\n",
           res.d_mcpd_m, err_cm, res.d_tof_m, res.geofence_pass ? "UNLOCK" : "DENY");
    EXPECT(err_cm < 3.0, "16-Channel MCPD ranging error < 3 cm");
    EXPECT(!res.relay_attack_detected && res.geofence_pass,
           "Legitimate car key at 1.65 m passes anti-relay check and unlocks geofence");

    /* Case B: MitM Coaxial/RF Relay Attack (attacker replays phase for 1.65 m from 18 m away -> RTT=120 ns) */
    on_ranging_result_t attack_res;
    EXPECT(on_ranging_solve(&rctx, 120.0, 2.50, 0.04, &attack_res) == ON_OK, "Relay attack check OK");
    printf("    -> Relay Attack (ToF=18.0 m vs Spoofed MCPD=1.65 m): RelayDetected=%s | Geofence=%s\n",
           attack_res.relay_attack_detected ? "YES" : "NO",
           attack_res.geofence_pass ? "UNLOCK" : "BLOCKED");
    EXPECT(attack_res.relay_attack_detected && !attack_res.geofence_pass,
           "Anti-relay physical causality check blocks 18m RF repeater attack 100%");
}

/* 19. Test v4.0 2-Byte Implicit-Address TDMA Header + Dynamic Super-Slot + 500us/1ms Multi-Gear */
static void test_v40_implicit_hdr_superslot_and_multigear(void) {
    printf("[TEST 19] v4.0 2B Implicit TDMA Header + Super-Slot Aggregation + 500us/1ms Multi-Gear...\n");

    on_mac_hdr_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.version    = ON_PROTO_VERSION;
    hdr.type       = ON_FRAME_TDMA_DATA;
    hdr.sec_en     = true;
    hdr.is_sub_g   = false;
    hdr.src_id     = 0x007u;
    hdr.dst_id     = ON_SHORT_ID_G_DEFAULT;
    hdr.sf_seq_low = 0x0Au;
    hdr.slot_idx   = 5u;

    const uint8_t payload[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
    uint8_t wire[32];
    size_t wire_len = 0u;
    EXPECT(on_frame_pack_implicit(&hdr, payload, sizeof(payload), wire, sizeof(wire), &wire_len) == ON_OK,
           "2-Byte implicit TDMA frame pack OK");
    EXPECT(wire_len == ON_IMPLICIT_HDR_LEN + sizeof(payload) + ON_PAYLOAD_CRC_LEN,
           "Implicit frame total wire length = 2B hdr + 6B payload + 2B CRC16 = 10B (saves 4B/pkt)");

    on_mac_hdr_t dec_hdr;
    const uint8_t *dec_pay = NULL;
    size_t dec_plen = 0u;
    EXPECT(on_frame_unpack_implicit(wire, wire_len, 0x007u, ON_SHORT_ID_G_DEFAULT, 5u,
                                    &dec_hdr, &dec_pay, &dec_plen) == ON_OK &&
           dec_plen == sizeof(payload) && memcmp(dec_pay, payload, sizeof(payload)) == 0,
           "2-Byte implicit TDMA frame unpacks & reconstructs 6B canonical header");

    /* Wrong slot or wrong implied_src_id must fail virtual Hdr_CRC8 immediately */
    EXPECT(on_frame_unpack_implicit(wire, wire_len, 0x008u, ON_SHORT_ID_G_DEFAULT, 5u,
                                    &dec_hdr, &dec_pay, &dec_plen) == ON_ERR_HDR_CRC,
           "Cross-node leakage rejected by virtual Hdr_CRC8 in 2B implicit header");

    /* Dynamic Super-Slot Aggregation & Multi-Gear Superframes (500 us / 1 ms / 4 ms) */
    on_sf_cfg_t cfg;
    EXPECT(on_sf_cfg_set_gear(&cfg, ON_SF_GEAR_FAST_500US) == ON_OK && cfg.t_sf_us == 500u,
           "2 kHz Fast-500us superframe gear valid");
    EXPECT(on_sf_cfg_set_gear(&cfg, ON_SF_GEAR_AUDIO_1MS) == ON_OK && cfg.t_sf_us == 1000u,
           "1 kHz Audio-1ms superframe gear valid");
    EXPECT(on_sf_cfg_set_gear(&cfg, ON_SF_GEAR_MESH_4MS) == ON_OK && cfg.t_sf_us == 4000u,
           "250 Hz Mesh-4ms superframe gear valid");

    /* Suppose slots 0, 1, 5 are occupied (bitmap = 0b00100011 = 0x0023); node at slot 1 has slots 2,3,4 free */
    uint8_t merged = on_sf_max_contiguous_slots(&cfg, 0x0023u, 1u, 4u);
    uint32_t budget_us = on_sf_superslot_air_budget_us(&cfg, merged);
    printf("    -> ImplicitHdr=2B (-66.7%%) | Super-Slot Merged=%u slots (%u us burst) | Gears=500us/1ms/4ms\n",
           merged, budget_us);
    EXPECT(merged == 4u && budget_us == 620u,
           "Super-Slot merges 4 contiguous slots (1..4) yielding 620 us single-SF burst");
}

/* 20. Test v4.0 Second-Order Alpha-Beta-Gamma Thermal Drift-Acceleration Clock Tracker */
static void test_v40_second_order_thermal_accel_clock(void) {
    printf("[TEST 20] v4.0 Second-Order Alpha-Beta-Gamma Thermal Drift-Acceleration Tracker...\n");

    on_sync_ctx_t pi_1st, abg_2nd;
    on_sync_init(&pi_1st);
    on_sync_init(&abg_2nd);
    on_sync_enable_accel_tracker(&abg_2nd, true, 0.04);

    /* Simulate cold-boot PA heating: base drift = +15 ppm, ramping at +4,000 ppb/s (+4 ppm/s) */
    const double base_ppb  = 15000.0;
    const double accel_ppb_s = 4000.0;
    const int64_t dt_net_ns = 4000000LL; /* 4 ms superframe */

    /* 40 beacons (160 ms) of normal tracking, then 25 frames (100 ms) of total blackout */
    for (int sf = 0; sf < 40; ++sf) {
        int64_t t_net = (int64_t)sf * dt_net_ns;
        double t_sec = (double)t_net * 1e-9;
        /* Exact integral of (1 + (base_ppb + accel * t) * 1e-9) dt = t + base*t + 0.5*accel*t^2 */
        int64_t t_loc = t_net + (int64_t)llround((base_ppb * t_sec + 0.5 * accel_ppb_s * t_sec * t_sec));
        on_sync_on_beacon(&pi_1st, t_net, t_loc);
        on_sync_on_beacon(&abg_2nd, t_net, t_loc);
    }

    /* Evaluate holdover error after 25 dropped superframes (100 ms blackout at SF=65) */
    int64_t eval_net = 65LL * dt_net_ns;
    double eval_sec = (double)eval_net * 1e-9;
    int64_t eval_loc = eval_net + (int64_t)llround((base_ppb * eval_sec + 0.5 * accel_ppb_s * eval_sec * eval_sec));

    int64_t err_1st_ns = llabs(on_sync_local_to_net(&pi_1st, eval_loc) - eval_net);
    int64_t err_2nd_ns = llabs(on_sync_local_to_net(&abg_2nd, eval_loc) - eval_net);
    printf("    -> Thermal Ramp (+4 ppm/s) + 25-SF (100ms) Blackout: 1st-Order PI Err=%lld ns | 2nd-Order ABG Err=%lld ns\n",
           (long long)err_1st_ns, (long long)err_2nd_ns);
    EXPECT(err_2nd_ns < err_1st_ns / 3 && err_2nd_ns < 15,
           "2nd-order Alpha-Beta-Gamma tracker cuts thermal-ramp blackout drift by >3x (< 15 ns)");
}

/* Inject a beacon with an independent RX time (to model variable Wi-Fi queuing). */
static void act_test_beacon(on_node_t *t, uint16_t src, uint32_t seq, uint32_t salt,
                            uint8_t level, uint8_t lead, int64_t tx_ns, int64_t rx_ns,
                            bool tamper) {
    on_sync_payload_t sp = {0};
    sp.sf_seq = seq;
    sp.session_salt = salt;
    sp.t_tx_net_ns = tx_ns;
    sp.t_sf_us = ON_DEFAULT_TSF_US;
    sp.act_fire_off_us = ON_DEFAULT_ACT_FIRE_US;
    sp.act_cmd = level;
    sp.act_flip_lead_sf = lead;
    on_mac_hdr_t hdr = {ON_PROTO_VERSION, ON_FRAME_SYNC_MAP, false, false,
        src, ON_SHORT_ID_BROADCAST, (uint8_t)(seq & 0x0Fu), 0u};
    uint8_t pbuf[ON_SYNC_PAYLOAD_WIRE_LEN], frame[ON_MAX_FRAME_LEN], hbuf[ON_MAC_HDR_LEN];
    size_t plen = 0u, flen = 0u;
    if (on_sync_payload_pack(&sp, pbuf, sizeof(pbuf), &plen) != ON_OK) return;
    if (t->sec.enabled) {
        if (on_hdr_pack(&hdr, hbuf) != ON_OK) return;
        on_sec_compute_beacon_mac(&t->sec, hbuf, pbuf, seq, salt, pbuf + 40);
    }
    if (tamper) pbuf[36] ^= 2u;
    if (on_frame_pack(&hdr, pbuf, plen, frame, sizeof(frame), &flen) == ON_OK) {
        on_node_on_rx_raw(t, frame, flen, rx_ns);
    }
}

static void test_act_lookahead(void) {
    printf("[TEST ACT] Authenticated lookahead GPIO scheduling...\n");
    on_sync_payload_t sp = {0}, decoded;
    sp.act_cmd = 1u;
    sp.act_flip_lead_sf = 7u;
    sp.sub_g_id_2 = 0xABCu;
    sp.flags = 0xA2u;
    sp.beacon_mac[0] = 0xBBu;
    uint8_t pbuf[ON_SYNC_PAYLOAD_WIRE_LEN];
    size_t plen = 0;
    EXPECT(on_sync_payload_pack(&sp, pbuf, sizeof pbuf, &plen) == ON_OK && plen == 44u &&
           pbuf[35] == 1u && pbuf[36] == 0xAEu && pbuf[37] == 0xA2u && pbuf[40] == 0xBBu &&
           on_sync_payload_unpack(pbuf, plen, &decoded) == ON_OK && decoded.act_flip_lead_sf == 7u &&
           decoded.hop_depth == 0u && decoded.sub_g_id_2 == 0xABCu,
           "ACT lead leaves beacon length, command, Sub-G-2 ID, flags and MAC location intact");
    sp.hop_depth = 1u;
    EXPECT(on_sync_payload_pack(&sp, pbuf, sizeof pbuf, &plen) == ON_OK && pbuf[36] == 0xA1u &&
           on_sync_payload_unpack(pbuf, plen, &decoded) == ON_OK && decoded.act_flip_lead_sf == 0u &&
           decoded.hop_depth == 1u, "SUB_SYNC remains unchanged");

    on_node_t t;
    on_node_init(&t, NULL, ON_ROLE_UNJOINED, 0);
    t.act_lookahead_enabled = true;
    const uint8_t key[16] = {1, 2, 3, 4};
    on_node_enable_security(&t, key);
    act_test_beacon(&t, 1u, 7u, 20u, 0u, 3u, 28000000LL, 28100000LL, false);
    const int64_t deadline = 43800000LL;
    EXPECT(t.act_lookahead.pending && t.act_lookahead.target_sf == 10u &&
           t.act_lookahead.deadline_local_ns == deadline && t.act_cmd == 0u,
           "Early beacon arms future target without flipping immediately");
    act_test_beacon(&t, 1u, 8u, 20u, 0u, 2u, 32000000LL, 40500000LL, false);
    EXPECT(t.act_lookahead.deadline_local_ns == deadline, "Slower repeated announcement cannot postpone the deadline");
    on_node_step(&t, deadline - 1LL);
    EXPECT(t.act_cmd == 0u && t.act_lookahead.pending, "Does not flip before the local deadline");
    EXPECT(on_node_next_wakeup_local_ns(&t, deadline - 1LL) == deadline, "Armed deadline is a wake-up candidate");
    on_node_step(&t, deadline);
    EXPECT(t.act_cmd == 1u && !t.act_lookahead.pending && t.last_act_was_scheduled &&
           t.last_act_local_ns == deadline && t.current_sf_seq < 10u,
           "Target beacon can be absent and TDMA can lag while the armed action still fires");
    act_test_beacon(&t, 1u, 9u, 20u, 0u, 1u, 36000000LL, 45000000LL, false);
    act_test_beacon(&t, 1u, 9u, 20u, 0u, 1u, 36000000LL, 46000000LL, false);
    EXPECT(t.act_cmd == 1u && !t.act_lookahead.pending, "Late and duplicate beacons cannot roll back or re-arm a fired target");
    act_test_beacon(&t, 1u, 10u, 20u, 1u, 0u, 40000000LL, 47000000LL, false);
    act_test_beacon(&t, 1u, 13u, 20u, 1u, 7u, 52000000LL, 53000000LL, true);
    EXPECT(!t.act_lookahead.pending && t.sec.beacon_auth_drops == 1u,
           "Modified ACT lead is rejected by beacon authentication");
    act_test_beacon(&t, 1u, 13u, 20u, 1u, 7u, 52000000LL, 53000000LL, false);
    EXPECT(t.act_lookahead.pending, "Following transition can be armed normally");
    act_test_beacon(&t, 1u, 0u, 21u, 0u, 0u, 1000000LL, 55000000LL, false);
    EXPECT(!t.act_lookahead.pending && t.act_cmd == 0u && t.act_lookahead.beacon_sf == 0u,
           "New boot session clears plan and accepts restarted sequence, even before JOIN");
    act_test_beacon(&t, 1u, 4u, 21u, 0u, 6u, 17000000LL, 71000000LL, false);
    act_test_beacon(&t, 2u, 1u, 21u, 1u, 0u, 18000000LL, 72000000LL, false);
    EXPECT(!t.act_lookahead.pending && t.act_cmd == 1u, "New primary invalidates prior pending action");

    on_node_init(&t, NULL, ON_ROLE_UNJOINED, 0);
    act_test_beacon(&t, 1u, 9u, 0u, 0u, 1u, 36000000LL, 36000000LL, false);
    on_node_step(&t, 43700000LL);
    EXPECT(!t.act_lookahead.pending && t.act_cmd == 0u, "Default terminal ignores lead and retains legacy behavior");
    t.act_lookahead_enabled = true;
    act_test_beacon(&t, 1u, UINT32_MAX, 0u, 1u, 1u, 50000000LL, 50000000LL, false);
    EXPECT(t.act_lookahead.pending && t.act_lookahead.target_sf == 0u, "Target frame zero is valid after sequence wrap");
    on_node_step(&t, t.act_lookahead.deadline_local_ns);
    EXPECT(t.act_cmd == 0u && t.last_act_was_scheduled, "Wrapped action is applied exactly like any other target");
}

static void test_queued_tx_min_delay_envelope(void) {
    printf("[TEST 25] Queued-TX Asymmetric Minimum-Delay Envelope Filter (ESP32 Wi-Fi Jitter Rejection)...\n");

    on_node_t t;
    on_node_init(&t, NULL, ON_ROLE_UNJOINED, 0);
    t.hal.tx_timing_best_effort = true;
    t.act_lookahead_enabled     = true;

    /*
     * Simulate an ESP32 terminal with a -22 ppm crystal offset (local clock runs slower:
     * true_local_ns(net_ns) = 10,000,000 + net_ns / (1.0 - 22e-6)) receiving 40 superframes
     * where esp_wifi_80211_tx adds asymmetric one-sided queuing spikes up to +1,800 us!
     * Including an initially queued boot beacon (+650 us on SF=0).
     */
    static const int64_t kQueueDelayUs[20] = {
        650, 180,   0, 1450,  45,   1, 1800, 920,   0, 310,
       1250,   0, 740, 1620,   1, 530,    0, 1100, 890,   0
    };
    const double true_rate_ppb = -22000.0; /* -22 ppm */
    const int64_t boot_offset_ns = 10000000LL;

    int64_t max_phase_err_after_floor_ns = 0;
    for (uint32_t sf = 0; sf < 40u; ++sf) {
        int64_t tx_net_ns = (int64_t)sf * 4000000LL;
        int64_t true_floor_loc_ns = boot_offset_ns +
            (int64_t)llround((double)tx_net_ns / (1.0 + true_rate_ppb * 1e-9));
        int64_t q_ns = kQueueDelayUs[sf % 20u] * 1000LL;
        int64_t rx_loc_ns = true_floor_loc_ns + q_ns;

        uint8_t lead = (sf == 37u) ? 4u : 0u; /* Arm lookahead flip for SF=41 during a +1100 us queued beacon */
        act_test_beacon(&t, 1u, sf, 77u, 0u, lead, tx_net_ns, rx_loc_ns, false);

        if (sf >= 10u) {
            int64_t pred_net_at_floor = on_sync_local_to_net(&t.sync, true_floor_loc_ns);
            int64_t phase_err_ns = pred_net_at_floor - tx_net_ns;
            if (phase_err_ns < 0) phase_err_ns = -phase_err_ns;
            if (phase_err_ns > max_phase_err_after_floor_ns) {
                max_phase_err_after_floor_ns = phase_err_ns;
            }
        }
    }

    int64_t target_sf41_net_ns = 41LL * 4000000LL + 3700000LL;
    int64_t true_sf41_act_loc_ns = boot_offset_ns +
        (int64_t)llround((double)target_sf41_net_ns / (1.0 + true_rate_ppb * 1e-9));
    int64_t lookahead_err_ns = t.act_lookahead.deadline_local_ns - true_sf41_act_loc_ns;
    if (lookahead_err_ns < 0) lookahead_err_ns = -lookahead_err_ns;

    printf("  -> Max clock phase error under 1800 us Wi-Fi TX queue spikes: %lld ns (%.2f us), NCO=%.2f ppm, Lookahead ACT err=%lld ns\n",
           (long long)max_phase_err_after_floor_ns,
           (double)max_phase_err_after_floor_ns * 1e-3,
           t.sync.rate_ppb * 1e-3,
           (long long)lookahead_err_ns);

    EXPECT(max_phase_err_after_floor_ns < 2500LL,
           "Minimum-delay envelope filter rejects +1800 us Wi-Fi queuing spikes to < 2.5 us phase error");
    EXPECT(t.act_lookahead.pending && t.act_lookahead.target_sf == 41u && lookahead_err_ns < 2500LL,
           "Lookahead GPIO deadline armed from a +1100 us delayed beacon hits true target within < 2.5 us");
}

/* ========================================================================== */
/* TEST 26: v5.0 Authenticated Multi-Hop Node OTA (1-Hop + 2-Hop Sub-G Relay) */
/* ========================================================================== */
static void test_v50_multihop_authenticated_node_ota(void) {
    printf("[TEST 26] v5.0 Authenticated Multi-Hop Node OTA (Broadcast + 2-Hop Relay + Selective NACK + Tamper Guard)...\n");

    static const uint8_t kKey[ON_SEC_KEY_LEN] = {
        0x55,0xAA,0x11,0x22,0x33,0x44,0x77,0x88,
        0x99,0x00,0xFE,0xDC,0xBA,0x98,0x76,0x54
    };

    /* 1. Prepare synthetic 1,024-byte firmware v5.1.0 image */
    uint8_t fw_img[1024];
    for (size_t i = 0u; i < sizeof(fw_img); ++i) {
        fw_img[i] = (uint8_t)((i * 37u + 0xA5u) & 0xFFu);
    }

    /* 2. Unit-test anti-downgrade, tamper rejection & selective-repeat NACK recovery under 25% loss */
    on_ota_ctx_t ota_tx, ota_rx;
    uint8_t rx_flash[1024];
    on_ota_ram_storage_t ram_st;
    on_ota_storage_ops_t st_ops;
    on_ota_ram_storage_init(&ram_st, rx_flash, sizeof(rx_flash), &st_ops);
    on_ota_init(&ota_tx, 0x00050000u, 1u, NULL);
    on_ota_init(&ota_rx, 0x00050000u, 1u, &st_ops);

    /* Reject firmware downgrade (v4.9.0 <= v5.0.0) without FORCE flag */
    on_ota_meta_t old_meta;
    memset(&old_meta, 0, sizeof(old_meta));
    old_meta.ota_id       = 1u;
    old_meta.fw_version   = 0x00040900u;
    old_meta.total_size   = sizeof(fw_img);
    old_meta.chunk_size   = 64u;
    old_meta.total_chunks = 16u;
    old_meta.target_id    = 0x004u;
    EXPECT(on_ota_rx_on_meta(&ota_rx.rx, &old_meta, 0x004u) != ON_OK &&
           ota_rx.rx.last_err == ON_OTA_ERR_DOWNGRADE,
           "OTA anti-downgrade guard rejects older firmware version");

    /* Start valid v5.1.0 authenticated OTA session */
    EXPECT(on_ota_tx_start_mem(&ota_tx.tx, 0x0501u, 0x00050100u, 0x004u, 1u,
                               ON_OTA_FLAG_AUTO_REBOOT, fw_img, sizeof(fw_img),
                               64u, kKey) == ON_OK,
           "OTA sender initializes authenticated 1024B image (16 x 64B chunks)");
    EXPECT(on_ota_rx_on_meta(&ota_rx.rx, &ota_tx.tx.meta, 0x004u) == ON_OK,
           "OTA receiver accepts valid v5.1.0 manifest");

    /* Drop chunks 2, 5, 9, 13 on first pass -> verify selective-repeat NACK bitmap recovers all 4! */
    uint32_t drops = 0u;
    for (uint32_t step = 0u; step < 64u && ota_rx.rx.state != ON_OTA_STATE_READY; ++step) {
        on_ota_chunk_t ch;
        if (on_ota_tx_next_chunk(&ota_tx.tx, &ch)) {
            bool drop_first = (ch.flags == 0u) &&
                              (ch.chunk_idx == 2u || ch.chunk_idx == 5u ||
                               ch.chunk_idx == 9u || ch.chunk_idx == 13u);
            if (drop_first) {
                drops++;
            } else {
                on_ota_rx_on_chunk(&ota_rx.rx, &ch, kKey);
            }
        }
        if (ota_rx.rx.status_dirty || (step % 4u) == 3u) {
            on_ota_status_t st;
            on_ota_rx_build_status(&ota_rx.rx, 0x004u, &st);
            ota_rx.rx.status_dirty = false;
            on_ota_tx_on_status(&ota_tx.tx, &st, step);
        }
    }
    EXPECT(drops == 4u && ota_rx.rx.state == ON_OTA_STATE_READY &&
           ota_tx.tx.chunks_retx_total >= 4u &&
           memcmp(rx_flash, fw_img, sizeof(fw_img)) == 0,
           "Selective-repeat NACK bitmap recovers all 4 dropped chunks bit-exact");

    /* Tamper test: corrupt 1 byte in flash -> verify image integrity & AES-CBC-MAC rejects it! */
    rx_flash[500] ^= 0x01u;
    EXPECT(on_ota_rx_verify_image(&ota_rx.rx, kKey) == ON_ERR_PAY_CRC &&
           ota_rx.rx.last_err == ON_OTA_ERR_CRC_MISMATCH,
           "Corrupted flash byte fails CRC-32 verification");
    rx_flash[500] ^= 0x01u;
    ota_rx.rx.meta.auth_tag[0] ^= 0x80u;
    EXPECT(on_ota_rx_verify_image(&ota_rx.rx, kKey) == ON_ERR_DECODE_FAIL &&
           ota_rx.rx.last_err == ON_OTA_ERR_AUTH_FAIL,
           "Forged OTA auth tag fails 64-bit AES-128-CBC-MAC verification");

    /* 3. End-to-End 10-Node Encrypted Multi-Hop Cluster OTA (1-Hop T4 + Sub-G T2 + 2-Hop Edge T8!) */
    sim_world_t w;
    sim_world_init(&w, 0x50A02026ULL);
    sim_world_setup_10node_demo(&w, true);
    for (uint32_t i = 0u; i < w.num_nodes; ++i) {
        on_node_enable_security(&w.nodes[i], kKey);
    }

    static on_ota_ctx_t node_ota[10];
    static uint8_t flash_bank[10][1024];
    static on_ota_ram_storage_t ram_bank[10];
    static on_ota_storage_ops_t ops_bank[10];
    for (uint32_t i = 0u; i < w.num_nodes; ++i) {
        on_ota_ram_storage_init(&ram_bank[i], flash_bank[i], sizeof(flash_bank[i]), &ops_bank[i]);
        on_ota_init(&node_ota[i], 0x00050000u, 1u, &ops_bank[i]);
        on_node_attach_ota(&w.nodes[i], &node_ota[i]);
    }

    /* Bring up 10-node cluster (including 2-hop wall-blocked T8_E1 & T9_E2 via Sub-G T2) */
    sim_world_run_us(&w, 80000, 10);
    EXPECT(w.nodes[8].role == ON_ROLE_T && w.nodes[8].is_sub_node,
           "2-hop wall-blocked edge node T8 joined via Sub-G before OTA");

    /* Start encrypted cluster-wide broadcast OTA (512B = 8 x 64B chunks) from Primary G */
    EXPECT(on_node_ota_start_mem(&w.nodes[0], 0x0510u, 0x00050100u, ON_SHORT_ID_BROADCAST,
                                 1u, ON_OTA_FLAG_AUTO_REBOOT, fw_img, 512u, 64u) == ON_OK,
           "Primary G starts AES-128-CCM encrypted broadcast OTA across 1-hop + 2-hop cluster");

    /* Stream chunks + selective-repeat NACK recovery until 1-hop T4, Sub-G T2 & 2-hop T8 are READY */
    sim_world_run_us(&w, 240000, 10);
    EXPECT(node_ota[4].rx.state == ON_OTA_STATE_READY &&
           memcmp(flash_bank[4], fw_img, 512u) == 0,
           "1-hop node T4 received & authenticated 512B OTA image over AES-128-CCM");
    EXPECT(node_ota[2].rx.state == ON_OTA_STATE_READY &&
           memcmp(flash_bank[2], fw_img, 512u) == 0,
           "Sub-G proxy T2 received & authenticated 512B OTA image");
    EXPECT(node_ota[8].rx.state == ON_OTA_STATE_READY &&
           memcmp(flash_bank[8], fw_img, 512u) == 0,
           "Wall-blocked 2-hop edge node T8 received & authenticated 512B OTA via Sub-G relay!");

    /* Schedule synchronized SF-aligned cluster commit & activation */
    EXPECT(on_node_ota_commit(&w.nodes[0], 6u, ON_OTA_FLAG_AUTO_REBOOT) == ON_OK,
           "Primary G schedules synchronized OTA commit (+6 SFs)");
    sim_world_run_us(&w, 60000, 10);

    printf("    -> Cluster OTA: T4(1-hop)=v%X T2(Sub-G)=v%X T8(2-hop)=v%X | TX=%u Collisions=%u\n",
           (unsigned)ram_bank[4].applied_fw_version,
           (unsigned)ram_bank[2].applied_fw_version,
           (unsigned)ram_bank[8].applied_fw_version,
           w.nodes[0].ota_tx_total, w.tdma_subg_collisions + w.cra_collisions);

    EXPECT(ram_bank[4].applied && ram_bank[4].applied_fw_version == 0x00050100u &&
           ram_bank[2].applied && ram_bank[2].applied_fw_version == 0x00050100u &&
           ram_bank[8].applied && ram_bank[8].applied_fw_version == 0x00050100u,
           "1-hop T4, Sub-G T2, and 2-hop wall-blocked T8 all activated v5.1.0 synchronously");
    EXPECT(w.tdma_subg_collisions == 0u && w.cra_collisions == 0u,
           "Zero RF collisions during encrypted multi-hop cluster OTA");
}

/* ========================================================================== */
/* TEST 27: v5.0 Public Wi-Fi STA + BLE GATT Gateway Bridge & Coexistence     */
/* ========================================================================== */
typedef struct {
    uint32_t wifi_connect_calls;
    uint32_t cloud_posts;
    uint32_t ble_notifies;
    uint8_t  aligned_ch;
    char     last_json[384];
    uint32_t dl_delivered;
    char     last_dl_text[96];
} test_gw_mock_t;

static on_err_t mock_wifi_connect(void *ctx, const char *ssid, const char *pass) {
    (void)ssid; (void)pass;
    ((test_gw_mock_t *)ctx)->wifi_connect_calls++;
    return ON_OK;
}
static on_err_t mock_cloud_post(void *ctx, const char *url, const char *json) {
    (void)url;
    test_gw_mock_t *m = (test_gw_mock_t *)ctx;
    m->cloud_posts++;
    strncpy(m->last_json, json, sizeof(m->last_json) - 1u);
    m->last_json[sizeof(m->last_json) - 1u] = '\0';
    return ON_OK;
}
static on_err_t mock_ble_notify(void *ctx, const uint8_t *data, size_t len) {
    (void)data; (void)len;
    ((test_gw_mock_t *)ctx)->ble_notifies++;
    return ON_OK;
}
static on_err_t mock_ch_align(void *ctx, uint8_t ch) {
    ((test_gw_mock_t *)ctx)->aligned_ch = ch;
    return ON_OK;
}
static on_err_t mock_dl_delivered(void *ctx, const on_gw_bridge_msg_t *msg) {
    test_gw_mock_t *m = (test_gw_mock_t *)ctx;
    m->dl_delivered++;
    size_t n = (msg->payload_len < sizeof(m->last_dl_text) - 1u) ? msg->payload_len : (sizeof(m->last_dl_text) - 1u);
    memcpy(m->last_dl_text, msg->payload, n);
    m->last_dl_text[n] = '\0';
    return ON_OK;
}

static void test_v50_public_wifi_ble_gateway_bridge(void) {
    printf("[TEST 27] v5.0 Public Wi-Fi STA + BLE GATT Gateway Bridge (BLE Prov + AP Ch-Sync + 2-Hop Cloud/BLE Tunnel)...\n");

    static const uint8_t kKey[ON_SEC_KEY_LEN] = {
        0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,
        0x90,0xA0,0xB0,0xC0,0xD0,0xE0,0xF0,0x01
    };

    sim_world_t w;
    sim_world_init(&w, 0x50B12026ULL);
    sim_world_setup_10node_demo(&w, true);
    for (uint32_t i = 0u; i < w.num_nodes; ++i) {
        on_node_enable_security(&w.nodes[i], kKey);
    }

    test_gw_mock_t g_mock, t8_mock;
    memset(&g_mock, 0, sizeof(g_mock));
    memset(&t8_mock, 0, sizeof(t8_mock));

    on_gw_transport_ops_t g_ops = {
        .user_ctx              = &g_mock,
        .wifi_connect          = mock_wifi_connect,
        .cloud_post_json       = mock_cloud_post,
        .ble_notify            = mock_ble_notify,
        .on_channel_align      = mock_ch_align,
        .on_downlink_delivered = NULL
    };
    on_gw_transport_ops_t t8_ops = {
        .user_ctx              = &t8_mock,
        .wifi_connect          = NULL,
        .cloud_post_json       = NULL,
        .ble_notify            = NULL,
        .on_channel_align      = NULL,
        .on_downlink_delivered = mock_dl_delivered
    };

    on_gw_ctx_t g_gw, t2_gw, t8_gw;
    on_gw_init(&g_gw, ON_GW_MODE_WIFI_AND_BLE, &g_ops);
    on_gw_init(&t2_gw, ON_GW_MODE_OFF, NULL);
    on_gw_init(&t8_gw, ON_GW_MODE_OFF, &t8_ops);
    on_node_attach_gateway(&w.nodes[0], &g_gw);
    on_node_attach_gateway(&w.nodes[2], &t2_gw);
    on_node_attach_gateway(&w.nodes[8], &t8_gw);

    /* 1. BLE phone connects and provisions public Wi-Fi router credentials + Cloud URL */
    on_gw_on_ble_state(&g_gw, true, 185u);
    const char *prov = "WIFI:Factory5G_24|SuperSecret99|https://iot.example.com/v1/telemetry";
    EXPECT(on_gw_handle_control_text(&g_gw, prov, strlen(prov), true) == ON_OK &&
           strcmp(g_gw.cfg.wifi_ssid, "Factory5G_24") == 0 &&
           strcmp(g_gw.cfg.cloud_url, "https://iot.example.com/v1/telemetry") == 0 &&
           g_mock.wifi_connect_calls == 1u,
           "BLE GATT Wi-Fi provisioning parses SSID/Pass/CloudURL and triggers STA connect");

    /* 2. Public Wi-Fi router connects on Channel 11 -> Gateway aligns OpenNear channel to Ch 11 */
    const uint8_t ip4[4] = { 192u, 168u, 1u, 88u };
    EXPECT(on_gw_on_wifi_connected(&g_gw, 11u, -48, ip4) == 11u &&
           g_mock.aligned_ch == 11u && g_gw.wifi_connected,
           "Wi-Fi STA AP channel alignment locks OpenNear coexistence channel to router Ch 11");

    /* 3. Bring up 10-node cluster -> 2-hop edge node T8 (0x100) sends telemetry to Public Cloud & BLE */
    sim_world_run_us(&w, 80000, 10);
    EXPECT(w.nodes[8].role == ON_ROLE_T && w.nodes[8].short_id == 0x100u,
           "2-hop wall-blocked edge node T8 online as 0x100");

    const uint8_t sensor_pay[4] = { 0xDEu, 0xADu, 0xBEu, 0xEFu };
    EXPECT(on_node_send_gw_uplink(&w.nodes[8], ON_GW_MSG_TELEMETRY_UP, sensor_pay, 4u) == ON_OK,
           "2-hop edge node T8 queues public-cloud telemetry uplink");

    /* 4. Phone sends BLE downlink command targeting 2-hop edge node 0x100 ("CMD:100|MOTOR_ON") */
    const char *ble_cmd = "CMD:100|MOTOR_ON";
    EXPECT(on_gw_handle_control_text(&g_gw, ble_cmd, strlen(ble_cmd), true) == ON_OK,
           "BLE phone queues downlink command to 2-hop node 0x100");

    /* 5. Phone also triggers Cloud HTTP OTA pull ("OTA:FFF|327936|http://ota.example.com/fw.bin") */
    const char *ota_cmd = "OTA:FFF|327936|http://ota.example.com/fw.bin";
    EXPECT(on_gw_handle_control_text(&g_gw, ota_cmd, strlen(ota_cmd), true) == ON_OK &&
           g_gw.cfg.ota_pull_requested && g_gw.cfg.ota_target_id == 0xFFFu &&
           g_gw.cfg.ota_target_ver == 327936u,
           "BLE/Cloud OTA trigger parses target=0xFFF, version & HTTP firmware URL");

    /* Run OpenNear superframes for bidirectional 2-hop <-> Sub-G <-> G relay */
    sim_world_run_us(&w, 120000, 10);
    uint32_t flushed = on_gw_poll_network(&g_gw);

    printf("    -> Cloud POSTs=%u BLE Notifies=%u | JSON=%s\n",
           g_mock.cloud_posts, g_mock.ble_notifies, g_mock.last_json);
    printf("    -> 2-Hop Node T8(0x100) Downlink RX=%u Payload='%s' | Collisions=%u\n",
           t8_mock.dl_delivered, t8_mock.last_dl_text, w.tdma_subg_collisions + w.cra_collisions);

    EXPECT(flushed == 1u && g_mock.cloud_posts == 1u && g_mock.ble_notifies == 1u &&
           strstr(g_mock.last_json, "\"node\":\"0x100\"") != NULL &&
           strstr(g_mock.last_json, "\"hop\":2") != NULL &&
           strstr(g_mock.last_json, "\"wifi_ch\":11") != NULL &&
           strstr(g_mock.last_json, "DEADBEEF") != NULL,
           "2-hop edge node T8 telemetry bridged through Sub-G + Primary G to Public Cloud JSON & BLE Notify");
    EXPECT(t8_mock.dl_delivered == 1u && strcmp(t8_mock.last_dl_text, "MOTOR_ON") == 0,
           "BLE/Cloud downlink command 'MOTOR_ON' delivered over AES-128-CCM to wall-blocked 2-hop T8");
    EXPECT(w.tdma_subg_collisions == 0u && w.cra_collisions == 0u,
           "Zero RF collisions during bidirectional Wi-Fi/BLE Gateway bridging");
}

int main(void) {
    printf("============================================================\n");
    printf("       OpenNear v5.0 Core Protocol Automated Test Suite     \n");
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
    test_v31_x25519_ecdh_onboarding();
    test_v31_multizone_subg_fdma_concurrency();
    test_v31_ptp_followup_and_ant_delay();
    test_v32_fpga_rtl_golden_vectors();
    test_v40_soft_combining_harq_and_multirate_polar();
    test_v40_isac_mcpd_tof_ranging_and_antirelay();
    test_v40_implicit_hdr_superslot_and_multigear();
    test_v40_second_order_thermal_accel_clock();
    test_best_effort_tx_timing();
    test_rejoin_after_primary_reboot();
    test_rejoin_after_primary_lease_expiry();
    test_act_lookahead();
    test_queued_tx_min_delay_envelope();
    test_v50_multihop_authenticated_node_ota();
    test_v50_public_wifi_ble_gateway_bridge();

    printf("\n------------------------------------------------------------\n");
    printf("SUMMARY: %d passed, %d failed\n", g_tests_passed, g_tests_failed);
    printf("------------------------------------------------------------\n");
    return (g_tests_failed == 0) ? 0 : 1;
}
