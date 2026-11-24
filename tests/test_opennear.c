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
#include "on_mesh.h"
#include "on_mqtt.h"
#include "on_locate.h"
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
static void act_test_beacon_timed(on_node_t *t, uint16_t src, uint32_t seq, uint32_t salt,
                                  uint8_t level, uint8_t lead, int64_t tx_ns, int64_t rx_ns,
                                  int64_t act_rx_ns, bool tamper) {
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
        on_node_on_rx_raw_timed(t, frame, flen, rx_ns, act_rx_ns);
    }
}

static void act_test_beacon(on_node_t *t, uint16_t src, uint32_t seq, uint32_t salt,
                            uint8_t level, uint8_t lead, int64_t tx_ns, int64_t rx_ns,
                            bool tamper) {
    act_test_beacon_timed(t, src, seq, salt, level, lead, tx_ns, rx_ns, rx_ns, tamper);
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

/* ========================================================================== */
/* TEST 28: v5.1 Any-Node Full Cluster Topology & Live Sensor Data Query      */
/* ========================================================================== */
static void test_v51_any_node_cluster_topology_and_data_query(void) {
    printf("[TEST 28] v5.1 Any-Node Full Cluster Topology & Live Sensor Data Introspection (G + 1-Hop T5 + 2-Hop T8)...\n");

    static const uint8_t kKey[ON_SEC_KEY_LEN] = {
        0xAA,0xBB,0xCC,0xDD,0x11,0x22,0x33,0x44,
        0x55,0x66,0x77,0x88,0x99,0x00,0x12,0x34
    };

    sim_world_t w;
    sim_world_init(&w, 0x51C12026ULL);
    sim_world_setup_10node_demo(&w, true);
    for (uint32_t i = 0u; i < w.num_nodes; ++i) {
        on_node_enable_security(&w.nodes[i], kKey);
    }

    /* Set custom application sensor payloads on 1-hop T4 and 2-hop wall-blocked T8 */
    const uint8_t t4_sensor[4] = { 0x11u, 0x22u, 0x33u, 0x44u };
    const uint8_t t8_sensor[4] = { 0xCAu, 0xFEu, 0xBAu, 0xBEu };
    on_node_set_tx_payload(&w.nodes[4], t4_sensor, 4u);
    on_node_set_tx_payload(&w.nodes[8], t8_sensor, 4u);

    /* Run 10-node cluster so 1-hop + 2-hop nodes join and stream sensor data to Primary G */
    sim_world_run_us(&w, 120000, 10);

    /* 1. Direct query on Primary G -> must see all 10 nodes (0-hop + 1-hop + 2-hop) + live payloads */
    on_cluster_snapshot_t g_snap;
    size_t g_cnt = on_node_get_cluster_snapshot(&w.nodes[0], &g_snap);
    char json_buf[2048];
    size_t jlen = on_node_format_cluster_json(&w.nodes[0], true, json_buf, sizeof(json_buf));

    printf("    -> Primary G snapshot: count=%u | JSON len=%u | T8_2hop=%s\n",
           (unsigned)g_cnt, (unsigned)jlen,
           strstr(json_buf, "CAFEBABE") ? "FOUND(CAFEBABE)" : "MISSING");

    EXPECT(g_cnt == 10u && jlen > 100u &&
           strstr(json_buf, "\"id\":\"0x100\"") != NULL &&
           strstr(json_buf, "\"hop\":2") != NULL &&
           strstr(json_buf, "11223344") != NULL &&
           strstr(json_buf, "CAFEBABE") != NULL,
           "Primary G tracks all 10 nodes (incl. 2-hop T8/T9) and exports live sensor payloads as JSON");

    /* 2. Query from an ordinary 1-hop Terminal node (T5) over encrypted OpenNear air */
    EXPECT(on_node_request_cluster_snapshot(&w.nodes[5]) == ON_OK,
           "1-hop terminal T5 requests full cluster snapshot from Primary G");
    sim_world_run_us(&w, 140000, 10);

    on_cluster_snapshot_t t5_snap;
    size_t t5_cnt = on_node_get_cluster_snapshot(&w.nodes[5], &t5_snap);
    char t5_json[2048];
    on_node_format_cluster_json(&w.nodes[5], true, t5_json, sizeof(t5_json));
    printf("    -> 1-Hop Terminal T5 remote snapshot: valid=%d count=%u cached=%d\n",
           w.nodes[5].cluster_snapshot_valid ? 1 : 0, (unsigned)t5_cnt,
           t5_snap.is_remote_cached ? 1 : 0);
    EXPECT(w.nodes[5].cluster_snapshot_valid && t5_cnt == 10u && t5_snap.is_remote_cached &&
           strstr(t5_json, "CAFEBABE") != NULL && strstr(t5_json, "11223344") != NULL,
           "1-hop terminal T5 fetches entire 10-node topology & live sensor data from G over air");

    /* 3. Query from a wall-blocked 2-hop Edge node (T8, 0x100) through Sub-G proxy! */
    EXPECT(on_node_request_cluster_snapshot(&w.nodes[8]) == ON_OK,
           "Wall-blocked 2-hop edge node T8 requests full cluster snapshot via Sub-G");
    sim_world_run_us(&w, 180000, 10);

    on_cluster_snapshot_t t8_snap;
    size_t t8_cnt = on_node_get_cluster_snapshot(&w.nodes[8], &t8_snap);
    char t8_json[2048];
    on_node_format_cluster_json(&w.nodes[8], true, t8_json, sizeof(t8_json));
    printf("    -> 2-Hop Edge T8 remote snapshot: valid=%d count=%u | T4_data=%s | Collisions=%u\n",
           w.nodes[8].cluster_snapshot_valid ? 1 : 0, (unsigned)t8_cnt,
           strstr(t8_json, "11223344") ? "FOUND" : "MISSING",
           w.tdma_subg_collisions + w.cra_collisions);
    EXPECT(w.nodes[8].cluster_snapshot_valid && t8_cnt == 10u && t8_snap.is_remote_cached &&
           strstr(t8_json, "11223344") != NULL && strstr(t8_json, "\"id\":\"0x001\"") != NULL,
           "Wall-blocked 2-hop edge node T8 fetches entire 10-node cluster topology & data through Sub-G");
    EXPECT(w.tdma_subg_collisions == 0u && w.cra_collisions == 0u,
           "Zero RF collisions during multi-hop paginated cluster topology queries");

    /* 4. Verify BLE/HTTP control text TOPO? and DATA? commands */
    on_gw_ctx_t gw;
    on_gw_init(&gw, ON_GW_MODE_WIFI_AND_BLE, NULL);
    EXPECT(on_gw_handle_control_text(&gw, "TOPO?", 5u, true) == ON_OK &&
           gw.cfg.topo_query_requested && !gw.cfg.topo_include_data,
           "BLE/HTTP TOPO? control query parsed cleanly");
    EXPECT(on_gw_handle_control_text(&gw, "DATA?", 5u, true) == ON_OK &&
           gw.cfg.topo_query_requested && gw.cfg.topo_include_data,
           "BLE/HTTP DATA? control query parsed cleanly");
}

/* ========================================================================== */
/* TEST 29: v6.0 Leaderless Cold-Boot Auto-Election (All-Unjoined Self-Form)  */
/* ========================================================================== */
static void test_v60_leaderless_cold_boot_auto_election(void) {
    printf("[TEST 29] v6.0 Leaderless Cold-Boot Auto-Election (Zero Pre-Configured Master)...\n");

    sim_world_t w;
    sim_world_init(&w, 0x60A12026ULL);
    sim_world_setup_10node_demo(&w, false);

    /* Force ALL 10 nodes to boot as ON_ROLE_UNJOINED (zero pre-designated Primary G!) */
    for (uint32_t i = 0u; i < w.num_nodes; ++i) {
        on_hal_ops_t hal_copy = w.nodes[i].hal;
        uint8_t caps = (i < 8u) ? 0x03u : 0x00u;
        on_node_init(&w.nodes[i], &hal_copy, ON_ROLE_UNJOINED, caps);
        if (i < 8u) {
            /* Enable leaderless auto-election on all 1-hop-capable nodes (lowest MAC wins!) */
            on_node_enable_leaderless_boot(&w.nodes[i], 20u);
        }
    }

    /* Run 200 ms: lowest-MAC node (idx=0, mac[5]=1) elects itself G, all others auto-join! */
    sim_world_run_us(&w, 200000, 10);

    uint32_t g_count = 0u, joined_1h = 0u, joined_2h = 0u;
    for (uint32_t i = 0u; i < w.num_nodes; ++i) {
        if (w.nodes[i].role == ON_ROLE_G) g_count++;
        else if (w.nodes[i].role == ON_ROLE_T || w.nodes[i].role == ON_ROLE_SUB_G) {
            if (w.nodes[i].is_sub_node) joined_2h++;
            else joined_1h++;
        }
    }

    printf("    -> Leaderless Election: G_count=%u (id=0x%03X, backup=0x%03X, subg=0x%03X) | 1-hop=%u | 2-hop=%u | Collisions=%u\n",
           g_count, (unsigned)w.nodes[0].short_id, (unsigned)w.nodes[0].backup_id,
           (unsigned)w.nodes[0].sub_g_id, joined_1h, joined_2h,
           w.tdma_subg_collisions + w.cra_collisions);

    EXPECT(g_count == 1u && w.nodes[0].role == ON_ROLE_G,
           "Exactly one lowest-MAC node self-elected as Primary G without manual config");
    EXPECT(joined_1h == 7u && w.nodes[0].backup_id != ON_SHORT_ID_UNASSIGNED &&
           w.nodes[0].sub_g_id != ON_SHORT_ID_UNASSIGNED,
           "All 7 1-hop peers auto-joined, elected Backup_ID and Sub-G proxy");
    EXPECT(w.tdma_subg_collisions == 0u && w.cra_collisions == 0u,
           "Zero RF collisions during leaderless cold-boot self-organization");
}

/* ========================================================================== */
/* TEST 30: v6.0 6-Hop Hybrid Mesh Routing + Self-Healing + 256-Node Epoch    */
/* ========================================================================== */
static void test_v60_6hop_mesh_routing_and_256node_epoch_tdma(void) {
    printf("[TEST 30] v6.0 6-Hop Hybrid Mesh Routing, Parent Failover & 256-Node Epoch-TDMA...\n");

    /* 1. Build 6-hop chain: Root(0x001) - N1(0x002) - N2(0x100) - N3(0x301) - N4(0x401) - N5(0x501) - N6(0x601)
     *    plus standby 3-hop relay N3b(0x302) reachable from N2 and N4 for self-healing failover! */
    enum { IDX_G = 0, IDX_N1, IDX_N2, IDX_N3, IDX_N3B, IDX_N4, IDX_N5, IDX_N6, NUM_M = 8 };
    static const uint16_t kIds[NUM_M] = { 0x001u, 0x002u, 0x100u, 0x301u, 0x302u, 0x401u, 0x501u, 0x601u };

    on_mesh_ctx_t m[NUM_M];
    on_mesh_init(&m[IDX_G], kIds[IDX_G], 0u, ON_SHORT_ID_UNASSIGNED);
    for (int i = 1; i < NUM_M; ++i) {
        on_mesh_init(&m[i], kIds[i], 0xFFu, ON_SHORT_ID_UNASSIGNED);
    }

    /* Propagate beacons along links (N3 has higher LQI=220 than standby N3b LQI=180) */
    on_mesh_on_beacon(&m[IDX_N1],  kIds[IDX_G],   m[IDX_G].my_hop_depth,   240u, 10u);
    on_mesh_on_beacon(&m[IDX_N2],  kIds[IDX_N1],  m[IDX_N1].my_hop_depth,  230u, 10u);
    on_mesh_on_beacon(&m[IDX_N3],  kIds[IDX_N2],  m[IDX_N2].my_hop_depth,  220u, 10u);
    on_mesh_on_beacon(&m[IDX_N3B], kIds[IDX_N2],  m[IDX_N2].my_hop_depth,  190u, 10u);
    on_mesh_on_beacon(&m[IDX_N4],  kIds[IDX_N3],  m[IDX_N3].my_hop_depth,  220u, 10u);
    on_mesh_on_beacon(&m[IDX_N4],  kIds[IDX_N3B], m[IDX_N3B].my_hop_depth, 180u, 10u);
    on_mesh_on_beacon(&m[IDX_N5],  kIds[IDX_N4],  m[IDX_N4].my_hop_depth,  210u, 10u);
    on_mesh_on_beacon(&m[IDX_N6],  kIds[IDX_N5],  m[IDX_N5].my_hop_depth,  200u, 10u);

    EXPECT(m[IDX_N6].my_hop_depth == 6u && m[IDX_N4].parent_id == kIds[IDX_N3],
           "6-hop basement sensor N6 auto-discovers 6-hop tree & N4 picks higher-LQI parent N3");

    /* Send 6-hop uplink telemetry from N6 -> N5 -> N4 -> N3 -> N2 -> N1 -> Root G */
    const uint8_t up_data[4] = { 0x66u, 0x77u, 0x88u, 0x99u };
    on_mesh_pkt_t pkt, fwd;
    EXPECT(on_mesh_build_uplink(&m[IDX_N6], ON_MESH_DATA_UP, up_data, 4u, &pkt) == ON_OK,
           "6-hop N6 builds uplink packet");

    static const int kUpPath[6] = { IDX_N5, IDX_N4, IDX_N3, IDX_N2, IDX_N1, IDX_G };
    uint16_t prev_id = kIds[IDX_N6];
    on_mesh_action_t act = ON_MESH_ACT_DROP;
    for (int step = 0; step < 6; ++step) {
        int cur = kUpPath[step];
        act = on_mesh_process_rx(&m[cur], prev_id, &pkt, 12u, &fwd);
        prev_id = kIds[cur];
        if (step < 5) {
            EXPECT(act == ON_MESH_ACT_FORWARD, "Intermediate relay forwards 6-hop uplink");
            pkt = fwd;
        }
    }
    EXPECT(act == ON_MESH_ACT_DELIVER && pkt.src_id == kIds[IDX_N6] &&
           pkt.hop_count == 6u && memcmp(pkt.payload, up_data, 4u) == 0,
           "6-hop uplink telemetry delivered intact to Root G across 6 hops");

    /* Verify duplicate rejection on replay */
    EXPECT(on_mesh_process_rx(&m[IDX_G], kIds[IDX_N1], &pkt, 13u, &fwd) == ON_MESH_ACT_DROP &&
           m[IDX_G].dedup_drops == 1u,
           "32-entry LRU dedup cache rejects duplicate mesh packet");

    /* Send tree-routed downlink command from Root G -> N1 -> N2 -> N3 -> N4 -> N5 -> N6 (ZERO flood!) */
    const char *cmd = "VALVE_OPEN";
    EXPECT(on_mesh_build_downlink(&m[IDX_G], kIds[IDX_N6], (const uint8_t *)cmd, 10u, &pkt) == ON_OK &&
           pkt.next_hop_id == kIds[IDX_N1] && (pkt.flags & ON_MESH_FLAG_FLOOD_FALLBACK) == 0u,
           "Root G routes downlink strictly along learned reverse tree (no flood)");

    static const int kDownPath[6] = { IDX_N1, IDX_N2, IDX_N3, IDX_N4, IDX_N5, IDX_N6 };
    prev_id = kIds[IDX_G];
    for (int step = 0; step < 6; ++step) {
        int cur = kDownPath[step];
        act = on_mesh_process_rx(&m[cur], prev_id, &pkt, 14u, &fwd);
        prev_id = kIds[cur];
        if (step < 5) {
            EXPECT(act == ON_MESH_ACT_FORWARD, "Intermediate relay tree-routes downlink");
            pkt = fwd;
        }
    }
    EXPECT(act == ON_MESH_ACT_DELIVER && memcmp(pkt.payload, cmd, 10u) == 0,
           "6-hop downlink command 'VALVE_OPEN' delivered to basement sensor N6");

    /* Kill relay N3 (only standby N3b refreshes beacon at SF=50) -> N4 self-heals to N3b! */
    on_mesh_on_beacon(&m[IDX_N4], kIds[IDX_N3B], m[IDX_N3B].my_hop_depth, 185u, 50u);
    EXPECT(m[IDX_N4].parent_id == kIds[IDX_N3B] && m[IDX_N4].parent_switches == 1u,
           "When 3-hop relay N3 dies, N4 self-heals to standby relay N3b automatically");

    /* 2. Verify 256-Node Multi-Epoch TDMA Zero-Collision Scheduler (mixed periods 1, 8, 32) */
    on_epoch_sched_t sched;
    on_epoch_sched_init(&sched);
    on_epoch_grant_t grants[256];
    uint32_t allocated = 0u;

    /* Allocate 1 high-rate 250 Hz actuator (period=1, 1 full slot), 8 medium sensors (period=8, 1 slot),
     * and 192 sleeping IoT sensors (period=32, 6 slots) = 201 heterogeneous nodes filling 100% of 8x32 matrix! */
    for (uint16_t i = 0u; i < 1u; ++i) {
        if (on_epoch_alloc(&sched, (uint16_t)(0x010u + i), 1u, 0u, &grants[allocated]) == ON_OK) allocated++;
    }
    for (uint16_t i = 0u; i < 8u; ++i) {
        if (on_epoch_alloc(&sched, (uint16_t)(0x020u + i), 8u, 0u, &grants[allocated]) == ON_OK) allocated++;
    }
    for (uint16_t i = 0u; i < 192u; ++i) {
        if (on_epoch_alloc(&sched, (uint16_t)(0x100u + i), 32u, 0u, &grants[allocated]) == ON_OK) allocated++;
    }

    /* Exhaustive collision check across every (sf, slot) in a full 32-SF epoch */
    uint32_t epoch_collisions = 0u;
    for (uint32_t sf = 0u; sf < 32u; ++sf) {
        uint8_t slot_hits[ON_EPOCH_MAX_SLOTS];
        memset(slot_hits, 0, sizeof(slot_hits));
        for (uint32_t i = 0u; i < allocated; ++i) {
            if (on_epoch_is_my_sf(sf, grants[i].epoch_phase, grants[i].epoch_period)) {
                slot_hits[grants[i].slot_idx]++;
                if (slot_hits[grants[i].slot_idx] > 1u) epoch_collisions++;
            }
        }
    }

    /* Pure 256-node period=32 capacity test */
    on_epoch_sched_t sched256;
    on_epoch_sched_init(&sched256);
    uint32_t alloc256 = 0u;
    for (uint16_t i = 0u; i < 256u; ++i) {
        on_epoch_grant_t g;
        if (on_epoch_alloc(&sched256, (uint16_t)(0x200u + i), 32u, 0u, &g) == ON_OK) alloc256++;
    }

    printf("    -> 6-Hop Mesh: N6 depth=%u failover=N3->N3b | Epoch-TDMA: mixed=%u (coll=%u), max_cap=%u, duty(p32)=%.2f%%\n",
           (unsigned)m[IDX_N6].my_hop_depth, allocated, epoch_collisions, alloc256,
           on_epoch_duty_cycle_pct(32u));

    EXPECT(allocated == 201u && epoch_collisions == 0u,
           "Heterogeneous Epoch-TDMA (250Hz + 31Hz + 7.8Hz) schedules 201 nodes with ZERO collisions");
    EXPECT(alloc256 == 256u && on_epoch_duty_cycle_pct(32u) < 0.35,
           "Full Epoch-TDMA supports 256 sleeping IoT nodes per cluster at < 0.35% duty cycle");
}

/* ========================================================================== */
/* TEST 31: v6.0 OASIS MQTT 3.1.1 Codec & Home Assistant Auto-Discovery       */
/* ========================================================================== */
typedef struct {
    uint32_t pub_count;
    char     last_topic[64];
    char     last_payload[256];
} test_mqtt_mock_t;

static on_err_t mock_mqtt_pub(void *ctx, const char *topic, const char *payload, bool retain) {
    (void)retain;
    test_mqtt_mock_t *m = (test_mqtt_mock_t *)ctx;
    m->pub_count++;
    strncpy(m->last_topic, topic, sizeof(m->last_topic) - 1u);
    strncpy(m->last_payload, payload, sizeof(m->last_payload) - 1u);
    return ON_OK;
}

static void test_v60_mqtt311_and_home_assistant_discovery(void) {
    printf("[TEST 31] v6.0 OASIS MQTT 3.1.1 Wire Codec & Home Assistant Auto-Discovery...\n");

    /* 1. CONNECT / CONNACK / SUBSCRIBE / PINGREQ round-trip */
    uint8_t buf[512];
    size_t clen = on_mqtt_encode_connect("opennear_gw_01", "ha_user", "secret", 30u, buf, sizeof(buf));
    EXPECT(clen > 20u && buf[0] == ON_MQTT_PKT_CONNECT && memcmp(buf + 4, "MQTT", 4u) == 0,
           "MQTT 3.1.1 CONNECT packet encoded cleanly with auth & keepalive");

    const uint8_t connack_ok[4] = { 0x20u, 0x02u, 0x00u, 0x00u };
    uint8_t rc = 0xFFu;
    EXPECT(on_mqtt_decode_connack(connack_ok, sizeof(connack_ok), &rc) && rc == 0u,
           "MQTT CONNACK accepted (rc=0) decoded cleanly");

    size_t slen = on_mqtt_encode_subscribe(1u, "opennear/cmd/#", buf, sizeof(buf));
    EXPECT(slen > 18u && buf[0] == ON_MQTT_PKT_SUBSCRIBE,
           "MQTT SUBSCRIBE for opennear/cmd/# encoded cleanly");

    /* 2. Home Assistant MQTT Auto-Discovery JSON + PUBLISH encode/decode */
    char ha_topic[96], ha_json[512];
    EXPECT(on_mqtt_format_ha_discovery(0x601u, 6u, "T", ha_topic, sizeof(ha_topic),
                                       ha_json, sizeof(ha_json)) &&
           strcmp(ha_topic, "homeassistant/sensor/opennear_601/config") == 0 &&
           strstr(ha_json, "\"stat_t\":\"opennear/node/0x601/state\"") != NULL &&
           strstr(ha_json, "\"cmd_t\":\"opennear/cmd/0x601\"") != NULL,
           "Home Assistant Discovery topic & JSON generated for 6-hop Mesh node 0x601");

    size_t plen = on_mqtt_encode_publish(ha_topic, (const uint8_t *)ha_json, strlen(ha_json),
                                         true, buf, sizeof(buf));
    on_mqtt_pub_view_t view;
    EXPECT(plen > strlen(ha_json) && on_mqtt_decode_publish(buf, plen, &view) &&
           view.retain && view.topic_len == strlen(ha_topic) &&
           view.payload_len == strlen(ha_json) &&
           memcmp(view.payload, ha_json, view.payload_len) == 0,
           "MQTT retained PUBLISH encode/decode round-trip matches bit-for-bit");

    /* 3. Inbound MQTT Command & OTA Trigger topic parser + BLE MQTT provisioning + Gateway forward */
    uint16_t tid = 0u;
    bool is_ota = false;
    EXPECT(on_mqtt_parse_cmd_topic("opennear/cmd/0x601", 18u, &tid, &is_ota) &&
           tid == 0x601u && !is_ota,
           "Inbound Home Assistant command topic opennear/cmd/0x601 resolves target=0x601");
    EXPECT(on_mqtt_parse_cmd_topic("opennear/ota/trigger", 20u, &tid, &is_ota) && is_ota,
           "Inbound MQTT OTA trigger topic opennear/ota/trigger detected");

    test_mqtt_mock_t mm;
    memset(&mm, 0, sizeof(mm));
    on_gw_transport_ops_t ops;
    memset(&ops, 0, sizeof(ops));
    ops.user_ctx     = &mm;
    ops.mqtt_publish = mock_mqtt_pub;

    on_gw_ctx_t gw;
    on_gw_init(&gw, ON_GW_MODE_WIFI_AND_BLE, &ops);
    const char *mprov = "MQTT:192.168.1.50|1883|ha_user|ha_pass";
    EXPECT(on_gw_handle_control_text(&gw, mprov, strlen(mprov), true) == ON_OK &&
           strcmp(gw.cfg.mqtt_host, "192.168.1.50") == 0 && gw.cfg.mqtt_port == 1883u &&
           strcmp(gw.cfg.mqtt_user, "ha_user") == 0,
           "BLE/HTTP MQTT:<host>|<port>|<user>|<pass> provisioning parsed cleanly");

    gw.mqtt_connected = true;
    on_gw_bridge_msg_t up;
    memset(&up, 0, sizeof(up));
    up.bridge_type = ON_GW_MSG_TELEMETRY_UP;
    up.origin_id   = 0x601u;
    up.seq         = 7u;
    up.payload[0]  = 0xABu;
    up.payload[1]  = 0xCDu;
    up.payload_len = 2u;
    on_gw_push_uplink(&gw, &up);
    on_gw_poll_network(&gw);

    printf("    -> HA Discovery: %s | MQTT Pub: %s -> %s\n", ha_topic, mm.last_topic, mm.last_payload);
    EXPECT(mm.pub_count == 1u && strcmp(mm.last_topic, "opennear/node/0x601/state") == 0 &&
           strstr(mm.last_payload, "ABCD") != NULL,
           "Gateway poller forwards 6-hop node telemetry directly to MQTT broker topic");
}

/* ========================================================================== */
/* TEST 32: v6.1 Audit Regressions (Streaming OTA MAC, SPSC Ring, Token, RX)  */
/* ========================================================================== */
static on_err_t test_mem_chunk_reader(void *ctx, uint32_t offset, uint8_t *out, size_t len) {
    const uint8_t *src = (const uint8_t *)ctx;
    memcpy(out, src + offset, len);
    return ON_OK;
}

static void test_v61_audit_regressions(void) {
    printf("[TEST 32] v6.1 Audit Regressions (Streaming OTA MAC + Lock-Free SPSC + Admin Token + Epoch RX Sleep)...\n");

    /* 1. Streaming flash-reader OTA auth tag must match contiguous RAM tag bit-for-bit */
    static const uint8_t kKey[ON_SEC_KEY_LEN] = {
        0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF0,
        0xFE,0xDC,0xBA,0x98,0x76,0x54,0x32,0x10
    };
    uint8_t img[333];
    for (size_t i = 0u; i < sizeof(img); ++i) img[i] = (uint8_t)((i * 19u + 7u) & 0xFFu);
    on_ota_meta_t meta;
    memset(&meta, 0, sizeof(meta));
    meta.ota_id     = 0x7777u;
    meta.fw_version = 0x00060100u;
    meta.total_size = sizeof(img);
    meta.crc32      = on_ota_crc32(img, sizeof(img));
    meta.hw_compat  = 1u;

    uint8_t tag_mem[ON_OTA_AUTH_TAG_LEN], tag_stream[ON_OTA_AUTH_TAG_LEN];
    on_ota_compute_auth_tag(kKey, &meta, img, tag_mem);
    EXPECT(on_ota_compute_auth_tag_reader(kKey, &meta, test_mem_chunk_reader, img, tag_stream) == ON_OK &&
           memcmp(tag_mem, tag_stream, ON_OTA_AUTH_TAG_LEN) == 0,
           "Streaming flash-reader OTA CBC-MAC matches contiguous memory MAC bit-exact");

    /* 2. Lock-free SPSC ring buffer wraparound & capacity stress (64 cycles) */
    on_gw_ctx_t gw;
    on_gw_init(&gw, ON_GW_MODE_WIFI_AND_BLE, NULL);
    uint32_t popped_ok = 0u;
    for (uint16_t round = 0u; round < 64u; ++round) {
        on_gw_bridge_msg_t in, out;
        memset(&in, 0, sizeof(in));
        in.seq = round;
        in.origin_id = 0x100u;
        EXPECT(on_gw_push_uplink(&gw, &in), "SPSC push succeeds");
        if (on_gw_pop_uplink(&gw, &out) && out.seq == round) popped_ok++;
    }
    EXPECT(popped_ok == 64u && on_gw_ring_count(&gw.uplink_q) == 0u,
           "Lock-free SPSC ring wraps cleanly across 64 cycles with zero lost frames");

    /* 3. Gateway Admin Token enforcement on BLE/HTTP control commands */
    on_gw_set_admin_token(&gw, "PIN9876");
    EXPECT(on_gw_handle_control_text(&gw, "CMD:100|HACK", 12u, true) == ON_ERR_DECODE_FAIL,
           "Unauthorized control command rejected when admin_token is set");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:WRONG|CMD:100|HACK", 23u, true) == ON_ERR_DECODE_FAIL,
           "Wrong admin token rejected");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:PIN9876|CMD:100|SAFE", 25u, true) == ON_OK &&
           on_gw_ring_count(&gw.downlink_q) == 1u,
           "Valid AUTH:<token>|CMD:... accepted and queued");
    EXPECT(on_gw_handle_control_text(&gw, "TOPO?", 5u, true) == ON_OK,
           "Read-only TOPO? query still permitted with admin_token active");

    /* 4. Sleeping Epoch-TDMA RX gate wakes only on scheduled epoch (1 out of 16 SFs) */
    on_node_t slp;
    on_node_init(&slp, NULL, ON_ROLE_T, 0u);
    slp.short_id              = 0x050u;
    slp.sync.locked           = true;
    slp.sync.lock_count       = 10u;
    slp.current_sf_start_net_ns = 0;
    on_node_set_epoch_schedule(&slp, 3u, 16u);
    uint32_t wake_sfs = 0u;
    for (uint32_t sf = 0u; sf < 16u; ++sf) {
        slp.current_sf_seq = sf;
        slp.current_sf_start_net_ns = (int64_t)sf * 4000000LL;
        if (on_node_is_rx_open(&slp, slp.current_sf_start_net_ns + 50000LL)) wake_sfs++;
    }
    printf("    -> Epoch-TDMA (period=16) SYNC RX wakeups: %u/16 SFs (%.2f%% duty)\n",
           wake_sfs, on_epoch_duty_cycle_pct(16u));
    EXPECT(wake_sfs == 1u, "Sleeping Epoch-TDMA node opens SYNC RX only on 1/16 scheduled superframes");
}

/* ========================================================================== */
/* Test 33: v6.2 Deep Hardening Regressions                                   */
/* ========================================================================== */
static void test_v62_deep_hardening_regressions(void) {
    printf("[TEST 33] v6.2 Deep Hardening (MQTT Stream Framing + Replay LRU + Epoch Sleep + Mesh Loop Guard + Token Rotate)...\n");

    /* 1. Coalesced TCP MQTT stream: [CONNACK][SUBACK][PINGRESP][PUBLISH #1][PUBLISH #2] */
    uint8_t stream[256];
    size_t pos = 0u;
    /* CONNACK (4B) */
    stream[pos++] = ON_MQTT_PKT_CONNACK; stream[pos++] = 2u; stream[pos++] = 0u; stream[pos++] = 0u;
    /* SUBACK (5B) */
    stream[pos++] = ON_MQTT_PKT_SUBACK; stream[pos++] = 3u; stream[pos++] = 0u; stream[pos++] = 1u; stream[pos++] = 0u;
    /* PINGRESP (2B) */
    stream[pos++] = ON_MQTT_PKT_PINGRESP; stream[pos++] = 0u;
    /* Two back-to-back PUBLISH packets */
    size_t p1 = on_mqtt_encode_publish("opennear/cmd/0x100", (const uint8_t *)"ON", 2u, false,
                                       stream + pos, sizeof(stream) - pos);
    pos += p1;
    size_t p2 = on_mqtt_encode_publish("opennear/cmd/0x200", (const uint8_t *)"OFF", 3u, false,
                                       stream + pos, sizeof(stream) - pos);
    pos += p2;

    /* Partial packet check (first 1 byte only) */
    uint8_t ptype = 0u;
    size_t plen = 0u;
    EXPECT(on_mqtt_next_packet(stream, 1u, &ptype, &plen) == 0,
           "MQTT stream framer returns 0 (need more bytes) on truncated header");

    /* Drain full coalesced buffer */
    size_t off = 0u;
    uint32_t total_pkts = 0u, pub_pkts = 0u;
    uint16_t targets[2] = {0u, 0u};
    while (off < pos) {
        int st = on_mqtt_next_packet(stream + off, pos - off, &ptype, &plen);
        if (st != 1) break;
        total_pkts++;
        if (ptype == ON_MQTT_PKT_PUBLISH) {
            on_mqtt_pub_view_t pv;
            if (on_mqtt_decode_publish(stream + off, plen, &pv)) {
                uint16_t tid = 0u;
                bool is_ota = false;
                if (on_mqtt_parse_cmd_topic(pv.topic, pv.topic_len, &tid, &is_ota) && !is_ota) {
                    if (pub_pkts < 2u) targets[pub_pkts] = tid;
                    pub_pkts++;
                }
            }
        }
        off += plen;
    }
    EXPECT(off == pos && total_pkts == 5u && pub_pkts == 2u &&
           targets[0] == 0x100u && targets[1] == 0x200u,
           "MQTT stream framer drains coalesced CONNACK+SUBACK+PINGRESP+2xPUBLISH without dropping commands");

    /* 2. Anti-replay table LRU eviction beyond ON_SEC_REPLAY_SLOTS (never fails open) */
    on_sec_ctx_t sec;
    on_sec_init(&sec);
    static const uint8_t kKey[ON_SEC_KEY_LEN] = {
        0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x01
    };
    on_sec_set_key(&sec, kKey);
    for (uint16_t i = 0u; i < 40u; ++i) {
        EXPECT(on_sec_check_replay(&sec, (uint16_t)(0x010u + i), 100u + i, 1u, ON_FRAME_TDMA_DATA),
               "Fresh packet accepted for node beyond replay table capacity");
    }
    EXPECT(!on_sec_check_replay(&sec, 0x037u, 139u, 1u, ON_FRAME_TDMA_DATA) && sec.replay_drops >= 1u,
           "40th node still protected against replay after LRU eviction of oldest slot");

    /* 3. Epoch-TDMA planned sleep keeps missed_sync_count=0 & prevents spurious Backup Takeover */
    on_node_t slp;
    on_node_init(&slp, NULL, ON_ROLE_T, 0x03u);
    slp.short_id                = 0x040u;
    slp.backup_id               = 0x040u; /* Even if designated backup before epoch config */
    slp.sync.locked             = true;
    slp.sync.lock_count         = 10u;
    slp.current_sf_seq          = 0u;
    slp.current_sf_start_net_ns = 0;
    on_node_set_epoch_schedule(&slp, 0u, 16u);
    for (uint32_t sf = 1u; sf < 15u; ++sf) {
        int64_t t = (int64_t)sf * 4000000LL + 350000LL;
        on_node_step(&slp, t);
    }
    EXPECT((slp.capabilities & 0x01u) == 0u && slp.missed_sync_count == 0u && slp.role == ON_ROLE_T,
           "Epoch-TDMA planned sleep keeps missed_sync_count=0 and blocks spurious Backup Takeover");

    /* 4. Mesh neighbor loop-guard & parent-safe eviction when neighbor table is full */
    on_mesh_ctx_t mesh;
    on_mesh_init(&mesh, 0x020u, 0xFFu, ON_SHORT_ID_UNASSIGNED);
    on_mesh_on_beacon(&mesh, 0x010u, 1u, 240u, 10u); /* Best hop-1 parent -> my_hop_depth = 2 */
    for (uint16_t k = 1u; k < ON_MESH_MAX_NEIGHBORS; ++k) {
        on_mesh_on_beacon(&mesh, (uint16_t)(0x010u + k), 1u, (uint8_t)(200u - k * 5u), 10u);
    }
    /* Inject deeper neighbor (loop candidate) & weaker hop-1 neighbor into full table */
    on_mesh_on_beacon(&mesh, 0x099u, 2u, 255u, 11u);
    on_mesh_on_beacon(&mesh, 0x098u, 1u, 50u,  11u);
    EXPECT(mesh.parent_id == 0x010u && mesh.my_hop_depth == 2u,
           "Mesh neighbor loop-guard & smart eviction never overwrite active parent");

    /* 5. Runtime Gateway TOKEN:<new_token> rotation & clear */
    on_gw_ctx_t gw;
    on_gw_init(&gw, ON_GW_MODE_WIFI_AND_BLE, NULL);
    EXPECT(on_gw_handle_control_text(&gw, "TOKEN:INIT123", 13u, false) == ON_OK &&
           gw.cfg.admin_token_updated && strcmp(gw.cfg.admin_token, "INIT123") == 0,
           "Initial TOKEN:<pin> accepted when no token is configured");
    gw.cfg.admin_token_updated = false;
    EXPECT(on_gw_handle_control_text(&gw, "TOKEN:HACK", 10u, true) == ON_ERR_DECODE_FAIL,
           "Unauthenticated TOKEN rotation rejected once token is active");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:INIT123|TOKEN:NEXT456", 26u, true) == ON_OK &&
           gw.cfg.admin_token_updated && strcmp(gw.cfg.admin_token, "NEXT456") == 0,
           "Authenticated TOKEN rotation succeeds and flags NVS persistence");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:NEXT456|TOKEN:", 19u, true) == ON_OK &&
           gw.cfg.admin_token[0] == '\0',
           "Authenticated empty TOKEN: cleanly clears admin token");

    /* 6. Mesh identity auto-sync when unjoined node promotes via Leaderless Boot */
    on_node_t lb;
    on_mesh_ctx_t lb_mesh;
    on_node_init(&lb, NULL, ON_ROLE_UNJOINED, 0x03u);
    lb.mac48[5] = 0x01u;
    on_mesh_init(&lb_mesh, ON_SHORT_ID_UNASSIGNED, 0xFFu, ON_SHORT_ID_UNASSIGNED);
    on_node_attach_mesh(&lb, &lb_mesh);
    on_node_enable_leaderless_boot(&lb, 20u);
    on_node_step(&lb, 1000000LL);
    on_node_step(&lb, 30000000LL);
    EXPECT(lb.role == ON_ROLE_G && lb_mesh.my_id == ON_SHORT_ID_G_DEFAULT && lb_mesh.my_hop_depth == 0u,
           "Mesh identity auto-syncs to root (id=0x001, hop=0) upon leaderless G election");
    printf("    -> MQTT 5-pkt coalesced stream OK | Replay LRU(40>32) OK | Epoch sleep missed=0 | Mesh root sync OK\n");
}

static void test_independent_act_clock(void) {
    printf("[TEST ACT CLOCK] Low-delay ACT phase estimate isolated from TDMA...\n");
    on_node_t filtered, control;
    on_node_init(&filtered, NULL, ON_ROLE_UNJOINED, 0);
    on_node_init(&control, NULL, ON_ROLE_UNJOINED, 0);
    const uint8_t key[16] = {1, 2, 3, 4};
    on_node_enable_security(&filtered, key);
    on_node_enable_security(&control, key);
    filtered.act_lookahead_enabled = control.act_lookahead_enabled = true;
    filtered.act_clock_enabled = true;
    filtered.act_clock_bias_us = 750u;
    const int64_t base_ns = 98000000LL;
    const int64_t sf_ns = ON_DEFAULT_TSF_US * 1000LL;
    const uint32_t target_sf = 1000u;
    /* +100 ppm synthetic clock, one clean sample per eight beacons, with every
     * fifth block entirely queued. TDMA's RX mapping has a separate error. */
    for (uint32_t seq = 0; seq < target_sf; ++seq) {
        int64_t tx = seq * sf_ns;
        int64_t truth = tx + base_ns + tx / 10000LL;
        int64_t extra = (seq % 8u != 0u || (seq / 40u) % 5u == 2u) ? 3200000LL : 0LL;
        uint8_t lead = seq >= target_sf - 7u ? (uint8_t)(target_sf - seq) : 0u;
        uint8_t level = (uint8_t)((seq / 10u) & 1u);
        int64_t act_rx = truth + 750000LL + extra;
        int64_t rx = act_rx + 2000000LL;
        act_test_beacon_timed(&filtered, 1u, seq, 20u, level, lead, tx, rx, act_rx, false);
        act_test_beacon_timed(&control, 1u, seq, 20u, level, lead, tx, rx, act_rx, false);
    }
    const int64_t target_net = target_sf * sf_ns + ON_DEFAULT_ACT_FIRE_US * 1000LL;
    const int64_t truth = target_net + base_ns + target_net / 10000LL;
    const int64_t error = filtered.act_lookahead.deadline_local_ns - truth;
    EXPECT(filtered.act_lookahead.pending && filtered.act_lookahead.target_sf == target_sf &&
           llabs(error) < 100000LL && control.act_lookahead.deadline_local_ns - truth > 2000000LL,
           "Rolling ACT clock survives multi-ms delay bursts and maps the target within 100 us");
    EXPECT(filtered.act_clock.rate_ppm >= 70 && filtered.act_clock.rate_ppm <= 130,
           "Independent ACT estimator tracks synthetic +100 ppm crystal drift");
    EXPECT(filtered.sync.ref_local_ns == control.sync.ref_local_ns &&
           filtered.sync.ref_net_ns == control.sync.ref_net_ns &&
           filtered.sync.rate_ppb == control.sync.rate_ppb &&
           filtered.current_sf_seq == control.current_sf_seq,
           "ACT estimator does not feed timestamps, drift or phase into TDMA");
    const int64_t last_net = filtered.act_clock.last_net_ns;
    act_test_beacon_timed(&filtered, 1u, target_sf - 1u, 20u, 1u, 1u, last_net, truth, 0LL, false);
    act_test_beacon_timed(&filtered, 1u, target_sf, 20u, 0u, 0u, target_sf * sf_ns, truth, 0LL, true);
    EXPECT(filtered.act_clock.last_net_ns == last_net &&
           filtered.act_lookahead.deadline_local_ns == truth + error,
           "Duplicate and unauthenticated beacons cannot move the independent clock");
    on_node_step(&filtered, truth + error);
    EXPECT(filtered.act_cmd == 0u && filtered.last_act_was_scheduled,
           "ACT fires at its independent deadline when the target beacon is lost");
    const uint32_t resume = target_sf + 400u;
    act_test_beacon_timed(&filtered, 1u, resume, 20u, 0u, 0u,
                         resume * sf_ns, resume * sf_ns + base_ns, resume * sf_ns + base_ns, false);
    EXPECT(filtered.act_clock.last_net_ns == resume * sf_ns && filtered.act_clock.rate_ppm == 0,
           "A longer outage discards stale ACT observations on resume");
    act_test_beacon_timed(&filtered, 1u, 0u, 21u, 0u, 0u, 0LL,
                         6100000000LL, 6100000000LL, false);
    unsigned valid = 0;
    for (unsigned i = 0; i < ON_ACT_CLOCK_BLOCKS; ++i) valid += filtered.act_clock.blocks[i].valid;
    EXPECT(filtered.act_clock.valid && filtered.act_clock.last_net_ns == 0 && valid == 1 &&
           !filtered.act_lookahead.pending, "Primary reboot invalidates prior ACT clock and plan");
}

/* ========================================================================== */
/* Test 34: v6.4 Optional MQTTS / TLS Cloud Configuration & Switching         */
/* ========================================================================== */
static void test_v64_optional_mqtts_tls_config(void) {
    printf("[TEST 34] v6.4 Optional MQTTS/TLS Config (MQTTS: shorthand + port 8883 auto-TLS + explicit flag)...\n");
    on_gw_ctx_t gw;
    on_gw_init(&gw, ON_GW_MODE_WIFI_AND_BLE, NULL);

    /* 1. Plain MQTT on 1883 */
    EXPECT(on_gw_handle_control_text(&gw, "MQTT:192.168.1.10|1883|ha|sec", 29u, false) == ON_OK &&
           !gw.cfg.mqtt_use_tls && gw.cfg.mqtt_port == 1883u &&
           strcmp(gw.cfg.mqtt_host, "192.168.1.10") == 0,
           "Plain MQTT on port 1883 keeps mqtt_use_tls=false");

    /* 2. Auto-TLS when port 8883 is selected */
    EXPECT(on_gw_handle_control_text(&gw, "MQTT:broker.emqx.io|8883|u1|p1", 30u, true) == ON_OK &&
           gw.cfg.mqtt_use_tls && gw.cfg.mqtt_port == 8883u,
           "MQTT on standard TLS port 8883 automatically enables mqtt_use_tls=true");

    /* 3. Explicit MQTTS: shorthand (defaults to 8883 when port=0) */
    EXPECT(on_gw_handle_control_text(&gw, "MQTTS:a1.iot.aws.com|0|dev|tok", 30u, false) == ON_OK &&
           gw.cfg.mqtt_use_tls && gw.cfg.mqtt_port == 8883u &&
           strcmp(gw.cfg.mqtt_host, "a1.iot.aws.com") == 0,
           "MQTTS: shorthand forces TLS and defaults port to 8883");

    /* 4. Explicit TLS override on custom port (|1 and |0) */
    EXPECT(on_gw_handle_control_text(&gw, "MQTT:cloud.example.com|443|u|p|1", 32u, false) == ON_OK &&
           gw.cfg.mqtt_use_tls && gw.cfg.mqtt_port == 443u,
           "Explicit trailing |1 enables MQTTS on custom port 443");
    EXPECT(on_gw_handle_control_text(&gw, "MQTT:10.0.0.5|8883|||0", 22u, false) == ON_OK &&
           !gw.cfg.mqtt_use_tls && gw.cfg.mqtt_port == 8883u,
           "Explicit trailing |0 allows disabling TLS even on port 8883");
    printf("    -> MQTT plain(1883) / auto-TLS(8883) / MQTTS shorthand / custom port TLS(443) all verified\n");
}

/* ========================================================================== */
/* Test 35: v6.5 Cluster-Wide Permanent MAC48 Propagation & HA MAC Discovery  */
/* ========================================================================== */
static void test_v65_cluster_mac48_and_ha_identity(void) {
    printf("[TEST 35] v6.5 Hardware MAC48 Propagation (0-Hop G + 1-Hop + 2-Hop Edge) & HA MAC Entity...\n");

    sim_world_t w;
    sim_world_init(&w, 0x65C12026ULL);
    sim_world_setup_10node_demo(&w, true);
    /* Run enough superframes for 1-hop + 2-hop join and Sub-G MAC_REPORT propagation to Primary G */
    sim_world_run_us(&w, 240000, 10);

    char g_json[3072];
    size_t jlen = on_node_format_cluster_json(&w.nodes[0], true, g_json, sizeof(g_json));

    char g_mac[18], t4_mac[18], t8_mac[18];
    snprintf(g_mac, sizeof(g_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             w.nodes[0].mac48[0], w.nodes[0].mac48[1], w.nodes[0].mac48[2],
             w.nodes[0].mac48[3], w.nodes[0].mac48[4], w.nodes[0].mac48[5]);
    snprintf(t4_mac, sizeof(t4_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             w.nodes[4].mac48[0], w.nodes[4].mac48[1], w.nodes[4].mac48[2],
             w.nodes[4].mac48[3], w.nodes[4].mac48[4], w.nodes[4].mac48[5]);
    snprintf(t8_mac, sizeof(t8_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             w.nodes[8].mac48[0], w.nodes[8].mac48[1], w.nodes[8].mac48[2],
             w.nodes[8].mac48[3], w.nodes[8].mac48[4], w.nodes[8].mac48[5]);

    EXPECT(jlen > 200u && strstr(g_json, g_mac) != NULL &&
           strstr(g_json, t4_mac) != NULL && strstr(g_json, t8_mac) != NULL,
           "Primary G JSON includes permanent MAC48 for 0-hop G, 1-hop T4, and wall-blocked 2-hop T8");

    /* Remote 1-hop node T5 fetches paginated snapshot and also receives 2-hop T8 MAC48 */
    EXPECT(on_node_request_cluster_snapshot(&w.nodes[5]) == ON_OK,
           "1-hop T5 requests cluster snapshot with MAC48");
    sim_world_run_us(&w, 160000, 10);
    char t5_json[3072];
    on_node_format_cluster_json(&w.nodes[5], true, t5_json, sizeof(t5_json));
    EXPECT(w.nodes[5].cluster_snapshot_valid && strstr(t5_json, t8_mac) != NULL,
           "Remote 1-hop T5 receives 2-hop T8 permanent MAC48 via paginated air snapshot");
    EXPECT(w.tdma_subg_collisions == 0u && w.cra_collisions == 0u,
           "Zero RF collisions during 2-hop MAC48 reporting and snapshot pagination");

    /* Verify MAC-anchored Home Assistant Discovery & State JSON */
    const uint8_t sample_mac[6] = { 0xA4u, 0xCFu, 0x12u, 0x34u, 0x56u, 0x78u };
    const uint8_t sample_pay[2] = { 0xABu, 0xCDu };
    char topic[96], payload[512];
    EXPECT(on_mqtt_format_ha_discovery_mac(0x004u, sample_mac, 2u, "T",
                                           topic, sizeof(topic), payload, sizeof(payload)) &&
           strcmp(topic, "homeassistant/sensor/opennear_A4CF12345678/config") == 0 &&
           strstr(payload, "\"uniq_id\":\"opennear_A4CF12345678\"") != NULL &&
           strstr(payload, "[\"mac\",\"A4:CF:12:34:56:78\"]") != NULL,
           "Home Assistant discovery anchors uniq_id & device connections to permanent MAC48");
    EXPECT(on_mqtt_format_node_state_mac(0x004u, sample_mac, 0x001u, 1u, -12,
                                         sample_pay, 2u, topic, sizeof(topic), payload, sizeof(payload)) &&
           strstr(payload, "\"mac\":\"A4:CF:12:34:56:78\"") != NULL,
           "MQTT node state JSON embeds permanent MAC48 alongside dynamic Short-ID");
    printf("    -> Cluster MACs verified: G=%s T4=%s T8(2-hop)=%s | HA uniq_id=opennear_A4CF12345678\n",
           g_mac, t4_mac, t8_mac);
}

/* ========================================================================== */
/* Test 36: v6.6 Multi-Hop Ghost-Node GC, 3~6-Hop MAC48 & System Reset/Reboot */
/* ========================================================================== */
static void test_v66_multihop_gc_6hop_mac_and_system_cmds(void) {
    printf("[TEST 36] v6.6 Multi-Hop Ghost-Node GC, 3~6-Hop MAC48 + 3-Bit Hop Depth & System Commands...\n");

    /* 1. Verify 3~6-hop Mesh ON_MESH_TOPO_REPORT MAC48 ingestion & 3-bit hop_depth roundtrip */
    sim_world_t w;
    sim_world_init(&w, 0x66C12026ULL);
    sim_world_setup_10node_demo(&w, true);
    on_mesh_ctx_t g_mesh;
    on_mesh_init(&g_mesh, ON_SHORT_ID_G_DEFAULT, 0u, ON_SHORT_ID_UNASSIGNED);
    on_node_attach_mesh(&w.nodes[0], &g_mesh);
    sim_world_run_us(&w, 120000, 10);

    /* Inject a 5-hop Mesh TOPO_REPORT packet carrying permanent MAC48 into Primary G */
    on_mesh_pkt_t mp;
    memset(&mp, 0, sizeof(mp));
    mp.mesh_type   = (uint8_t)ON_MESH_TOPO_REPORT;
    mp.hop_count   = 5u;
    mp.ttl         = 2u;
    mp.src_id      = 0x305u;
    mp.dst_id      = ON_SHORT_ID_G_DEFAULT;
    mp.msg_id      = 77u;
    mp.payload_len = 6u;
    const uint8_t hop5_mac[6] = { 0xDEu, 0xADu, 0xBEu, 0xEFu, 0x05u, 0x06u };
    memcpy(mp.payload, hop5_mac, 6u);
    uint8_t mbuf[ON_MAX_PAYLOAD_LEN], frame[ON_MAX_FRAME_LEN];
    size_t mlen = 0u, flen = 0u;
    EXPECT(on_mesh_pkt_pack(&mp, mbuf, sizeof(mbuf), &mlen) == ON_OK, "Pack 5-hop Mesh TOPO_REPORT");
    on_mac_hdr_t mhdr;
    memset(&mhdr, 0, sizeof(mhdr));
    mhdr.version    = ON_PROTO_VERSION;
    mhdr.type       = (uint8_t)ON_FRAME_MESH;
    mhdr.src_id     = 0x005u; /* 1-hop Sub-G relay */
    mhdr.dst_id     = ON_SHORT_ID_G_DEFAULT;
    mhdr.sf_seq_low = (uint8_t)(w.nodes[0].current_sf_seq & 0x0Fu);
    mhdr.slot_idx   = 15u;
    EXPECT(on_frame_pack(&mhdr, mbuf, mlen, frame, sizeof(frame), &flen) == ON_OK, "Pack ON_FRAME_MESH");
    on_node_on_rx_raw(&w.nodes[0], frame, flen, sim_phys_to_local_ns(&w, 0u, w.phys_now_ns));

    char json[3072];
    on_node_format_cluster_json(&w.nodes[0], false, json, sizeof(json));
    EXPECT(strstr(json, "\"id\":\"0x305\"") != NULL &&
           strstr(json, "\"mac\":\"DE:AD:BE:EF:05:06\"") != NULL &&
           strstr(json, "\"hop\":5") != NULL,
           "Primary G records 5-hop Mesh node with full MAC48 & hop_depth=5");

    /* Remote 1-hop node T5 queries paginated TOPO_RESP and preserves hop_depth=5 (3-bit) & MAC48 */
    EXPECT(on_node_request_cluster_snapshot(&w.nodes[5]) == ON_OK, "T5 requests snapshot with 5-hop node");
    sim_world_run_us(&w, 160000, 10);
    on_node_format_cluster_json(&w.nodes[5], false, json, sizeof(json));
    EXPECT(w.nodes[5].cluster_snapshot_valid &&
           strstr(json, "\"id\":\"0x305\"") != NULL &&
           strstr(json, "\"hop\":5") != NULL &&
           strstr(json, "\"mac\":\"DE:AD:BE:EF:05:06\"") != NULL,
           "Paginated TOPO_RESP preserves 3-bit hop_depth=5 (no 2-bit truncation) & MAC48");

    /* 2. Verify multi-hop ghost-node lease expiry GC cleans up silent 5-hop node after timeout */
    w.nodes[0].peer_lease_timeout_sf = 100u; /* multi_hop_timeout = 200 SFs (800 ms) */
    sim_world_run_us(&w, 900000, 10); /* 225 SFs with no further packets from 0x305 */
    on_node_format_cluster_json(&w.nodes[0], false, json, sizeof(json));
    EXPECT(strstr(json, "\"id\":\"0x305\"") == NULL,
           "Silent 5-hop ghost node automatically purged by multi-hop cluster GC");

    /* 3. Verify constant-time admin token gating on RESET_WIFI & REBOOT commands */
    on_gw_ctx_t gw;
    on_gw_init(&gw, ON_GW_MODE_WIFI_AND_BLE, NULL);
    on_gw_set_wifi_creds(&gw, "OfficeWiFi", "Secret123", "");
    on_gw_set_admin_token(&gw, "Pin2026!");
    EXPECT(on_gw_handle_control_text(&gw, "RESET_WIFI", 10u, false) == ON_ERR_DECODE_FAIL &&
           on_gw_handle_control_text(&gw, "AUTH:WrongPin|RESET_WIFI", 24u, false) == ON_ERR_DECODE_FAIL &&
           !gw.cfg.wifi_reset_requested && strcmp(gw.cfg.wifi_ssid, "OfficeWiFi") == 0,
           "Unauthenticated or wrong-token RESET_WIFI rejected");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:Pin2026!|RESET_WIFI", 24u, false) == ON_OK &&
           gw.cfg.wifi_reset_requested && gw.cfg.wifi_ssid[0] == '\0',
           "Authenticated RESET_WIFI clears SSID and flags SoftAP reopen");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:Pin2026!|REBOOT", 20u, false) == ON_OK &&
           gw.cfg.reboot_requested,
           "Authenticated REBOOT sets graceful restart flag");
    printf("    -> 5-hop MAC48 + 3-bit hop roundtrip OK | Ghost-node GC purged 0x305 | Auth RESET_WIFI/REBOOT OK\n");
}

/* ========================================================================== */
/* Test 37: v6.7 Multi-GW MQTT LWT + Alias HA Discovery + NETKEY/ALIAS Parser */
/* ========================================================================== */
static void test_v67_mqtt_lwt_alias_and_netkey(void) {
    printf("[TEST 37] v6.7 MQTT LWT CONNECT + Alias/Availability HA Discovery + NETKEY/ALIAS Commands...\n");

    /* 1. Verify OASIS MQTT 3.1.1 CONNECT with LWT Will Flag (0x04) + Will Retain (0x20) + User/Pass */
    uint8_t cbuf[256];
    size_t clen = on_mqtt_encode_connect_lwt("opennear_gw_112233", "u1", "p1", 30u,
                                             "opennear/gw/112233/status", "offline", true,
                                             cbuf, sizeof(cbuf));
    EXPECT(clen > 40u && cbuf[0] == ON_MQTT_PKT_CONNECT, "MQTT LWT CONNECT packet encoded");
    /* Connect Flags byte is at offset 1 (type) + 1 (rem_len <128) + 6 ("MQTT") + 1 (level=4) = 9 */
    EXPECT((cbuf[9] & (0x80u | 0x40u | 0x20u | 0x04u | 0x02u)) == (0x80u | 0x40u | 0x20u | 0x04u | 0x02u),
           "MQTT CONNECT flags set CleanSession + Will + WillRetain + User + Pass");

    /* 2. Verify HA Discovery embeds custom alias, permanent MAC, and LWT availability topic */
    const uint8_t mac[6] = { 0xA4u, 0xCFu, 0x12u, 0xAAu, 0xBBu, 0xCCu };
    char topic[96], json[600];
    EXPECT(on_mqtt_format_ha_discovery_full(0x004u, mac, 1u, "T",
                                            "LivingRoom_Temp", "opennear/gw/112233/status",
                                            topic, sizeof(topic), json, sizeof(json)) &&
           strstr(json, "\"name\":\"LivingRoom_Temp\"") != NULL &&
           strstr(json, "\"uniq_id\":\"opennear_A4CF12AABBCC\"") != NULL &&
           strstr(json, "\"avty_t\":\"opennear/gw/112233/status\"") != NULL &&
           strstr(json, "\"pl_not_avail\":\"offline\"") != NULL,
           "HA Discovery integrates MAC uniq_id + friendly alias + LWT availability topic");

    /* 3. Verify NETKEY:<32hex> validation, clearing & ALIAS:<MAC>|<name> parsing with Admin Token */
    on_gw_ctx_t gw;
    on_gw_init(&gw, ON_GW_MODE_WIFI_AND_BLE, NULL);
    on_gw_set_admin_token(&gw, "Sec67!");

    EXPECT(on_gw_handle_control_text(&gw, "NETKEY:00112233445566778899AABBCCDDEEFF", 39u, false) == ON_ERR_DECODE_FAIL,
           "Unauthenticated NETKEY rejected when Admin Token active");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:Sec67!|NETKEY:BADHEX", 25u, false) == ON_ERR_INVALID_ARG,
           "Malformed NETKEY (<32 hex chars) rejected");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:Sec67!|NETKEY:00112233445566778899AABBCCDDEEFF", 51u, false) == ON_OK &&
           gw.cfg.net_key_custom && gw.cfg.net_key_updated &&
           gw.cfg.net_key[0] == 0x00u && gw.cfg.net_key[15] == 0xFFu,
           "Authenticated 32-hex NETKEY parsed into 16-byte AES-128 cluster key");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:Sec67!|NETKEY:", 19u, false) == ON_OK &&
           !gw.cfg.net_key_custom && gw.cfg.net_key_updated,
           "Empty NETKEY: resets cluster key to default");

    EXPECT(on_gw_handle_control_text(&gw, "AUTH:Sec67!|ALIAS:A4:CF:12:AA:BB:CC|Kitchen_Relay", 49u, false) == ON_OK &&
           gw.cfg.alias_updated && memcmp(gw.cfg.alias_mac, mac, 6u) == 0 &&
           strcmp(gw.cfg.alias_name, "Kitchen_Relay") == 0,
           "Authenticated ALIAS:<MAC>|<name> parses colon-formatted MAC48 and friendly name");
    printf("    -> MQTT LWT CONNECT=%uB | HA Alias+Avty OK | NETKEY(16B) & ALIAS parser OK\n", (unsigned)clen);
}

/* ========================================================================== */
/* Test 38: v6.8 Post-Failover Full MAC48 Recovery & MAC-Addressed MQTT Cmds  */
/* ========================================================================== */
static void test_v68_failover_mac_recovery_and_mac_mqtt_routing(void) {
    printf("[TEST 38] v6.8 Post-Failover Full MAC48 Recovery (1-Hop + 2-Hop) & MAC-Addressed MQTT Routing...\n");

    /* 1. Run 10-node cluster -> kill Primary G -> verify Backup_ID takes over AND recovers 1-hop & 2-hop MAC48s! */
    sim_world_t w;
    sim_world_init(&w, 0x68F02026ULL);
    sim_world_setup_10node_demo(&w, true);
    sim_world_run_us(&w, 200000, 10); /* 50 SFs steady-state */

    uint16_t bk_id = w.nodes[0].backup_id;
    int bk_idx = -1;
    for (uint32_t i = 1u; i < w.num_nodes; ++i) {
        if (w.nodes[i].short_id == bk_id) { bk_idx = (int)i; break; }
    }
    EXPECT(bk_idx > 0, "Backup_ID node identified before failover");

    /* Kill Primary G and advance 30 superframes so Backup_ID takes over and 1-hop + 2-hop nodes re-report MAC48s */
    w.alive[0] = false;
    sim_world_run_us(&w, 120000, 10);

    EXPECT(w.nodes[bk_idx].role == ON_ROLE_G, "Backup_ID seamlessly promoted to new Primary G");
    uint16_t resolved_t4 = on_node_find_short_id_by_mac(&w.nodes[bk_idx], w.nodes[4].mac48);
    uint16_t resolved_t8 = on_node_find_short_id_by_mac(&w.nodes[bk_idx], w.nodes[8].mac48);
    EXPECT(resolved_t4 == w.nodes[4].short_id,
           "New Primary G recovers 1-hop T4 permanent MAC48 -> Short-ID mapping after failover");
    EXPECT(resolved_t8 == w.nodes[8].short_id,
           "New Primary G recovers wall-blocked 2-hop T8 permanent MAC48 -> Short-ID mapping after failover");
    EXPECT(w.tdma_subg_collisions == 0u && w.cra_collisions == 0u,
           "Zero RF collisions during post-failover MAC48 recovery");

    /* 2. Verify MAC-addressed MQTT command topic parsing (compact 12-hex & colon-separated) */
    uint16_t tid = 0u;
    uint8_t pmac[6] = {0};
    bool has_mac = false, is_ota = false;
    const uint8_t expect_mac[6] = { 0xA4u, 0xCFu, 0x12u, 0xAAu, 0xBBu, 0xCCu };
    EXPECT(on_mqtt_parse_cmd_topic_ex("opennear/cmd/A4CF12AABBCC", 25u, &tid, pmac, &has_mac, &is_ota) &&
           has_mac && !is_ota && memcmp(pmac, expect_mac, 6u) == 0,
           "MQTT parser resolves compact 12-hex MAC command topic");
    EXPECT(on_mqtt_parse_cmd_topic_ex("opennear/cmd/A4:CF:12:AA:BB:CC", 30u, &tid, pmac, &has_mac, &is_ota) &&
           has_mac && !is_ota && memcmp(pmac, expect_mac, 6u) == 0,
           "MQTT parser resolves colon-formatted MAC command topic");

    /* 3. Verify MAC-anchored HA Discovery & State topic use permanent MACHEX */
    char topic[96], json[600];
    const uint8_t sample_pay[2] = { 0x12u, 0x34u };
    EXPECT(on_mqtt_format_ha_discovery_full(0x007u, expect_mac, 2u, "T",
                                            "Porch_Sensor", "opennear/gw/AABBCC/status",
                                            topic, sizeof(topic), json, sizeof(json)) &&
           strstr(json, "\"stat_t\":\"opennear/node/A4CF12AABBCC/state\"") != NULL &&
           strstr(json, "\"cmd_t\":\"opennear/cmd/A4CF12AABBCC\"") != NULL,
           "HA Discovery binds stat_t & cmd_t directly to permanent MAC48");
    EXPECT(on_mqtt_format_node_state_mac(0x007u, expect_mac, 0x005u, 2u, 15,
                                         sample_pay, 2u, topic, sizeof(topic), json, sizeof(json)) &&
           strcmp(topic, "opennear/node/A4CF12AABBCC/state") == 0,
           "Node state publishes to permanent MAC-anchored MQTT topic");
    printf("    -> Post-failover G=0x%03X recovered T4=0x%03X & 2-hop T8=0x%03X (0 collisions) | MAC MQTT cmd/state OK\n",
           (unsigned)w.nodes[bk_idx].short_id, (unsigned)resolved_t4, (unsigned)resolved_t8);
}

/* 39. Test v7.0 Zero-Preconfig Dynamic Tagging, Multi-Anchor RSSI Reporting & 2D Indoor Positioning */
typedef struct {
    uint32_t count;
    uint16_t last_anchor_id;
    uint16_t last_target_id;
    int8_t   last_rssi_dbm;
    bool     matched_expected;
    uint16_t expect_anchor_id;
    uint16_t expect_target_id;
    int8_t   expect_rssi_dbm;
} test_v70_rssi_cb_t;

static void test_v70_rssi_cb(void *ctx, uint16_t anchor_id, uint16_t target_id, int8_t rssi_dbm) {
    test_v70_rssi_cb_t *st = (test_v70_rssi_cb_t *)ctx;
    st->count++;
    st->last_anchor_id = anchor_id;
    st->last_target_id = target_id;
    st->last_rssi_dbm  = rssi_dbm;
    if (anchor_id == st->expect_anchor_id &&
        target_id == st->expect_target_id &&
        rssi_dbm  == st->expect_rssi_dbm) {
        st->matched_expected = true;
    }
}

static void test_v70_indoor_locate_rssi_and_tags(void) {
    printf("[TEST 39] v7.0 Zero-Preconfig Tags, Anchor RSSI Overhearing & 2D Indoor Positioning...\n");

    /* Part A: 4-Anchor 2D trilateration + log-distance path-loss solver */
    on_loc_ctx_t loc;
    on_loc_init(&loc);

    const uint8_t a_mac[4][6] = {
        { 0xAA, 0x10, 0x00, 0x00, 0x00, 0x01 },
        { 0xAA, 0x10, 0x00, 0x00, 0x00, 0x02 },
        { 0xAA, 0x10, 0x00, 0x00, 0x00, 0x03 },
        { 0xAA, 0x10, 0x00, 0x00, 0x00, 0x04 }
    };
    const int32_t ax[4] = { 0, 600, 600, 0 };
    const int32_t ay[4] = { 0, 0,   400, 400 };
    const char   *ar[4] = { "Living", "Bedroom", "Study", "Kitchen" };
    for (int i = 0; i < 4; ++i) {
        EXPECT(on_loc_set_anchor(&loc, a_mac[i], (uint16_t)(i + 1), ar[i], ax[i], ay[i]) == ON_OK,
               "Anchor registered in indoor positioning engine");
    }

    const uint8_t tgt_mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x09 };
    const double true_x = 200.0, true_y = 150.0; /* Nearest to Living (0,0), d=250cm */
    for (uint32_t step = 0u; step < 5u; ++step) {
        uint32_t now_ms = 1000u + step * 500u;
        for (int i = 0; i < 4; ++i) {
            double dx = (true_x - (double)ax[i]) / 100.0;
            double dy = (true_y - (double)ay[i]) / 100.0;
            double dm = sqrt(dx * dx + dy * dy);
            if (dm < 0.25) dm = 0.25;
            double rssi_f = -45.0 - 10.0 * 2.5 * log10(dm);
            int8_t rssi = (int8_t)llround(rssi_f);
            EXPECT(on_loc_observe_rssi(&loc, a_mac[i], tgt_mac, 0x009u, rssi, now_ms) == ON_OK,
                   "on_loc_observe_rssi accepts anchor reading");
        }
    }

    on_loc_estimate_t est, est_a;
    EXPECT(on_loc_solve(&loc, tgt_mac, 3500u, &est) && est.valid, "2D indoor solve succeeds");
    est_a = est;
    EXPECT(est.anchor_count == 4u, "2D solve uses all 4 active anchors");
    EXPECT(strcmp(est.room, "Living") == 0, "Room classifier picks nearest anchor Living");
    int32_t err_x = est.x_cm - (int32_t)true_x;
    int32_t err_y = est.y_cm - (int32_t)true_y;
    if (err_x < 0) err_x = -err_x;
    if (err_y < 0) err_y = -err_y;
    EXPECT(err_x <= 35 && err_y <= 35, "Gauss-Newton 2D trilateration converges within 35 cm");

    char loc_json[1024];
    size_t jlen = on_loc_format_json(&loc, 3500u, loc_json, sizeof(loc_json));
    EXPECT(jlen > 40u &&
           strstr(loc_json, "\"anchors\":[") != NULL &&
           strstr(loc_json, "\"targets\":[") != NULL &&
           strstr(loc_json, "\"Living\"") != NULL &&
           strstr(loc_json, "DE:AD:BE:EF:00:09") != NULL,
           "on_loc_format_json emits valid anchors & solved target JSON");

    /* Part B: +4 dB hysteresis prevents room ping-pong, +6 dB switches cleanly */
    on_loc_ctx_t hloc;
    on_loc_init(&hloc);
    EXPECT(on_loc_set_anchor(&hloc, a_mac[0], 1u, "Living", 0, 0) == ON_OK, "Hysteresis anchor 0");
    EXPECT(on_loc_set_anchor(&hloc, a_mac[1], 2u, "Bedroom", 500, 0) == ON_OK, "Hysteresis anchor 1");
    for (uint32_t s = 0u; s < 4u; ++s) {
        on_loc_observe_rssi(&hloc, a_mac[0], tgt_mac, 0x009u, -50, 1000u + s * 200u);
        on_loc_observe_rssi(&hloc, a_mac[1], tgt_mac, 0x009u, -56, 1000u + s * 200u);
    }
    EXPECT(on_loc_solve(&hloc, tgt_mac, 2000u, &est) && strcmp(est.room, "Living") == 0,
           "Target initializes in Living");

    /* Bedroom rises to -48 dBm (+2 dB over Living -50 dBm): below +4 dB hysteresis -> MUST stay Living */
    for (uint32_t s = 0u; s < 5u; ++s) {
        on_loc_observe_rssi(&hloc, a_mac[0], tgt_mac, 0x009u, -50, 2200u + s * 200u);
        on_loc_observe_rssi(&hloc, a_mac[1], tgt_mac, 0x009u, -48, 2200u + s * 200u);
    }
    EXPECT(on_loc_solve(&hloc, tgt_mac, 3200u, &est) && strcmp(est.room, "Living") == 0,
           "+4 dB hysteresis prevents room ping-pong on small +2 dB fluctuation");

    /* Bedroom rises to -43 dBm (+7 dB over Living -50 dBm): exceeds +4 dB -> switches to Bedroom */
    for (uint32_t s = 0u; s < 6u; ++s) {
        on_loc_observe_rssi(&hloc, a_mac[0], tgt_mac, 0x009u, -50, 3500u + s * 200u);
        on_loc_observe_rssi(&hloc, a_mac[1], tgt_mac, 0x009u, -43, 3500u + s * 200u);
    }
    EXPECT(on_loc_solve(&hloc, tgt_mac, 4800u, &est) && strcmp(est.room, "Bedroom") == 0,
           "Target transitions cleanly to Bedroom when RSSI exceeds +4 dB hysteresis");

    /* Removing anchor with empty room name succeeds */
    EXPECT(on_loc_set_anchor(&hloc, a_mac[1], 2u, "", 0, 0) == ON_OK,
           "Empty room name dynamically removes anchor");
    EXPECT(!hloc.anchors[1].active, "Removed anchor slot marked inactive");

    /* Part C: Live 10-node cluster promiscuous RSSI overhearing + zero-collision ON_GW_MSG_RSSI_REPORT */
    sim_world_t w;
    sim_world_init(&w, 0x70A12026ULL);
    sim_world_setup_10node_demo(&w, true);
    sim_world_run_us(&w, 160000, 10); /* 40 SFs steady-state */

    on_node_t *g_node  = &w.nodes[0];
    on_node_t *a_node  = &w.nodes[3]; /* 1-hop node acting as fixed Anchor */
    on_node_t *t_node  = &w.nodes[4]; /* 1-hop target node overheard by Anchor */
    EXPECT(a_node->role == ON_ROLE_T && t_node->role == ON_ROLE_T,
           "Anchor T3 and target T4 joined 1-hop cluster");

    test_v70_rssi_cb_t cb_st;
    memset(&cb_st, 0, sizeof(cb_st));
    cb_st.expect_anchor_id = a_node->short_id;
    cb_st.expect_target_id = t_node->short_id;
    cb_st.expect_rssi_dbm  = -58;
    on_node_set_rssi_observer(g_node, test_v70_rssi_cb, &cb_st);
    on_node_enable_rssi_anchor_report(a_node, true);

    /* Inject a promiscuously overheard unicast frame (src=T4, dst=G) with RSSI=-58 dBm into Anchor T3 */
    on_mac_hdr_t oh;
    memset(&oh, 0, sizeof(oh));
    oh.version    = ON_PROTO_VERSION;
    oh.type       = (uint8_t)ON_FRAME_TDMA_DATA;
    oh.src_id     = t_node->short_id;
    oh.dst_id     = g_node->short_id; /* Unicast to G (NOT addressed to T3!) */
    oh.sf_seq_low = (uint8_t)(a_node->current_sf_seq & 0x0Fu);
    oh.slot_idx   = t_node->my_slot;
    uint8_t dummy_pay[4] = { 0x01u, 0x02u, 0x03u, 0x04u };
    uint8_t raw_overheard[ON_MAX_FRAME_LEN];
    size_t  raw_len = 0u;
    EXPECT(on_frame_pack(&oh, dummy_pay, sizeof(dummy_pay), raw_overheard, sizeof(raw_overheard), &raw_len) == ON_OK,
           "Packed overheard peer frame for RSSI injection");

    int64_t a_local_ns = sim_phys_to_local_ns(&w, 3u, w.phys_now_ns);
    on_node_on_rx_raw_rssi(a_node, raw_overheard, raw_len, a_local_ns, -58);
    EXPECT(a_node->rssi_obs_count >= 1u, "Anchor records overheard peer RSSI before unicast filter");

    /* Run 75 SFs (300 ms) so Anchor T3's 64-SF D-TDMA RSSI_REPORT slot fires with zero collisions */
    sim_world_run_us(&w, 300000, 10);
    EXPECT(w.tdma_subg_collisions == 0u && w.cra_collisions == 0u,
           "Zero-collision ON_GW_MSG_RSSI_REPORT delivery");
    EXPECT(cb_st.matched_expected, "Primary G received Anchor T3 overheard RSSI (-58 dBm) for T4");

    uint8_t resolved_mac[6];
    EXPECT(on_node_find_mac_by_short_id(g_node, t_node->short_id, resolved_mac) &&
           memcmp(resolved_mac, t_node->mac48, 6u) == 0,
           "on_node_find_mac_by_short_id resolves permanent hardware MAC48");

    /* Part D: Zero-preconfig TAG:, CMD_TAG:, ANCHOR: control commands + admin_token gating */
    on_gw_ctx_t gw;
    on_gw_init(&gw, ON_GW_MODE_WIFI_AND_BLE, NULL);
    on_gw_set_admin_token(&gw, "v7secret");

    const char *c1 = "TAG:A4CF12AABBCC|living,light";
    const char *c2 = "CMD_TAG:living|RELAY_ON";
    const char *c3 = "ANCHOR:A4CF12AABBCC|Living|250|120";
    EXPECT(on_gw_handle_control_text(&gw, c1, strlen(c1), false) == ON_ERR_DECODE_FAIL,
           "Unauthenticated TAG rejected when admin_token is set");
    EXPECT(on_gw_handle_control_text(&gw, c2, strlen(c2), false) == ON_ERR_DECODE_FAIL,
           "Unauthenticated CMD_TAG rejected when admin_token is set");
    EXPECT(on_gw_handle_control_text(&gw, c3, strlen(c3), false) == ON_ERR_DECODE_FAIL,
           "Unauthenticated ANCHOR rejected when admin_token is set");

    const char *a1 = "AUTH:v7secret|TAG:A4CF12AABBCC|living,light";
    EXPECT(on_gw_handle_control_text(&gw, a1, strlen(a1), false) == ON_OK &&
           gw.cfg.tag_updated &&
           gw.cfg.tag_mac[0] == 0xA4u && gw.cfg.tag_mac[5] == 0xCCu &&
           strcmp(gw.cfg.tag_list, "living,light") == 0,
           "Authenticated TAG:<MAC>|<tags> parsed cleanly");

    const char *a2 = "AUTH:v7secret|CMD_TAG:living|RELAY_ON";
    EXPECT(on_gw_handle_control_text(&gw, a2, strlen(a2), false) == ON_OK &&
           gw.cfg.cmd_tag_requested &&
           strcmp(gw.cfg.cmd_tag, "living") == 0 &&
           gw.cfg.cmd_tag_payload_len == 8u &&
           memcmp(gw.cfg.cmd_tag_payload, "RELAY_ON", 8u) == 0,
           "Authenticated CMD_TAG:<tag>|<payload> parsed cleanly");

    const char *a3 = "AUTH:v7secret|ANCHOR:A4CF12AABBCC|Living|250|120";
    EXPECT(on_gw_handle_control_text(&gw, a3, strlen(a3), false) == ON_OK &&
           gw.cfg.anchor_updated &&
           strcmp(gw.cfg.anchor_room, "Living") == 0 &&
           gw.cfg.anchor_x_cm == 250 && gw.cfg.anchor_y_cm == 120,
           "Authenticated ANCHOR:<MAC>|<room>|<x>|<y> parsed cleanly");

    gw.cfg.anchor_updated = false;
    const char *a4 = "AUTH:v7secret|ANCHOR:A4CF12AABBCC||0|0";
    EXPECT(on_gw_handle_control_text(&gw, a4, strlen(a4), false) == ON_OK &&
           gw.cfg.anchor_updated &&
           gw.cfg.anchor_room[0] == '\0',
           "Authenticated ANCHOR with empty room parses removal command");

    printf("    -> 4-Anchor 2D est=(%d,%d)cm (err %d,%d cm) | Hysteresis OK | Live RSSI_REPORT OK | TAG/CMD_TAG/ANCHOR OK\n",
           (int)est_a.x_cm, (int)est_a.y_cm, (int)err_x, (int)err_y);
}

/* 40. Test v7.1 Standard BLE iBeacon & Eddystone Parser, Calibrated 2D Positioning & Multi-Anchor Relay */
typedef struct {
    uint32_t count;
    bool     matched;
    uint16_t expect_anchor;
    uint8_t  expect_mac[6];
    uint8_t  expect_src;
    int8_t   expect_tx1m;
    int8_t   expect_rssi;
} test_v71_ble_cb_t;

static void test_v71_ble_cb(void *ctx, uint16_t anchor_id, const uint8_t tag_mac48[6],
                            uint8_t source, int8_t tx_power_1m_dbm, int8_t rssi_dbm) {
    test_v71_ble_cb_t *st = (test_v71_ble_cb_t *)ctx;
    st->count++;
    if (anchor_id == st->expect_anchor &&
        memcmp(tag_mac48, st->expect_mac, 6u) == 0 &&
        source == st->expect_src &&
        tx_power_1m_dbm == st->expect_tx1m &&
        rssi_dbm == st->expect_rssi) {
        st->matched = true;
    }
}

static void test_v71_ble_ibeacon_eddystone_and_relay(void) {
    printf("[TEST 40] v7.1 Standard BLE iBeacon/Eddystone Parser, Calibrated 2D Locate & Multi-Anchor Relay...\n");

    /* Part A: Apple iBeacon standard 30-byte BLE Advertising Data */
    const uint8_t ib_ad[30] = {
        0x02, 0x01, 0x06,
        0x1A, 0xFF, 0x4C, 0x00, 0x02, 0x15,
        0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2,
        0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0,
        0x03, 0xE9, /* Major = 1001 */
        0x00, 0x2A, /* Minor = 42 */
        0xC5        /* txPower@1m = -59 dBm */
    };
    on_ble_beacon_t ib;
    EXPECT(on_ble_beacon_parse(ib_ad, sizeof(ib_ad), &ib) && ib.valid,
           "Apple iBeacon AD packet parsed cleanly");
    EXPECT(ib.source == ON_LOC_SRC_IBEACON && ib.major == 1001u && ib.minor == 42u &&
           ib.tx_power_1m_dbm == -59 && strcmp(ib.label, "iB:1001/42") == 0,
           "iBeacon UUID, Major/Minor, calibrated txPower@1m (-59 dBm), and label match");

    const uint8_t rand_mac_a[6] = { 0x41, 0x22, 0x33, 0x44, 0x55, 0x66 };
    const uint8_t rand_mac_b[6] = { 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC };
    uint8_t ib_mac_a[6], ib_mac_b[6];
    on_ble_beacon_derive_mac(&ib, rand_mac_a, ib_mac_a);
    on_ble_beacon_derive_mac(&ib, rand_mac_b, ib_mac_b);
    EXPECT(memcmp(ib_mac_a, ib_mac_b, 6u) == 0 && ib_mac_a[0] == 0x02u &&
           ib_mac_a[2] == 0x03u && ib_mac_a[3] == 0xE9u &&
           ib_mac_a[4] == 0x00u && ib_mac_a[5] == 0x2Au,
           "iBeacon deterministic MAC48 is immune to iOS/Android BLE random MAC rotation");

    /* Part B: Google Eddystone-UID, Eddystone-TLM & noise rejection */
    const uint8_t ed_uid_ad[28] = {
        0x03, 0x03, 0xAA, 0xFE,
        0x17, 0x16, 0xAA, 0xFE,
        0x00, /* Frame Type: Eddystone-UID */
        0xEE, /* txPower@0m = -18 dBm -> txPower@1m = -18 - 41 = -59 dBm */
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, /* 10B Namespace */
        0x01, 0x02, 0xAA, 0xBB, 0xCC, 0xDD,                         /* 6B Instance */
        0x00, 0x00
    };
    on_ble_beacon_t ed;
    EXPECT(on_ble_beacon_parse(ed_uid_ad, sizeof(ed_uid_ad), &ed) && ed.valid &&
           ed.source == ON_LOC_SRC_EDDYSTONE && ed.eddy_frame_type == 0x00u &&
           ed.tx_power_1m_dbm == -59 && strcmp(ed.label, "Ed:AABBCCDD") == 0,
           "Google Eddystone-UID parsed with -41 dB 0m->1m txPower conversion");
    uint8_t ed_mac[6];
    on_ble_beacon_derive_mac(&ed, rand_mac_a, ed_mac);
    EXPECT(ed_mac[0] == 0x06u && ed_mac[2] == 0xAAu && ed_mac[5] == 0xDDu,
           "Eddystone-UID derives deterministic stable 6-byte Trackable ID");

    const uint8_t ed_tlm_ad[18] = {
        0x11, 0x16, 0xAA, 0xFE,
        0x20, 0x00, /* Eddystone-TLM v0 */
        0x0B, 0xEA, /* vbatt = 3050 mV */
        0x18, 0x80, /* temp = 24.50 C */
        0x00, 0x00, 0x00, 0x10,
        0x00, 0x00, 0x00, 0x20
    };
    on_ble_beacon_t tlm;
    EXPECT(on_ble_beacon_parse(ed_tlm_ad, sizeof(ed_tlm_ad), &tlm) && tlm.valid &&
           tlm.eddy_frame_type == 0x20u && tlm.tlm_batt_mv == 3050u && tlm.tlm_temp_c_x100 == 2450,
           "Google Eddystone-TLM battery voltage (3050 mV) & temp (24.50 C) parsed");

    const uint8_t noise_ad[10] = { 0x02, 0x01, 0x06, 0x06, 0x09, 'T', 'V', '1', '2', '3' };
    on_ble_beacon_t nope;
    EXPECT(!on_ble_beacon_parse(noise_ad, sizeof(noise_ad), &nope),
           "Non-beacon generic BLE device advertisements ignored cleanly");

    /* Part C: 4-Anchor 2D Trilateration using Beacon's calibrated txPower@1m (-59 dBm vs anchor -45 dBm) */
    on_loc_ctx_t loc;
    on_loc_init(&loc);
    const uint8_t a_mac[4][6] = {
        { 0xAA, 0x71, 0x00, 0x00, 0x00, 0x01 },
        { 0xAA, 0x71, 0x00, 0x00, 0x00, 0x02 },
        { 0xAA, 0x71, 0x00, 0x00, 0x00, 0x03 },
        { 0xAA, 0x71, 0x00, 0x00, 0x00, 0x04 }
    };
    const int32_t ax[4] = { 0, 600, 600, 0 };
    const int32_t ay[4] = { 0, 0,   400, 400 };
    const char   *ar[4] = { "Living", "Bedroom", "Study", "Kitchen" };
    for (int i = 0; i < 4; ++i) {
        on_loc_set_anchor(&loc, a_mac[i], (uint16_t)(i + 1), ar[i], ax[i], ay[i]);
    }
    const double true_x = 380.0, true_y = 260.0; /* Nearest to Study (600,400) */
    for (uint32_t s = 0u; s < 5u; ++s) {
        uint32_t now_ms = 2000u + s * 500u;
        for (int i = 0; i < 4; ++i) {
            double dx = (true_x - (double)ax[i]) / 100.0;
            double dy = (true_y - (double)ay[i]) / 100.0;
            double dm = sqrt(dx * dx + dy * dy);
            if (dm < 0.25) dm = 0.25;
            /* Synthesize RSSI using iBeacon's -59 dBm calibrated TX power! */
            int8_t rssi = (int8_t)llround(-59.0 - 10.0 * 2.5 * log10(dm));
            EXPECT(on_loc_observe_beacon(&loc, a_mac[i], ib_mac_a, ON_LOC_SRC_IBEACON,
                                         ib.tx_power_1m_dbm, rssi, ib.label, now_ms) == ON_OK,
                   "on_loc_observe_beacon records calibrated iBeacon observation");
        }
    }
    on_loc_estimate_t est;
    EXPECT(on_loc_solve(&loc, ib_mac_a, 4500u, &est) && est.valid &&
           est.source == ON_LOC_SRC_IBEACON && est.tx_power_1m_dbm == -59 &&
           strcmp(est.room, "Study") == 0,
           "2D solver uses iBeacon calibrated txPower@1m (-59 dBm) and classifies room Study");
    int32_t err_x = est.x_cm - (int32_t)true_x;
    int32_t err_y = est.y_cm - (int32_t)true_y;
    if (err_x < 0) err_x = -err_x;
    if (err_y < 0) err_y = -err_y;
    EXPECT(err_x <= 35 && err_y <= 35, "Calibrated iBeacon 2D trilateration converges within 35 cm");

    char jbuf[1024];
    size_t jlen = on_loc_format_json(&loc, 4500u, jbuf, sizeof(jbuf));
    EXPECT(jlen > 40u &&
           strstr(jbuf, "\"src\":\"ibeacon\"") != NULL &&
           strstr(jbuf, "\"label\":\"iB:1001/42\"") != NULL &&
           strstr(jbuf, "\"tx1m\":-59") != NULL,
           "on_loc_format_json emits src, label, and calibrated tx1m for BLE beacons");

    /* Part D: Live 10-Node Satellite Anchor -> Primary G zero-collision ON_GW_MSG_BLE_BEACON_REPORT */
    sim_world_t w;
    sim_world_init(&w, 0x71B12026ULL);
    sim_world_setup_10node_demo(&w, true);
    sim_world_run_us(&w, 160000, 10);

    on_node_t *g_node = &w.nodes[0];
    on_node_t *a_node = &w.nodes[3]; /* 1-hop satellite Anchor scanning BLE beacons */
    test_v71_ble_cb_t bcb;
    memset(&bcb, 0, sizeof(bcb));
    bcb.expect_anchor = a_node->short_id;
    memcpy(bcb.expect_mac, ib_mac_a, 6u);
    bcb.expect_src  = (uint8_t)ON_LOC_SRC_IBEACON;
    bcb.expect_tx1m = -59;
    bcb.expect_rssi = -64;
    on_node_set_ble_beacon_observer(g_node, test_v71_ble_cb, &bcb);
    on_node_enable_rssi_anchor_report(a_node, true);

    on_node_report_ble_beacon(a_node, ib_mac_a, (uint8_t)ON_LOC_SRC_IBEACON, -59, -64);
    EXPECT(a_node->ble_obs_count == 1u, "Satellite anchor buffers passive BLE beacon observation");

    sim_world_run_us(&w, 300000, 10); /* 75 SFs so private D-TDMA BLE_BEACON_REPORT fires */
    EXPECT(w.tdma_subg_collisions == 0u && w.cra_collisions == 0u,
           "Zero RF collisions during satellite ON_GW_MSG_BLE_BEACON_REPORT");
    EXPECT(bcb.matched, "Primary G receives satellite anchor iBeacon observation over D-TDMA");

    /* Part E: BLESCAN:<0|1> control command + Admin Token gating */
    on_gw_ctx_t gw;
    on_gw_init(&gw, ON_GW_MODE_WIFI_AND_BLE, NULL);
    on_gw_set_admin_token(&gw, "ble71");
    EXPECT(on_gw_handle_control_text(&gw, "BLESCAN:0", 9u, false) == ON_ERR_DECODE_FAIL,
           "Unauthenticated BLESCAN rejected when admin_token active");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:ble71|BLESCAN:9", 20u, false) == ON_ERR_INVALID_ARG,
           "Invalid BLESCAN value rejected");
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:ble71|BLESCAN:0", 20u, false) == ON_OK &&
           gw.cfg.ble_scan_updated && !gw.cfg.ble_scan_enabled,
           "Authenticated BLESCAN:0 disables passive scanner");
    gw.cfg.ble_scan_updated = false;
    EXPECT(on_gw_handle_control_text(&gw, "AUTH:ble71|BLESCAN:1", 20u, false) == ON_OK &&
           gw.cfg.ble_scan_updated && gw.cfg.ble_scan_enabled,
           "Authenticated BLESCAN:1 enables passive scanner");

    printf("    -> iBeacon(tx1m=-59) 2D est=(%d,%d)cm (err %d,%d cm) | Eddystone UID/TLM OK | Satellite Relay OK (0 coll)\n",
           (int)est.x_cm, (int)est.y_cm, (int)err_x, (int)err_y);
}

/* 41. Test v7.1 Deep Audit: 2-Hop Wall-Blocked Edge Anchor Relay, Stale Target GC & Eddystone-URL */
static void test_v71_deep_audit_2hop_anchor_and_stale_gc(void) {
    printf("[TEST 41] v7.1 Deep Audit: 2-Hop Wall-Blocked Edge Anchor Relay, Stale Target GC & Eddystone-URL...\n");

    /* Part A: Wall-blocked 2-hop Edge Node T8 acting as positioning Anchor via Sub-G relay */
    sim_world_t w;
    sim_world_init(&w, 0x71D42026ULL);
    sim_world_setup_10node_demo(&w, true);
    sim_world_run_us(&w, 200000, 10); /* 50 SFs so 2-hop T8 & T9 join and register MAC48s */

    on_node_t *g_node  = &w.nodes[0];
    on_node_t *t8_node = &w.nodes[8]; /* 2-hop wall-blocked edge node */
    on_node_t *t9_node = &w.nodes[9]; /* 2-hop sibling heard by T8 */
    EXPECT(t8_node->role == ON_ROLE_T && t8_node->is_sub_node && t8_node->short_id >= 0x100u,
           "2-hop edge node T8 joined Sub-G sub-superframe");

    const uint8_t ib_mac[6] = { 0x02, 0xA9, 0x03, 0xE9, 0x00, 0x55 };
    test_v71_ble_cb_t bcb;
    memset(&bcb, 0, sizeof(bcb));
    bcb.expect_anchor = t8_node->short_id; /* Must be attributed to 2-hop T8 (0x100), NOT Sub-G! */
    memcpy(bcb.expect_mac, ib_mac, 6u);
    bcb.expect_src  = (uint8_t)ON_LOC_SRC_IBEACON;
    bcb.expect_tx1m = -59;
    bcb.expect_rssi = -67;
    on_node_set_ble_beacon_observer(g_node, test_v71_ble_cb, &bcb);
    on_node_enable_rssi_anchor_report(t8_node, true);

    on_node_report_ble_beacon(t8_node, ib_mac, (uint8_t)ON_LOC_SRC_IBEACON, -59, -67);
    EXPECT(t8_node->ble_obs_count == 1u, "2-hop edge anchor T8 buffers BLE beacon observation");

    /* Run 110 SFs so 2-hop T8 sends in Sub-TDMA slot and Sub-G relays to Primary G with 0 collisions */
    sim_world_run_us(&w, 440000, 10);
    EXPECT(w.tdma_subg_collisions == 0u && w.cra_collisions == 0u,
           "Zero RF collisions during 2-hop T8 -> Sub-G -> Primary G BLE_BEACON_REPORT relay");
    EXPECT(bcb.matched,
           "Primary G attributes 2-hop relayed BLE_BEACON_REPORT to true 2-hop Anchor T8 (origin_id)");

    /* Now verify 2-hop Edge Anchor T8 overhearing sibling T9's RSSI and relaying via Sub-G */
    test_v70_rssi_cb_t rcb;
    memset(&rcb, 0, sizeof(rcb));
    rcb.expect_anchor_id = t8_node->short_id;
    rcb.expect_target_id = t9_node->short_id;
    rcb.expect_rssi_dbm  = -61;
    on_node_set_rssi_observer(g_node, test_v70_rssi_cb, &rcb);

    on_mac_hdr_t oh;
    memset(&oh, 0, sizeof(oh));
    oh.version    = ON_PROTO_VERSION;
    oh.type       = (uint8_t)ON_FRAME_TDMA_DATA;
    oh.is_sub_g   = true;
    oh.src_id     = t9_node->short_id;
    oh.dst_id     = t8_node->parent_id;
    oh.sf_seq_low = (uint8_t)(t8_node->current_sf_seq & 0x0Fu);
    oh.slot_idx   = t9_node->my_slot;
    uint8_t dpay[2] = { 0xE9u, 0x01u };
    uint8_t raw_oh[ON_MAX_FRAME_LEN];
    size_t  raw_len = 0u;
    EXPECT(on_frame_pack(&oh, dpay, sizeof(dpay), raw_oh, sizeof(raw_oh), &raw_len) == ON_OK,
           "Packed 2-hop sibling frame for RSSI overhearing");
    on_node_on_rx_raw_rssi(t8_node, raw_oh, raw_len,
                           sim_phys_to_local_ns(&w, 8u, w.phys_now_ns), -61);
    sim_world_run_us(&w, 440000, 10);
    EXPECT(w.tdma_subg_collisions == 0u && w.cra_collisions == 0u,
           "Zero RF collisions during 2-hop T8 -> Sub-G -> Primary G RSSI_REPORT relay");
    EXPECT(rcb.matched_expected,
           "Primary G attributes 2-hop relayed RSSI_REPORT to true 2-hop Anchor T8");

    /* Part B: Stale Target Auto-GC & Out-of-Order Timestamp Immunity */
    on_loc_ctx_t loc;
    on_loc_init(&loc);
    const uint8_t amac[6] = { 0xAA, 0x88, 0x00, 0x00, 0x00, 0x01 };
    const uint8_t tmac[6] = { 0x02, 0xBB, 0x00, 0x01, 0x00, 0x02 };
    on_loc_set_anchor(&loc, amac, 1u, "Garage", 0, 0);
    on_loc_observe_beacon(&loc, amac, tmac, ON_LOC_SRC_IBEACON, -59, -62, "iB:1/2", 5000u);

    on_loc_estimate_t est;
    /* Out-of-order earlier timestamp (4000 < 5000) must not underflow unsigned subtraction */
    EXPECT(on_loc_solve(&loc, tmac, 4000u, &est) && est.valid,
           "on_loc_solve handles out-of-order timestamp without unsigned underflow");
    /* After > 2 * ON_LOC_OBS_STALE_MS (50000 ms), stale target slot is automatically garbage-collected */
    EXPECT(!on_loc_solve(&loc, tmac, 50000u, &est) && !loc.targets[0].active,
           "Stale target (>2x stale window) automatically garbage-collected from target table");

    /* Part C: Eddystone-URL (frame_type = 0x10) parsing & fallback BLE MAC derivation */
    const uint8_t ed_url_ad[14] = {
        0x0D, 0x16, 0xAA, 0xFE,
        0x10,       /* Eddystone-URL */
        0xEC,       /* txPower@0m = -20 dBm -> txPower@1m = -61 dBm */
        0x03,       /* https:// */
        'o', 'p', 'e', 'n', '.', 'i', 'o'
    };
    on_ble_beacon_t url_b;
    EXPECT(on_ble_beacon_parse(ed_url_ad, sizeof(ed_url_ad), &url_b) && url_b.valid &&
           url_b.source == ON_LOC_SRC_EDDYSTONE && url_b.eddy_frame_type == 0x10u &&
           url_b.tx_power_1m_dbm == -61 && strcmp(url_b.label, "EdURL:03") == 0,
           "Google Eddystone-URL frame parsed with -41 dB 0m->1m calibration");
    const uint8_t fb_mac[6] = { 0xC4, 0x55, 0x66, 0x77, 0x88, 0x99 };
    uint8_t url_mac[6];
    on_ble_beacon_derive_mac(&url_b, fb_mac, url_mac);
    EXPECT(memcmp(url_mac, fb_mac, 6u) == 0, "Eddystone-URL preserves fallback BLE MAC48");

    printf("    -> 2-Hop Edge Anchor T8(0x%03X) BLE+RSSI Relay OK (0 coll) | Stale Target GC OK | Eddystone-URL OK\n",
           (unsigned)t8_node->short_id);
}

int main(void) {
    printf("============================================================\n");
    printf("       OpenNear v7.1 Core Protocol Automated Test Suite     \n");
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
    test_v51_any_node_cluster_topology_and_data_query();
    test_v60_leaderless_cold_boot_auto_election();
    test_v60_6hop_mesh_routing_and_256node_epoch_tdma();
    test_v60_mqtt311_and_home_assistant_discovery();
    test_v61_audit_regressions();
    test_v62_deep_hardening_regressions();
    test_independent_act_clock();
    test_v64_optional_mqtts_tls_config();
    test_v65_cluster_mac48_and_ha_identity();
    test_v66_multihop_gc_6hop_mac_and_system_cmds();
    test_v67_mqtt_lwt_alias_and_netkey();
    test_v68_failover_mac_recovery_and_mac_mqtt_routing();
    test_v70_indoor_locate_rssi_and_tags();
    test_v71_ble_ibeacon_eddystone_and_relay();
    test_v71_deep_audit_2hop_anchor_and_stale_gc();

    printf("\n------------------------------------------------------------\n");
    printf("SUMMARY: %d passed, %d failed\n", g_tests_passed, g_tests_failed);
    printf("------------------------------------------------------------\n");
    return (g_tests_failed == 0) ? 0 : 1;
}
