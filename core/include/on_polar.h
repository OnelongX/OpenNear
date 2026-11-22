#ifndef OPENNEAR_POLAR_H
#define OPENNEAR_POLAR_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear L1 Control-Plane Short CA-Polar Code (CRC-8 Aided Polar, N=128)
 *
 * Uses the 3GPP TS 38.212 5G NR Polar reliability sequence Q_0^{127}.
 * Default control block profile:
 *   Code length N = 128 bits
 *   Message bits A = 40 bits (5 bytes) + CRC-8 = 48 bits (K = 48, rate 0.375)
 *   Decoded via CRC-Aided Successive Cancellation List (CA-SCL, L = 8).
 *
 * Designed so SYNC/MAP control headers survive severe negative-SNR interference
 * while business payloads can adaptively switch to higher-rate Polar/LDPC.
 */

#define ON_POLAR_N            128u
#define ON_POLAR_LOG2_N       7u
#define ON_POLAR_MSG_BYTES    5u
#define ON_POLAR_MSG_BITS     (ON_POLAR_MSG_BYTES * 8u)   /* 40 bits */
#define ON_POLAR_CRC_BITS     8u
#define ON_POLAR_K            (ON_POLAR_MSG_BITS + ON_POLAR_CRC_BITS) /* 48 bits */
#define ON_POLAR_MAX_MSG_BYTES 11u                        /* Max K = 11*8 + 8 = 96 (Rate 0.75) */
#define ON_POLAR_MAX_K        (ON_POLAR_MAX_MSG_BYTES * 8u + ON_POLAR_CRC_BITS)
#define ON_POLAR_LIST_SIZE    8u

/*
 * v4.0 Soft-Combining HARQ LLR Accumulator (Chase Combining / IR-HARQ):
 * Retains soft Q4.4 LLRs across failed transmissions and combines them
 * (L_comb[i] = sat(L_prev[i] + L_new[i])) for +3 dB SNR gain on retransmission.
 */
typedef struct {
    int16_t  accum_llr_q4[ON_POLAR_N];
    uint8_t  tx_count;
    size_t   msg_bytes;
} on_polar_harq_buf_t;

/*
 * Encode 5 message bytes (40 bits) -> appends CRC-8 (total 48 info bits) ->
 * maps onto top-48 reliable 5G NR bit-channels -> outputs 128 coded bits (0 or 1).
 */
on_err_t on_polar_encode_block(const uint8_t msg[ON_POLAR_MSG_BYTES],
                               uint8_t coded_bits[ON_POLAR_N]);

/*
 * CA-SCL Decode 128 channel LLRs (float reference) -> outputs 5 recovered message bytes
 * from the lowest path-metric candidate that passes CRC-8 verification.
 */
on_err_t on_polar_decode_block(const float llr[ON_POLAR_N],
                               uint8_t msg_out[ON_POLAR_MSG_BYTES]);

/*
 * v3.0 Fixed-Point (Q4.4 int16_t, zero float math) Embedded / FPGA-HLS CA-SCL Decoder.
 * Quantizes channel LLRs with scale = 16 (1.0 -> 16, clipped to [-512, +511]) and uses
 * exact hardware min-sum path metrics for MCU & RTL portability.
 */
on_err_t on_polar_decode_block_fixed(const int16_t llr_q4[ON_POLAR_N],
                                     uint8_t msg_out[ON_POLAR_MSG_BYTES]);

/*
 * v4.0 Multi-Rate Adaptive CA-Polar (N=128, msg_bytes = 4..11 -> K = 40..96, R = 0.31..0.75)
 * Supports NearLink-style dynamic code-rate adaptation linked with AMC SNR regimes.
 */
on_err_t on_polar_encode_rate(const uint8_t *msg, size_t msg_bytes,
                              uint8_t coded_bits[ON_POLAR_N]);
on_err_t on_polar_decode_rate_fixed(const int16_t llr_q4[ON_POLAR_N],
                                    size_t msg_bytes,
                                    uint8_t *msg_out);

/*
 * v4.0 Soft-Combining HARQ API:
 * Accumulates Q4.4 soft LLRs across initial TX and HARQ retransmissions before CA-SCL decoding.
 */
void     on_polar_harq_init(on_polar_harq_buf_t *hb, size_t msg_bytes);
on_err_t on_polar_harq_combine_and_decode(on_polar_harq_buf_t *hb,
                                          const int16_t rx_llr_q4[ON_POLAR_N],
                                          uint8_t *msg_out);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_POLAR_H */
