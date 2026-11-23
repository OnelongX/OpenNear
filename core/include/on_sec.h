#ifndef OPENNEAR_SEC_H
#define OPENNEAR_SEC_H

#include "on_types.h"
#include "on_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v2.0 Hardware-Aligned Security Layer: AES-128-CCM (AEAD, M=4, L=2)
 *
 * - Zero over-the-air Nonce bytes! The 13-byte CCM Nonce is deterministically
 *   reconstructed from (src_id, sf_seq, slot_idx, frame_type, is_sub_g).
 * - Full 6-byte compact MAC header is authenticated as AAD (tampering with
 *   Short-ID, slot_idx, or flags immediately fails MIC verification).
 * - Adds only 4 bytes (32-bit CBC-MAC MIC tag) to encrypted payloads.
 * - Includes per-source monotonic replay protection window.
 */

#define ON_SEC_KEY_LEN        16u
#define ON_SEC_NONCE_LEN      13u
#define ON_SEC_MIC_LEN        4u
#define ON_SEC_REPLAY_SLOTS   32u

typedef struct {
    bool     used;
    uint16_t src_id;
    uint64_t last_watermark; /* ((uint64_t)sf_seq << 8) | (slot_idx << 4) | type */
} on_replay_entry_t;

typedef struct {
    bool              enabled;
    uint8_t           master_key[ON_SEC_KEY_LEN];
    uint8_t           key[ON_SEC_KEY_LEN];        /* Active KDF-derived session key */
    uint32_t          session_salt;               /* Per-boot salt preventing post-reboot Nonce reuse */
    uint8_t           key_epoch;                  /* Active key rotation epoch */
    on_replay_entry_t replay_tbl[ON_SEC_REPLAY_SLOTS];
    uint32_t          auth_failures;
    uint32_t          replay_drops;
    uint32_t          beacon_auth_drops;
} on_sec_ctx_t;

/* Low-level AES-128 block cipher (NIST FIPS 197 test-vector verified) */
void     on_aes128_encrypt_block(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

/* AES-128-CCM (M=4, L=2) AEAD encrypt & decrypt */
on_err_t on_ccm_encrypt(const uint8_t key[ON_SEC_KEY_LEN],
                        const uint8_t nonce[ON_SEC_NONCE_LEN],
                        const uint8_t *aad, size_t aad_len,
                        const uint8_t *plain, size_t plain_len,
                        uint8_t *cipher_out, uint8_t mic_out[ON_SEC_MIC_LEN]);

on_err_t on_ccm_decrypt(const uint8_t key[ON_SEC_KEY_LEN],
                        const uint8_t nonce[ON_SEC_NONCE_LEN],
                        const uint8_t *aad, size_t aad_len,
                        const uint8_t *cipher, size_t cipher_len,
                        const uint8_t mic[ON_SEC_MIC_LEN],
                        uint8_t *plain_out);

/* Security context management, KDF session key rotation & deterministic Nonce builder */
void     on_sec_init(on_sec_ctx_t *ctx);
void     on_sec_set_key(on_sec_ctx_t *ctx, const uint8_t master_key[ON_SEC_KEY_LEN]);
void     on_sec_set_session(on_sec_ctx_t *ctx, uint32_t session_salt, uint8_t key_epoch);
void     on_sec_derive_session_key(const uint8_t master_key[ON_SEC_KEY_LEN],
                                   uint32_t session_salt, uint8_t key_epoch,
                                   uint8_t session_key_out[ON_SEC_KEY_LEN]);
void     on_sec_build_nonce(const on_mac_hdr_t *hdr, uint32_t sf_seq,
                            uint8_t nonce_out[ON_SEC_NONCE_LEN]);
void     on_sec_build_nonce_salted(const on_mac_hdr_t *hdr, uint32_t sf_seq,
                                   uint32_t session_salt,
                                   uint8_t nonce_out[ON_SEC_NONCE_LEN]);

/* v3.0 SYNC/MAP & SUB_SYNC Beacon anti-spoofing 32-bit CBC-MAC authentication */
void     on_sec_compute_beacon_mac(const on_sec_ctx_t *ctx,
                                   const uint8_t hdr6[ON_MAC_HDR_LEN],
                                   const uint8_t *sync_wire_40b,
                                   uint32_t sf_seq,
                                   uint32_t session_salt,
                                   uint8_t mac4_out[4]);
bool     on_sec_verify_beacon_mac(on_sec_ctx_t *ctx,
                                  const uint8_t hdr6[ON_MAC_HDR_LEN],
                                  const uint8_t *sync_wire_44b,
                                  uint32_t sf_seq,
                                  uint32_t session_salt);

/* Verify and advance monotonic anti-replay watermark for a source Short-ID */
bool     on_sec_check_replay(on_sec_ctx_t *ctx, uint16_t src_id,
                             uint32_t sf_seq, uint8_t slot_idx, uint8_t frame_type);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_SEC_H */
