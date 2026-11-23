#include "on_sec.h"
#include <string.h>

/* Standard AES S-Box (FIPS 197) */
static const uint8_t kSbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static const uint8_t kRcon[10] = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36
};

static uint8_t xtime(uint8_t x) {
    return (uint8_t)((x << 1) ^ ((x & 0x80u) ? 0x1bu : 0x00u));
}

static void aes128_key_expand(const uint8_t key[16], uint8_t rk[176]) {
    memcpy(rk, key, 16u);
    for (uint32_t i = 4u; i < 44u; ++i) {
        uint8_t tmp[4];
        memcpy(tmp, &rk[(i - 1u) * 4u], 4u);
        if ((i % 4u) == 0u) {
            uint8_t t = tmp[0];
            tmp[0] = (uint8_t)(kSbox[tmp[1]] ^ kRcon[(i / 4u) - 1u]);
            tmp[1] = kSbox[tmp[2]];
            tmp[2] = kSbox[tmp[3]];
            tmp[3] = kSbox[t];
        }
        for (uint32_t b = 0u; b < 4u; ++b) {
            rk[i * 4u + b] = (uint8_t)(rk[(i - 4u) * 4u + b] ^ tmp[b]);
        }
    }
}

void on_aes128_encrypt_block(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
    uint8_t rk[176];
    aes128_key_expand(key, rk);

    uint8_t s[16];
    for (uint32_t i = 0u; i < 16u; ++i) s[i] = (uint8_t)(in[i] ^ rk[i]);

    for (uint32_t round = 1u; round <= 10u; ++round) {
        /* SubBytes */
        for (uint32_t i = 0u; i < 16u; ++i) s[i] = kSbox[s[i]];

        /* ShiftRows */
        uint8_t t;
        t = s[1]; s[1] = s[5]; s[5] = s[9];  s[9]  = s[13]; s[13] = t;
        t = s[2]; s[2] = s[10]; s[10] = t;
        t = s[6]; s[6] = s[14]; s[14] = t;
        t = s[15]; s[15] = s[11]; s[11] = s[7]; s[7] = s[3]; s[3] = t;

        /* MixColumns (rounds 1..9) */
        if (round < 10u) {
            for (uint32_t c = 0u; c < 4u; ++c) {
                uint8_t *col = &s[c * 4u];
                uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
                uint8_t xr = (uint8_t)(a0 ^ a1 ^ a2 ^ a3);
                col[0] ^= (uint8_t)(xr ^ xtime((uint8_t)(a0 ^ a1)));
                col[1] ^= (uint8_t)(xr ^ xtime((uint8_t)(a1 ^ a2)));
                col[2] ^= (uint8_t)(xr ^ xtime((uint8_t)(a2 ^ a3)));
                col[3] ^= (uint8_t)(xr ^ xtime((uint8_t)(a3 ^ a0)));
            }
        }

        /* AddRoundKey */
        for (uint32_t i = 0u; i < 16u; ++i) {
            s[i] ^= rk[round * 16u + i];
        }
    }
    memcpy(out, s, 16u);
}

/* Compute CBC-MAC tag T (M=4, L=2) per RFC 3610 */
static void ccm_compute_cbc_mac(const uint8_t key[16],
                                const uint8_t nonce[ON_SEC_NONCE_LEN],
                                const uint8_t *aad, size_t aad_len,
                                const uint8_t *plain, size_t plain_len,
                                uint8_t tag_out[16]) {
    uint8_t b0[16];
    uint8_t flags = (uint8_t)((aad_len > 0u ? 0x40u : 0x00u) |
                              (((ON_SEC_MIC_LEN - 2u) / 2u) << 3) |
                              (2u - 1u)); /* L=2 -> L'=1 */
    b0[0] = flags;
    memcpy(&b0[1], nonce, ON_SEC_NONCE_LEN);
    b0[14] = (uint8_t)((plain_len >> 8) & 0xFFu);
    b0[15] = (uint8_t)(plain_len & 0xFFu);

    uint8_t x[16];
    on_aes128_encrypt_block(key, b0, x);

    /* Absorb AAD (supports aad_len < 65280) */
    if (aad_len > 0u && aad) {
        uint8_t blk[16];
        size_t pos = 0u;
        memset(blk, 0, sizeof(blk));
        blk[0] = (uint8_t)((aad_len >> 8) & 0xFFu);
        blk[1] = (uint8_t)(aad_len & 0xFFu);
        size_t first = aad_len < 14u ? aad_len : 14u;
        memcpy(&blk[2], aad, first);
        pos += first;
        for (size_t i = 0u; i < 16u; ++i) blk[i] ^= x[i];
        on_aes128_encrypt_block(key, blk, x);

        while (pos < aad_len) {
            memset(blk, 0, sizeof(blk));
            size_t chunk = (aad_len - pos) < 16u ? (aad_len - pos) : 16u;
            memcpy(blk, aad + pos, chunk);
            pos += chunk;
            for (size_t i = 0u; i < 16u; ++i) blk[i] ^= x[i];
            on_aes128_encrypt_block(key, blk, x);
        }
    }

    /* Absorb plaintext */
    size_t pos = 0u;
    while (pos < plain_len) {
        uint8_t blk[16];
        memset(blk, 0, sizeof(blk));
        size_t chunk = (plain_len - pos) < 16u ? (plain_len - pos) : 16u;
        memcpy(blk, plain + pos, chunk);
        pos += chunk;
        for (size_t i = 0u; i < 16u; ++i) blk[i] ^= x[i];
        on_aes128_encrypt_block(key, blk, x);
    }

    memcpy(tag_out, x, 16u);
}

static void ccm_ctr_crypt(const uint8_t key[16],
                          const uint8_t nonce[ON_SEC_NONCE_LEN],
                          const uint8_t *in, size_t len,
                          uint8_t *out, uint8_t s0_out[16]) {
    uint8_t a[16];
    a[0] = 1u; /* L' = L - 1 = 1 */
    memcpy(&a[1], nonce, ON_SEC_NONCE_LEN);
    a[14] = 0u;
    a[15] = 0u;
    on_aes128_encrypt_block(key, a, s0_out);

    size_t pos = 0u;
    uint16_t ctr = 1u;
    while (pos < len) {
        a[14] = (uint8_t)((ctr >> 8) & 0xFFu);
        a[15] = (uint8_t)(ctr & 0xFFu);
        uint8_t ks[16];
        on_aes128_encrypt_block(key, a, ks);
        size_t chunk = (len - pos) < 16u ? (len - pos) : 16u;
        for (size_t i = 0u; i < chunk; ++i) {
            out[pos + i] = (uint8_t)(in[pos + i] ^ ks[i]);
        }
        pos += chunk;
        ctr++;
    }
}

on_err_t on_ccm_encrypt(const uint8_t key[ON_SEC_KEY_LEN],
                        const uint8_t nonce[ON_SEC_NONCE_LEN],
                        const uint8_t *aad, size_t aad_len,
                        const uint8_t *plain, size_t plain_len,
                        uint8_t *cipher_out, uint8_t mic_out[ON_SEC_MIC_LEN]) {
    if (!key || !nonce || !mic_out) return ON_ERR_INVALID_ARG;
    if (plain_len > 0u && (!plain || !cipher_out)) return ON_ERR_INVALID_ARG;

    uint8_t tag[16], s0[16];
    ccm_compute_cbc_mac(key, nonce, aad, aad_len, plain, plain_len, tag);
    ccm_ctr_crypt(key, nonce, plain, plain_len, cipher_out, s0);

    for (size_t i = 0u; i < ON_SEC_MIC_LEN; ++i) {
        mic_out[i] = (uint8_t)(tag[i] ^ s0[i]);
    }
    return ON_OK;
}

on_err_t on_ccm_decrypt(const uint8_t key[ON_SEC_KEY_LEN],
                        const uint8_t nonce[ON_SEC_NONCE_LEN],
                        const uint8_t *aad, size_t aad_len,
                        const uint8_t *cipher, size_t cipher_len,
                        const uint8_t mic[ON_SEC_MIC_LEN],
                        uint8_t *plain_out) {
    if (!key || !nonce || !mic) return ON_ERR_INVALID_ARG;
    if (cipher_len > 0u && (!cipher || !plain_out)) return ON_ERR_INVALID_ARG;

    uint8_t s0[16], tag[16];
    ccm_ctr_crypt(key, nonce, cipher, cipher_len, plain_out, s0);
    ccm_compute_cbc_mac(key, nonce, aad, aad_len, plain_out, cipher_len, tag);

    /* Constant-time MIC comparison */
    uint8_t diff = 0u;
    for (size_t i = 0u; i < ON_SEC_MIC_LEN; ++i) {
        diff |= (uint8_t)((tag[i] ^ s0[i]) ^ mic[i]);
    }
    if (diff != 0u) {
        if (cipher_len > 0u) memset(plain_out, 0, cipher_len);
        return ON_ERR_DECODE_FAIL;
    }
    return ON_OK;
}

void on_sec_derive_session_key(const uint8_t master_key[ON_SEC_KEY_LEN],
                               uint32_t session_salt, uint8_t key_epoch,
                               uint8_t session_key_out[ON_SEC_KEY_LEN]) {
    /* AES-128 PRF KDF: session_key = AES_master(salt || epoch || "OPENNEAR_KD") */
    uint8_t blk[16];
    blk[0]  = (uint8_t)(session_salt & 0xFFu);
    blk[1]  = (uint8_t)((session_salt >> 8) & 0xFFu);
    blk[2]  = (uint8_t)((session_salt >> 16) & 0xFFu);
    blk[3]  = (uint8_t)((session_salt >> 24) & 0xFFu);
    blk[4]  = key_epoch;
    memcpy(&blk[5], "OPENNEAR_KD", 11u);
    on_aes128_encrypt_block(master_key, blk, session_key_out);
}

void on_sec_init(on_sec_ctx_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

void on_sec_set_key(on_sec_ctx_t *ctx, const uint8_t master_key[ON_SEC_KEY_LEN]) {
    if (!ctx || !master_key) return;
    memcpy(ctx->master_key, master_key, ON_SEC_KEY_LEN);
    ctx->enabled = true;
    on_sec_derive_session_key(ctx->master_key, ctx->session_salt, ctx->key_epoch, ctx->key);
    memset(ctx->replay_tbl, 0, sizeof(ctx->replay_tbl));
}

void on_sec_set_session(on_sec_ctx_t *ctx, uint32_t session_salt, uint8_t key_epoch) {
    if (!ctx) return;
    if (ctx->session_salt == session_salt && ctx->key_epoch == key_epoch && ctx->enabled) {
        return;
    }
    ctx->session_salt = session_salt;
    ctx->key_epoch    = key_epoch;
    if (ctx->enabled) {
        on_sec_derive_session_key(ctx->master_key, ctx->session_salt, ctx->key_epoch, ctx->key);
        /* Reset replay watermarks cleanly when salt changes (e.g. G reboot) */
        memset(ctx->replay_tbl, 0, sizeof(ctx->replay_tbl));
    }
}

void on_sec_build_nonce_salted(const on_mac_hdr_t *hdr, uint32_t sf_seq,
                               uint32_t session_salt,
                               uint8_t nonce_out[ON_SEC_NONCE_LEN]) {
    if (!hdr || !nonce_out) return;
    /* 13-byte deterministic Nonce mixing session_salt so rebooting G never reuses Nonces */
    uint16_t src_xor = (uint16_t)(hdr->src_id ^ (uint16_t)(session_salt & 0xFFFFu));
    uint16_t dst_xor = (uint16_t)(hdr->dst_id ^ (uint16_t)((session_salt >> 16) & 0xFFFFu));
    nonce_out[0]  = (uint8_t)((src_xor >> 8) & 0xFFu);
    nonce_out[1]  = (uint8_t)(src_xor & 0xFFu);
    nonce_out[2]  = (uint8_t)((dst_xor >> 8) & 0xFFu);
    nonce_out[3]  = (uint8_t)(dst_xor & 0xFFu);
    nonce_out[4]  = (uint8_t)(sf_seq & 0xFFu);
    nonce_out[5]  = (uint8_t)((sf_seq >> 8) & 0xFFu);
    nonce_out[6]  = (uint8_t)((sf_seq >> 16) & 0xFFu);
    nonce_out[7]  = (uint8_t)((sf_seq >> 24) & 0xFFu);
    nonce_out[8]  = hdr->slot_idx;
    nonce_out[9]  = hdr->type;
    nonce_out[10] = hdr->is_sub_g ? 1u : 0u;
    nonce_out[11] = (uint8_t)((session_salt >> 8) & 0xFFu) ^ 0x4Fu;
    nonce_out[12] = (uint8_t)((session_salt >> 24) & 0xFFu) ^ 0x4Eu;
}

void on_sec_build_nonce(const on_mac_hdr_t *hdr, uint32_t sf_seq,
                        uint8_t nonce_out[ON_SEC_NONCE_LEN]) {
    on_sec_build_nonce_salted(hdr, sf_seq, 0u, nonce_out);
}

void on_sec_compute_beacon_mac(const on_sec_ctx_t *ctx,
                               const uint8_t hdr6[ON_MAC_HDR_LEN],
                               const uint8_t *sync_wire_40b,
                               uint32_t sf_seq,
                               uint32_t session_salt,
                               uint8_t mac4_out[4]) {
    if (!ctx || !hdr6 || !sync_wire_40b || !mac4_out) return;
    /* Authenticate 6B header + 40B beacon body using master_key (so late-joiners can verify before salt sync) */
    uint8_t aad[ON_MAC_HDR_LEN + 40u];
    memcpy(aad, hdr6, ON_MAC_HDR_LEN);
    memcpy(aad + ON_MAC_HDR_LEN, sync_wire_40b, 40u);

    uint8_t nonce[ON_SEC_NONCE_LEN];
    nonce[0]  = 0xBCu; /* Beacon domain separator */
    nonce[1]  = (uint8_t)(session_salt & 0xFFu);
    nonce[2]  = (uint8_t)((session_salt >> 8) & 0xFFu);
    nonce[3]  = (uint8_t)((session_salt >> 16) & 0xFFu);
    nonce[4]  = (uint8_t)((session_salt >> 24) & 0xFFu);
    nonce[5]  = (uint8_t)(sf_seq & 0xFFu);
    nonce[6]  = (uint8_t)((sf_seq >> 8) & 0xFFu);
    nonce[7]  = (uint8_t)((sf_seq >> 16) & 0xFFu);
    nonce[8]  = (uint8_t)((sf_seq >> 24) & 0xFFu);
    nonce[9]  = hdr6[0];
    nonce[10] = hdr6[1];
    nonce[11] = hdr6[2];
    nonce[12] = hdr6[3];

    on_ccm_encrypt(ctx->master_key, nonce, aad, sizeof(aad), NULL, 0u, NULL, mac4_out);
}

bool on_sec_verify_beacon_mac(on_sec_ctx_t *ctx,
                              const uint8_t hdr6[ON_MAC_HDR_LEN],
                              const uint8_t *sync_wire_44b,
                              uint32_t sf_seq,
                              uint32_t session_salt) {
    if (!ctx || !ctx->enabled) return true;
    uint8_t expected[4];
    on_sec_compute_beacon_mac(ctx, hdr6, sync_wire_44b, sf_seq, session_salt, expected);
    uint8_t diff = 0u;
    for (int i = 0; i < 4; ++i) {
        diff |= (uint8_t)(expected[i] ^ sync_wire_44b[40 + i]);
    }
    if (diff != 0u) {
        ctx->beacon_auth_drops++;
        return false;
    }
    return true;
}

bool on_sec_check_replay(on_sec_ctx_t *ctx, uint16_t src_id,
                         uint32_t sf_seq, uint8_t slot_idx, uint8_t frame_type) {
    if (!ctx || !ctx->enabled) return true;
    /* Unjoined nodes (0x000) use C-RA join handshake before Short-ID assignment */
    if (src_id == ON_SHORT_ID_UNASSIGNED) return true;

    /* Chronological monotonic watermark:
     *   sf_seq > phase (0=D-TDMA slot, 1=C-RA/service) > frame_type > slot_idx
     * Putting D-TDMA slot-local GW_BRIDGE (slot_idx < 12, e.g. v6.5 MAC_REPORT) in phase 0
     * ensures a subsequent C-RA RTT_REQ/ALERT in the same superframe stays strictly monotonic. */
    uint64_t phase = (frame_type == ON_FRAME_TDMA_DATA ||
                      (frame_type == ON_FRAME_GW_BRIDGE && slot_idx < 12u)) ? 0u : 1u;
    uint64_t wm = ((uint64_t)sf_seq << 9)
                | (phase << 8)
                | ((uint64_t)(frame_type & 0x0Fu) << 4)
                | (uint64_t)(slot_idx & 0x0Fu);

    int free_idx = -1;
    uint32_t oldest_idx = 0u;
    for (uint32_t i = 0u; i < ON_SEC_REPLAY_SLOTS; ++i) {
        if (ctx->replay_tbl[i].used && ctx->replay_tbl[i].src_id == src_id) {
            if (wm <= ctx->replay_tbl[i].last_watermark) {
                ctx->replay_drops++;
                return false;
            }
            ctx->replay_tbl[i].last_watermark = wm;
            return true;
        }
        if (!ctx->replay_tbl[i].used && free_idx < 0) {
            free_idx = (int)i;
        }
        if (ctx->replay_tbl[i].used &&
            ctx->replay_tbl[i].last_watermark < ctx->replay_tbl[oldest_idx].last_watermark) {
            oldest_idx = i;
        }
    }
    /* Evict oldest watermark when full so anti-replay never fails open */
    uint32_t target = (free_idx >= 0) ? (uint32_t)free_idx : oldest_idx;
    ctx->replay_tbl[target].used           = true;
    ctx->replay_tbl[target].src_id         = src_id;
    ctx->replay_tbl[target].last_watermark = wm;
    return true;
}
