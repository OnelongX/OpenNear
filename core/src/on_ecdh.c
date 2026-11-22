#include "on_ecdh.h"
#include <string.h>

/*
 * Compact Constant-Time Curve25519 (GF(2^255 - 19)) Field Arithmetic
 * Uses 16 limbs of int64_t (16 bits per limb, TweetNaCl / Kleppmann-style reference).
 */
typedef int64_t gf16_t[16];

static void gf_zero(gf16_t out) {
    for (int i = 0; i < 16; ++i) out[i] = 0;
}

static void gf_copy(gf16_t out, const gf16_t in) {
    for (int i = 0; i < 16; ++i) out[i] = in[i];
}

static void gf_add(gf16_t out, const gf16_t a, const gf16_t b) {
    for (int i = 0; i < 16; ++i) out[i] = a[i] + b[i];
}

static void gf_sub(gf16_t out, const gf16_t a, const gf16_t b) {
    for (int i = 0; i < 16; ++i) out[i] = a[i] - b[i];
}

static void gf_carry(gf16_t o) {
    for (int i = 0; i < 16; ++i) {
        o[i] += (1LL << 16);
        int64_t c = o[i] >> 16;
        o[(i + 1) * (i < 15)] += c - 1 + 37 * (c - 1) * (i == 15);
        o[i] -= (c << 16);
    }
}

static void gf_cswap(gf16_t p, gf16_t q, int64_t b) {
    int64_t mask = ~(b - 1);
    for (int i = 0; i < 16; ++i) {
        int64_t t = mask & (p[i] ^ q[i]);
        p[i] ^= t;
        q[i] ^= t;
    }
}

static void gf_mul(gf16_t out, const gf16_t a, const gf16_t b) {
    int64_t t[31];
    for (int i = 0; i < 31; ++i) t[i] = 0;
    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 16; ++j) {
            t[i + j] += a[i] * b[j];
        }
    }
    for (int i = 0; i < 15; ++i) {
        t[i] += 38 * t[i + 16];
    }
    for (int i = 0; i < 16; ++i) out[i] = t[i];
    gf_carry(out);
    gf_carry(out);
}

static void gf_sq(gf16_t out, const gf16_t a) {
    gf_mul(out, a, a);
}

static void gf_inv(gf16_t out, const gf16_t in) {
    gf16_t c;
    gf_copy(c, in);
    for (int i = 253; i >= 0; --i) {
        gf_sq(c, c);
        if (i != 2 && i != 4) {
            gf_mul(c, c, in);
        }
    }
    gf_copy(out, c);
}

static void gf_unpack(gf16_t o, const uint8_t n[32]) {
    for (int i = 0; i < 16; ++i) {
        o[i] = (int64_t)n[2 * i] + ((int64_t)n[2 * i + 1] << 8);
    }
    o[15] &= 0x7FFF;
}

static void gf_pack(uint8_t o[32], const gf16_t n) {
    gf16_t m, t;
    gf_copy(t, n);
    gf_carry(t);
    gf_carry(t);
    gf_carry(t);
    for (int j = 0; j < 2; ++j) {
        m[0] = t[0] - 0xFFED;
        for (int i = 1; i < 15; ++i) {
            m[i] = t[i] - 0xFFFF - ((m[i - 1] >> 16) & 1);
            m[i - 1] &= 0xFFFF;
        }
        m[15] = t[15] - 0x7FFF - ((m[14] >> 16) & 1);
        int64_t b = (m[15] >> 16) & 1;
        m[14] &= 0xFFFF;
        gf_cswap(t, m, 1 - b);
    }
    for (int i = 0; i < 16; ++i) {
        o[2 * i]     = (uint8_t)(t[i] & 0xFF);
        o[2 * i + 1] = (uint8_t)((t[i] >> 8) & 0xFF);
    }
}

void on_x25519_scalarmult(uint8_t out32[ON_X25519_KEY_LEN],
                          const uint8_t scalar32[ON_X25519_KEY_LEN],
                          const uint8_t point32[ON_X25519_KEY_LEN]) {
    uint8_t z[32];
    gf16_t x, a, b, c, d, e, f, k121665;
    memcpy(z, scalar32, 32u);
    /* RFC 7748 scalar clamping */
    z[0]  &= 248u;
    z[31] &= 127u;
    z[31] |= 64u;

    gf_unpack(x, point32);
    gf_zero(a); gf_zero(b); gf_zero(c); gf_zero(d);
    a[0] = 1;
    d[0] = 1;
    gf_copy(b, x);
    gf_zero(k121665);
    k121665[0] = 0xDB41;
    k121665[1] = 1;

    for (int i = 254; i >= 0; --i) {
        int64_t bit = (z[i >> 3] >> (i & 7)) & 1;
        gf_cswap(a, b, bit);
        gf_cswap(c, d, bit);
        gf_add(e, a, c);
        gf_sub(a, a, c);
        gf_add(c, b, d);
        gf_sub(b, b, d);
        gf_sq(d, e);
        gf_sq(f, a);
        gf_mul(a, c, a);
        gf_mul(c, b, e);
        gf_add(e, a, c);
        gf_sub(a, a, c);
        gf_sq(b, a);
        gf_sub(c, d, f);
        gf_mul(a, c, k121665);
        gf_add(a, a, d);
        gf_mul(c, c, a);
        gf_mul(a, d, f);
        gf_mul(d, b, x);
        gf_sq(b, e);
        gf_cswap(a, b, bit);
        gf_cswap(c, d, bit);
    }
    gf_inv(c, c);
    gf_mul(a, a, c);
    gf_pack(out32, a);
}

void on_x25519_public_key(const uint8_t priv32[ON_X25519_KEY_LEN],
                          uint8_t pub32_out[ON_X25519_KEY_LEN]) {
    uint8_t base9[32];
    memset(base9, 0, sizeof(base9));
    base9[0] = 9u;
    on_x25519_scalarmult(pub32_out, priv32, base9);
}

on_err_t on_x25519_shared_secret(const uint8_t my_priv32[ON_X25519_KEY_LEN],
                                 const uint8_t peer_pub32[ON_X25519_KEY_LEN],
                                 uint8_t shared32_out[ON_X25519_KEY_LEN]) {
    if (!my_priv32 || !peer_pub32 || !shared32_out) return ON_ERR_INVALID_ARG;
    on_x25519_scalarmult(shared32_out, my_priv32, peer_pub32);

    /* Reject all-zero low-order point result (RFC 7748 Section 6.1) */
    uint8_t acc = 0u;
    for (size_t i = 0; i < ON_X25519_KEY_LEN; ++i) acc |= shared32_out[i];
    return (acc != 0u) ? ON_OK : ON_ERR_DECODE_FAIL;
}

void on_ecdh_derive_onboarding_key(const uint8_t shared32[ON_X25519_KEY_LEN],
                                   uint32_t oob_pin_code,
                                   const uint8_t joiner_mac48[6],
                                   uint8_t net_key16_out[ON_SEC_KEY_LEN],
                                   uint8_t confirm_tag4_out[4]) {
    /* Two-stage AES-128 Davies-Meyer compression over 32B X25519 secret + OOB PIN + MAC48 */
    uint8_t stage1[16], blk2[16], stage2[16];
    on_aes128_encrypt_block(shared32, shared32 + 16, stage1);
    for (int i = 0; i < 16; ++i) stage1[i] ^= shared32[16 + i];

    blk2[0] = (uint8_t)(oob_pin_code & 0xFFu);
    blk2[1] = (uint8_t)((oob_pin_code >> 8) & 0xFFu);
    blk2[2] = (uint8_t)((oob_pin_code >> 16) & 0xFFu);
    blk2[3] = (uint8_t)((oob_pin_code >> 24) & 0xFFu);
    if (joiner_mac48) {
        memcpy(&blk2[4], joiner_mac48, 6u);
    } else {
        memset(&blk2[4], 0, 6u);
    }
    memcpy(&blk2[10], "ON_OOB", 6u);

    on_aes128_encrypt_block(stage1, blk2, net_key16_out);
    if (confirm_tag4_out) {
        blk2[15] ^= 0xA5u;
        on_aes128_encrypt_block(net_key16_out, blk2, stage2);
        memcpy(confirm_tag4_out, stage2, 4u);
    }
}
