#ifndef OPENNEAR_ECDH_H
#define OPENNEAR_ECDH_H

#include "on_types.h"
#include "on_sec.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v3.1 Zero-Touch Asymmetric Onboarding: X25519 (RFC 7748 Curve25519) + OOB Passkey KDF
 *
 * - Pure C11, zero external dependencies, constant-time Montgomery ladder over GF(2^255 - 19).
 * - Verified bit-exact against official RFC 7748 Section 6.1 test vectors.
 * - Allows unprovisioned nodes to establish a cryptographically authenticated 128-bit
 *   OpenNear network/session key using ephemeral X25519 public keys + a 32-bit OOB PIN/QR code.
 */

#define ON_X25519_KEY_LEN  32u

/* Raw X25519 scalar multiplication: out32 = scalar32 * point32 on Curve25519 */
void     on_x25519_scalarmult(uint8_t out32[ON_X25519_KEY_LEN],
                              const uint8_t scalar32[ON_X25519_KEY_LEN],
                              const uint8_t point32[ON_X25519_KEY_LEN]);

/* Generate X25519 public key from 32-byte private key (using base point u = 9) */
void     on_x25519_public_key(const uint8_t priv32[ON_X25519_KEY_LEN],
                              uint8_t pub32_out[ON_X25519_KEY_LEN]);

/* Compute 32-byte shared secret from local private key and peer's 32-byte public key */
on_err_t on_x25519_shared_secret(const uint8_t my_priv32[ON_X25519_KEY_LEN],
                                 const uint8_t peer_pub32[ON_X25519_KEY_LEN],
                                 uint8_t shared32_out[ON_X25519_KEY_LEN]);

/* Derive 16-byte OpenNear AES-128 key + 4-byte OOB confirmation tag from X25519 shared secret + PIN */
void     on_ecdh_derive_onboarding_key(const uint8_t shared32[ON_X25519_KEY_LEN],
                                       uint32_t oob_pin_code,
                                       const uint8_t joiner_mac48[6],
                                       uint8_t net_key16_out[ON_SEC_KEY_LEN],
                                       uint8_t confirm_tag4_out[4]);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_ECDH_H */
