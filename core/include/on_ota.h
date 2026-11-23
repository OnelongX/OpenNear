#ifndef OPENNEAR_OTA_H
#define OPENNEAR_OTA_H

#include "on_types.h"
#include "on_sec.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v5.0 Authenticated Multi-Hop Node OTA Engine
 *
 * Features:
 * - Compact wire frames fitting inside OpenNear 120-byte MTU with AES-128-CCM AEAD
 * - Full-image IEEE 802.3 CRC-32 + 64-bit AES-128-CBC-MAC authenticity verification
 * - Anti-downgrade version guard + A/B partition rollback confirmation hook
 * - O(1)-RAM 32-chunk sliding-window selective-repeat ARQ (supports up to 6 MB images)
 * - 1-to-N Broadcast cluster OTA + 2-Hop Sub-G transparent relay OTA
 * - Synchronized superframe-aligned cluster activation (commit_sf_seq)
 */

#define ON_OTA_MAX_CHUNK_LEN     96u
#define ON_OTA_AUTH_TAG_LEN      8u
#define ON_OTA_WINDOW_CHUNKS     32u

#define ON_OTA_META_WIRE_LEN     30u
#define ON_OTA_CHUNK_HDR_LEN     6u
#define ON_OTA_STATUS_WIRE_LEN   14u
#define ON_OTA_COMMIT_WIRE_LEN   13u

/* OTA manifest & commit flags */
#define ON_OTA_FLAG_FORCE        0x01u /* Bypass anti-downgrade check */
#define ON_OTA_FLAG_AUTO_REBOOT  0x02u /* Activate immediately upon synchronized commit */
#define ON_OTA_FLAG_VERIFY_ONLY  0x04u /* Verify flash integrity without rebooting */
#define ON_OTA_FLAG_ABORT        0x08u /* Abort active OTA session / rollback */

typedef enum {
    ON_OTA_STATE_IDLE      = 0,
    ON_OTA_STATE_RECEIVING = 1,
    ON_OTA_STATE_VERIFYING = 2,
    ON_OTA_STATE_READY     = 3, /* Full image written & CRC32+MAC verified */
    ON_OTA_STATE_COMMITTED = 4, /* Scheduled for synchronized SF activation */
    ON_OTA_STATE_APPLIED   = 5, /* Storage apply_and_reboot executed */
    ON_OTA_STATE_FAILED    = 6
} on_ota_state_t;

typedef enum {
    ON_OTA_ERR_NONE          = 0,
    ON_OTA_ERR_DOWNGRADE     = 1,
    ON_OTA_ERR_TOO_LARGE     = 2,
    ON_OTA_ERR_STORAGE       = 3,
    ON_OTA_ERR_CRC_MISMATCH  = 4,
    ON_OTA_ERR_AUTH_FAIL     = 5,
    ON_OTA_ERR_BAD_PARAM     = 6,
    ON_OTA_ERR_ABORTED       = 7
} on_ota_err_t;

/* Wire payload structures */
typedef struct {
    uint16_t ota_id;
    uint32_t fw_version;       /* Monotonic SemVer packed: (major<<16)|(minor<<8)|patch */
    uint32_t total_size;       /* Total firmware binary length in bytes */
    uint16_t chunk_size;       /* Bytes per chunk (1..96) */
    uint16_t total_chunks;     /* ceil(total_size / chunk_size) */
    uint32_t crc32;            /* IEEE 802.3 CRC-32 of entire image */
    uint8_t  auth_tag[ON_OTA_AUTH_TAG_LEN]; /* 64-bit AES-128-CBC-MAC over manifest + image */
    uint16_t target_id;        /* Specific Short-ID or ON_SHORT_ID_BROADCAST (0xFFF) */
    uint8_t  flags;
    uint8_t  hw_compat;        /* Target hardware class (0=any, 1=ESP32, 2=nRF52840, 3=FPGA) */
} on_ota_meta_t;

typedef struct {
    uint16_t ota_id;
    uint16_t chunk_idx;
    uint8_t  data_len;
    uint8_t  flags;
    uint8_t  data[ON_OTA_MAX_CHUNK_LEN];
} on_ota_chunk_t;

typedef struct {
    uint16_t ota_id;
    uint16_t node_id;
    uint8_t  state;            /* on_ota_state_t */
    uint8_t  err_code;         /* on_ota_err_t */
    uint16_t next_missing;     /* Lowest contiguous unreceived chunk index */
    uint16_t rx_chunks;        /* Total unique chunks stored so far */
    uint32_t nack_bitmap;      /* Bit k=1 => chunk (next_missing + k) is missing */
} on_ota_status_t;

typedef struct {
    uint16_t ota_id;
    uint32_t commit_sf_seq;    /* Synchronized network SF at which to activate/reboot */
    uint32_t crc32;
    uint16_t target_id;
    uint8_t  flags;
} on_ota_commit_t;

/* Pluggable platform flash / partition storage HAL (ESP32 esp_ota_ops, nRF52, or RAM) */
typedef struct {
    void *user_ctx;
    on_err_t (*begin)(void *ctx, uint32_t total_size, uint32_t fw_version);
    on_err_t (*write_chunk)(void *ctx, uint32_t offset, const uint8_t *data, size_t len);
    on_err_t (*read_chunk)(void *ctx, uint32_t offset, uint8_t *out, size_t len);
    on_err_t (*finalize)(void *ctx, bool verified_ok);
    on_err_t (*apply_and_reboot)(void *ctx, uint32_t fw_version);
    on_err_t (*confirm_boot)(void *ctx);
} on_ota_storage_ops_t;

/* OTA Receiver Context (runs on Terminal / Sub-G / Backup nodes) */
typedef struct {
    on_ota_storage_ops_t storage;
    uint32_t       current_fw_version;
    uint8_t        hw_class;
    on_ota_state_t state;
    on_ota_err_t   last_err;

    on_ota_meta_t  meta;
    uint16_t       base_chunk;        /* Sliding window base (next_missing) */
    uint32_t       win_rx_bitmap;     /* Bit k=1 => base_chunk+k received */
    uint16_t       rx_chunks;
    uint16_t       dup_chunks;
    uint32_t       commit_sf_seq;
    uint8_t        commit_flags;
    bool           status_dirty;      /* True when a status/NACK report should be sent */
    uint32_t       last_status_sf;
    uint32_t       applied_version;
} on_ota_rx_t;

/* OTA Sender / Coordinator Context (runs on Primary G or Gateway) */
#define ON_OTA_MAX_TARGETS 16u

typedef struct {
    bool           active;
    uint16_t       node_id;
    uint8_t        state;
    uint8_t        err_code;
    uint16_t       next_missing;
    uint16_t       rx_chunks;
    uint32_t       nack_bitmap;
    uint32_t       last_status_sf;
} on_ota_target_track_t;

typedef struct {
    bool           active;
    bool           meta_sent;
    bool           commit_sent;
    on_ota_meta_t  meta;
    const uint8_t *image_buf;         /* Optional in-memory firmware buffer */
    on_err_t     (*read_image)(void *ctx, uint32_t offset, uint8_t *out, size_t len);
    void          *read_ctx;

    uint16_t       tx_cursor;         /* Next fresh chunk to stream */
    uint16_t       retx_base;         /* Selective-repeat retransmit window base */
    uint32_t       retx_bitmap;       /* Bitmask of missing chunks requested by targets */
    uint32_t       meta_repeat_cnt;
    uint32_t       commit_sf_seq;
    uint32_t       chunks_tx_total;
    uint32_t       chunks_retx_total;
    uint32_t       last_tx_sf;
    on_ota_target_track_t targets[ON_OTA_MAX_TARGETS];
} on_ota_tx_t;

/* Combined OTA context attachable to on_node_t */
typedef struct {
    on_ota_rx_t rx;
    on_ota_tx_t tx;
} on_ota_ctx_t;

/* RAM-backed storage helper (for host tests, simulator & in-memory verification) */
typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   written_size;
    bool     begun;
    bool     finalized;
    bool     applied;
    bool     boot_confirmed;
    uint32_t applied_fw_version;
} on_ota_ram_storage_t;

/* CRC-32 (IEEE 802.3) & 64-bit AES-128-CBC-MAC image authenticator */
uint32_t on_ota_crc32_update(uint32_t crc, const uint8_t *data, size_t len);
uint32_t on_ota_crc32(const uint8_t *data, size_t len);

void     on_ota_compute_auth_tag(const uint8_t key[ON_SEC_KEY_LEN],
                                 const on_ota_meta_t *meta,
                                 const uint8_t *image_data,
                                 uint8_t tag8_out[ON_OTA_AUTH_TAG_LEN]);

on_err_t on_ota_compute_auth_tag_reader(const uint8_t key[ON_SEC_KEY_LEN],
                                        const on_ota_meta_t *meta,
                                        on_err_t (*read_fn)(void *, uint32_t, uint8_t *, size_t),
                                        void *read_ctx,
                                        uint8_t tag8_out[ON_OTA_AUTH_TAG_LEN]);

/* Wire codecs */
on_err_t on_ota_meta_pack(const on_ota_meta_t *m, uint8_t *out, size_t cap, size_t *len);
on_err_t on_ota_meta_unpack(const uint8_t *in, size_t len, on_ota_meta_t *m);

on_err_t on_ota_chunk_pack(const on_ota_chunk_t *c, uint8_t *out, size_t cap, size_t *len);
on_err_t on_ota_chunk_unpack(const uint8_t *in, size_t len, on_ota_chunk_t *c);

on_err_t on_ota_status_pack(const on_ota_status_t *s, uint8_t *out, size_t cap, size_t *len);
on_err_t on_ota_status_unpack(const uint8_t *in, size_t len, on_ota_status_t *s);

on_err_t on_ota_commit_pack(const on_ota_commit_t *c, uint8_t *out, size_t cap, size_t *len);
on_err_t on_ota_commit_unpack(const uint8_t *in, size_t len, on_ota_commit_t *c);

/* Initialization & RAM storage binding */
void     on_ota_init(on_ota_ctx_t *ctx, uint32_t current_fw_version, uint8_t hw_class,
                     const on_ota_storage_ops_t *storage);
void     on_ota_ram_storage_init(on_ota_ram_storage_t *ram, uint8_t *backing_buf,
                                 size_t backing_cap, on_ota_storage_ops_t *ops_out);

/* Receiver API */
on_err_t on_ota_rx_on_meta(on_ota_rx_t *rx, const on_ota_meta_t *meta, uint16_t my_short_id);
on_err_t on_ota_rx_on_chunk(on_ota_rx_t *rx, const on_ota_chunk_t *chunk,
                            const uint8_t *auth_key_or_null);
on_err_t on_ota_rx_verify_image(on_ota_rx_t *rx, const uint8_t *auth_key_or_null);
void     on_ota_rx_build_status(const on_ota_rx_t *rx, uint16_t my_short_id,
                                on_ota_status_t *out_status);
on_err_t on_ota_rx_on_commit(on_ota_rx_t *rx, const on_ota_commit_t *commit, uint16_t my_short_id);
bool     on_ota_rx_step_commit(on_ota_rx_t *rx, uint32_t current_sf_seq);

/* Sender / Coordinator API */
on_err_t on_ota_tx_start_mem(on_ota_tx_t *tx, uint16_t ota_id, uint32_t fw_version,
                             uint16_t target_id, uint8_t hw_compat, uint8_t flags,
                             const uint8_t *image_buf, uint32_t image_size,
                             uint16_t chunk_size, const uint8_t *auth_key_or_null);

on_err_t on_ota_tx_start_reader(on_ota_tx_t *tx, uint16_t ota_id, uint32_t fw_version,
                                uint16_t target_id, uint8_t hw_compat, uint8_t flags,
                                uint32_t image_size, uint16_t chunk_size,
                                uint32_t precomputed_crc32,
                                const uint8_t precomputed_tag8[ON_OTA_AUTH_TAG_LEN],
                                on_err_t (*read_fn)(void *, uint32_t, uint8_t *, size_t),
                                void *read_ctx);

void     on_ota_tx_on_status(on_ota_tx_t *tx, const on_ota_status_t *st, uint32_t current_sf_seq);
bool     on_ota_tx_next_chunk(on_ota_tx_t *tx, on_ota_chunk_t *out_chunk);
bool     on_ota_tx_all_targets_ready(const on_ota_tx_t *tx, uint32_t min_targets);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_OTA_H */
