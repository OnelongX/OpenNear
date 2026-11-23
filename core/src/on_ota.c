#include "on_ota.h"
#include <string.h>

/* Little-endian helpers */
static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}
static uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}
static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ========================================================================== */
/* IEEE 802.3 CRC-32 & Streaming AES-128-CBC-MAC Image Authenticator          */
/* ========================================================================== */

uint32_t on_ota_crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    uint32_t c = ~crc;
    for (size_t i = 0u; i < len; ++i) {
        c ^= (uint32_t)data[i];
        for (int b = 0; b < 8; ++b) {
            c = (c & 1u) ? ((c >> 1) ^ 0xEDB88320UL) : (c >> 1);
        }
    }
    return ~c;
}

uint32_t on_ota_crc32(const uint8_t *data, size_t len) {
    return on_ota_crc32_update(0u, data, len);
}

typedef struct {
    uint8_t key[ON_SEC_KEY_LEN];
    uint8_t state[16];
    uint8_t buf[16];
    size_t  buf_len;
} ota_cbcmac_t;

static void cbcmac_init(ota_cbcmac_t *m, const uint8_t key[ON_SEC_KEY_LEN],
                        const on_ota_meta_t *meta) {
    memcpy(m->key, key, ON_SEC_KEY_LEN);
    memset(m->state, 0, 16u);
    memset(m->buf, 0, 16u);
    m->buf_len = 0u;

    /* Domain-separated header block binding ota_id, fw_version, total_size, crc32, hw_compat */
    uint8_t hdr_blk[16];
    hdr_blk[0] = 'O'; hdr_blk[1] = 'N'; hdr_blk[2] = 'O'; hdr_blk[3] = 'T';
    put_u16(hdr_blk + 4, meta->ota_id);
    put_u32(hdr_blk + 6, meta->fw_version);
    put_u32(hdr_blk + 10, meta->total_size);
    hdr_blk[14] = meta->hw_compat;
    hdr_blk[15] = (uint8_t)(meta->crc32 & 0xFFu);
    on_aes128_encrypt_block(m->key, hdr_blk, m->state);
}

static void cbcmac_update(ota_cbcmac_t *m, const uint8_t *data, size_t len) {
    size_t pos = 0u;
    while (pos < len) {
        size_t take = 16u - m->buf_len;
        if (take > len - pos) take = len - pos;
        memcpy(m->buf + m->buf_len, data + pos, take);
        m->buf_len += take;
        pos += take;
        if (m->buf_len == 16u) {
            uint8_t x[16];
            for (int i = 0; i < 16; ++i) x[i] = m->state[i] ^ m->buf[i];
            on_aes128_encrypt_block(m->key, x, m->state);
            m->buf_len = 0u;
        }
    }
}

static void cbcmac_finish(ota_cbcmac_t *m, uint32_t crc32, uint8_t tag8_out[ON_OTA_AUTH_TAG_LEN]) {
    if (m->buf_len > 0u) {
        memset(m->buf + m->buf_len, 0, 16u - m->buf_len);
        uint8_t x[16];
        for (int i = 0; i < 16; ++i) x[i] = m->state[i] ^ m->buf[i];
        on_aes128_encrypt_block(m->key, x, m->state);
        m->buf_len = 0u;
    }
    /* Finalize with full CRC-32 mix block */
    uint8_t fin[16];
    memset(fin, 0xA5, sizeof(fin));
    put_u32(fin, crc32);
    for (int i = 0; i < 16; ++i) fin[i] ^= m->state[i];
    on_aes128_encrypt_block(m->key, fin, m->state);
    memcpy(tag8_out, m->state, ON_OTA_AUTH_TAG_LEN);
}

void on_ota_compute_auth_tag(const uint8_t key[ON_SEC_KEY_LEN],
                             const on_ota_meta_t *meta,
                             const uint8_t *image_data,
                             uint8_t tag8_out[ON_OTA_AUTH_TAG_LEN]) {
    if (!key || !meta || !tag8_out) return;
    ota_cbcmac_t mac;
    cbcmac_init(&mac, key, meta);
    if (image_data && meta->total_size > 0u) {
        cbcmac_update(&mac, image_data, meta->total_size);
    }
    cbcmac_finish(&mac, meta->crc32, tag8_out);
}

on_err_t on_ota_compute_auth_tag_reader(const uint8_t key[ON_SEC_KEY_LEN],
                                        const on_ota_meta_t *meta,
                                        on_err_t (*read_fn)(void *, uint32_t, uint8_t *, size_t),
                                        void *read_ctx,
                                        uint8_t tag8_out[ON_OTA_AUTH_TAG_LEN]) {
    if (!key || !meta || !read_fn || !tag8_out) return ON_ERR_INVALID_ARG;
    ota_cbcmac_t mac;
    cbcmac_init(&mac, key, meta);
    uint8_t buf[128];
    uint32_t off = 0u;
    while (off < meta->total_size) {
        size_t n = meta->total_size - off;
        if (n > sizeof(buf)) n = sizeof(buf);
        on_err_t rc = read_fn(read_ctx, off, buf, n);
        if (rc != ON_OK) return rc;
        cbcmac_update(&mac, buf, n);
        off += (uint32_t)n;
    }
    cbcmac_finish(&mac, meta->crc32, tag8_out);
    return ON_OK;
}

/* ========================================================================== */
/* Wire Codecs                                                                */
/* ========================================================================== */

on_err_t on_ota_meta_pack(const on_ota_meta_t *m, uint8_t *out, size_t cap, size_t *len) {
    if (!m || !out || !len) return ON_ERR_INVALID_ARG;
    if (cap < ON_OTA_META_WIRE_LEN) return ON_ERR_TOO_LONG;
    put_u16(out + 0, m->ota_id);
    put_u32(out + 2, m->fw_version);
    put_u32(out + 6, m->total_size);
    put_u16(out + 10, m->chunk_size);
    put_u16(out + 12, m->total_chunks);
    put_u32(out + 14, m->crc32);
    memcpy(out + 18, m->auth_tag, ON_OTA_AUTH_TAG_LEN);
    put_u16(out + 26, m->target_id & ON_SHORT_ID_MASK);
    out[28] = m->flags;
    out[29] = m->hw_compat;
    *len = ON_OTA_META_WIRE_LEN;
    return ON_OK;
}

on_err_t on_ota_meta_unpack(const uint8_t *in, size_t len, on_ota_meta_t *m) {
    if (!in || !m) return ON_ERR_INVALID_ARG;
    if (len < ON_OTA_META_WIRE_LEN) return ON_ERR_TOO_SHORT;
    m->ota_id       = get_u16(in + 0);
    m->fw_version   = get_u32(in + 2);
    m->total_size   = get_u32(in + 6);
    m->chunk_size   = get_u16(in + 10);
    m->total_chunks = get_u16(in + 12);
    m->crc32        = get_u32(in + 14);
    memcpy(m->auth_tag, in + 18, ON_OTA_AUTH_TAG_LEN);
    m->target_id    = get_u16(in + 26) & ON_SHORT_ID_MASK;
    m->flags        = in[28];
    m->hw_compat    = in[29];
    return ON_OK;
}

on_err_t on_ota_chunk_pack(const on_ota_chunk_t *c, uint8_t *out, size_t cap, size_t *len) {
    if (!c || !out || !len) return ON_ERR_INVALID_ARG;
    if (c->data_len > ON_OTA_MAX_CHUNK_LEN) return ON_ERR_TOO_LONG;
    size_t total = ON_OTA_CHUNK_HDR_LEN + (size_t)c->data_len;
    if (cap < total) return ON_ERR_TOO_LONG;
    put_u16(out + 0, c->ota_id);
    put_u16(out + 2, c->chunk_idx);
    out[4] = c->data_len;
    out[5] = c->flags;
    if (c->data_len > 0u) {
        memcpy(out + ON_OTA_CHUNK_HDR_LEN, c->data, c->data_len);
    }
    *len = total;
    return ON_OK;
}

on_err_t on_ota_chunk_unpack(const uint8_t *in, size_t len, on_ota_chunk_t *c) {
    if (!in || !c) return ON_ERR_INVALID_ARG;
    if (len < ON_OTA_CHUNK_HDR_LEN) return ON_ERR_TOO_SHORT;
    c->ota_id    = get_u16(in + 0);
    c->chunk_idx = get_u16(in + 2);
    c->data_len  = in[4];
    c->flags     = in[5];
    if (c->data_len > ON_OTA_MAX_CHUNK_LEN || len < ON_OTA_CHUNK_HDR_LEN + (size_t)c->data_len) {
        return ON_ERR_TOO_SHORT;
    }
    if (c->data_len > 0u) {
        memcpy(c->data, in + ON_OTA_CHUNK_HDR_LEN, c->data_len);
    }
    return ON_OK;
}

on_err_t on_ota_status_pack(const on_ota_status_t *s, uint8_t *out, size_t cap, size_t *len) {
    if (!s || !out || !len) return ON_ERR_INVALID_ARG;
    if (cap < ON_OTA_STATUS_WIRE_LEN) return ON_ERR_TOO_LONG;
    put_u16(out + 0, s->ota_id);
    put_u16(out + 2, s->node_id & ON_SHORT_ID_MASK);
    out[4] = s->state;
    out[5] = s->err_code;
    put_u16(out + 6, s->next_missing);
    put_u16(out + 8, s->rx_chunks);
    put_u32(out + 10, s->nack_bitmap);
    *len = ON_OTA_STATUS_WIRE_LEN;
    return ON_OK;
}

on_err_t on_ota_status_unpack(const uint8_t *in, size_t len, on_ota_status_t *s) {
    if (!in || !s) return ON_ERR_INVALID_ARG;
    if (len < ON_OTA_STATUS_WIRE_LEN) return ON_ERR_TOO_SHORT;
    s->ota_id       = get_u16(in + 0);
    s->node_id      = get_u16(in + 2) & ON_SHORT_ID_MASK;
    s->state        = in[4];
    s->err_code     = in[5];
    s->next_missing = get_u16(in + 6);
    s->rx_chunks    = get_u16(in + 8);
    s->nack_bitmap  = get_u32(in + 10);
    return ON_OK;
}

on_err_t on_ota_commit_pack(const on_ota_commit_t *c, uint8_t *out, size_t cap, size_t *len) {
    if (!c || !out || !len) return ON_ERR_INVALID_ARG;
    if (cap < ON_OTA_COMMIT_WIRE_LEN) return ON_ERR_TOO_LONG;
    put_u16(out + 0, c->ota_id);
    put_u32(out + 2, c->commit_sf_seq);
    put_u32(out + 6, c->crc32);
    put_u16(out + 10, c->target_id & ON_SHORT_ID_MASK);
    out[12] = c->flags;
    *len = ON_OTA_COMMIT_WIRE_LEN;
    return ON_OK;
}

on_err_t on_ota_commit_unpack(const uint8_t *in, size_t len, on_ota_commit_t *c) {
    if (!in || !c) return ON_ERR_INVALID_ARG;
    if (len < ON_OTA_COMMIT_WIRE_LEN) return ON_ERR_TOO_SHORT;
    c->ota_id        = get_u16(in + 0);
    c->commit_sf_seq = get_u32(in + 2);
    c->crc32         = get_u32(in + 6);
    c->target_id     = get_u16(in + 10) & ON_SHORT_ID_MASK;
    c->flags         = in[12];
    return ON_OK;
}

/* ========================================================================== */
/* Initialization & RAM Storage Backend                                       */
/* ========================================================================== */

void on_ota_init(on_ota_ctx_t *ctx, uint32_t current_fw_version, uint8_t hw_class,
                 const on_ota_storage_ops_t *storage) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->rx.current_fw_version = current_fw_version;
    ctx->rx.hw_class           = hw_class;
    ctx->rx.state              = ON_OTA_STATE_IDLE;
    ctx->rx.last_err           = ON_OTA_ERR_NONE;
    if (storage) {
        ctx->rx.storage = *storage;
    }
}

static on_err_t ram_begin(void *ctx, uint32_t total_size, uint32_t fw_version) {
    (void)fw_version;
    on_ota_ram_storage_t *r = (on_ota_ram_storage_t *)ctx;
    if (!r || !r->buf || total_size > r->cap) return ON_ERR_NO_MEM;
    memset(r->buf, 0xFF, total_size);
    r->written_size = total_size;
    r->begun        = true;
    r->finalized    = false;
    r->applied      = false;
    return ON_OK;
}

static on_err_t ram_write(void *ctx, uint32_t offset, const uint8_t *data, size_t len) {
    on_ota_ram_storage_t *r = (on_ota_ram_storage_t *)ctx;
    if (!r || !r->begun || !data || (size_t)offset + len > r->cap) return ON_ERR_INVALID_ARG;
    memcpy(r->buf + offset, data, len);
    return ON_OK;
}

static on_err_t ram_read(void *ctx, uint32_t offset, uint8_t *out, size_t len) {
    on_ota_ram_storage_t *r = (on_ota_ram_storage_t *)ctx;
    if (!r || !r->buf || !out || (size_t)offset + len > r->cap) return ON_ERR_INVALID_ARG;
    memcpy(out, r->buf + offset, len);
    return ON_OK;
}

static on_err_t ram_finalize(void *ctx, bool verified_ok) {
    on_ota_ram_storage_t *r = (on_ota_ram_storage_t *)ctx;
    if (!r) return ON_ERR_INVALID_ARG;
    r->finalized = verified_ok;
    return verified_ok ? ON_OK : ON_ERR_DECODE_FAIL;
}

static on_err_t ram_apply(void *ctx, uint32_t fw_version) {
    on_ota_ram_storage_t *r = (on_ota_ram_storage_t *)ctx;
    if (!r || !r->finalized) return ON_ERR_NOT_READY;
    r->applied            = true;
    r->applied_fw_version = fw_version;
    return ON_OK;
}

static on_err_t ram_confirm(void *ctx) {
    on_ota_ram_storage_t *r = (on_ota_ram_storage_t *)ctx;
    if (!r) return ON_ERR_INVALID_ARG;
    r->boot_confirmed = true;
    return ON_OK;
}

void on_ota_ram_storage_init(on_ota_ram_storage_t *ram, uint8_t *backing_buf,
                             size_t backing_cap, on_ota_storage_ops_t *ops_out) {
    if (!ram || !ops_out) return;
    memset(ram, 0, sizeof(*ram));
    ram->buf = backing_buf;
    ram->cap = backing_cap;
    ops_out->user_ctx         = ram;
    ops_out->begin            = ram_begin;
    ops_out->write_chunk      = ram_write;
    ops_out->read_chunk       = ram_read;
    ops_out->finalize         = ram_finalize;
    ops_out->apply_and_reboot = ram_apply;
    ops_out->confirm_boot     = ram_confirm;
}

/* ========================================================================== */
/* Receiver State Machine                                                     */
/* ========================================================================== */

static bool target_matches(uint16_t target_id, uint16_t my_short_id) {
    return (target_id == ON_SHORT_ID_BROADCAST) ||
           (my_short_id != ON_SHORT_ID_UNASSIGNED && target_id == my_short_id);
}

on_err_t on_ota_rx_on_meta(on_ota_rx_t *rx, const on_ota_meta_t *meta, uint16_t my_short_id) {
    if (!rx || !meta) return ON_ERR_INVALID_ARG;
    if (!target_matches(meta->target_id, my_short_id)) return ON_OK;

    if (meta->hw_compat != 0u && rx->hw_class != 0u && meta->hw_compat != rx->hw_class) {
        return ON_OK; /* Manifest meant for a different hardware family */
    }
    if ((meta->flags & ON_OTA_FLAG_ABORT) != 0u) {
        rx->state        = ON_OTA_STATE_IDLE;
        rx->last_err     = ON_OTA_ERR_ABORTED;
        rx->status_dirty = true;
        return ON_OK;
    }

    /* Duplicate manifest for an already active or completed OTA session -> just refresh status */
    if ((rx->state == ON_OTA_STATE_RECEIVING || rx->state == ON_OTA_STATE_READY ||
         rx->state == ON_OTA_STATE_COMMITTED) && rx->meta.ota_id == meta->ota_id &&
        rx->meta.crc32 == meta->crc32) {
        rx->status_dirty = true;
        return ON_OK;
    }

    if (meta->chunk_size == 0u || meta->chunk_size > ON_OTA_MAX_CHUNK_LEN ||
        meta->total_size == 0u || meta->total_chunks == 0u) {
        rx->state        = ON_OTA_STATE_FAILED;
        rx->last_err     = ON_OTA_ERR_BAD_PARAM;
        rx->status_dirty = true;
        return ON_ERR_INVALID_ARG;
    }
    uint32_t expected_chunks = (meta->total_size + meta->chunk_size - 1u) / meta->chunk_size;
    if (expected_chunks != meta->total_chunks) {
        rx->state        = ON_OTA_STATE_FAILED;
        rx->last_err     = ON_OTA_ERR_BAD_PARAM;
        rx->status_dirty = true;
        return ON_ERR_INVALID_ARG;
    }

    /* Anti-downgrade version guard unless ON_OTA_FLAG_FORCE is set */
    if ((meta->flags & ON_OTA_FLAG_FORCE) == 0u && meta->fw_version <= rx->current_fw_version) {
        rx->state        = ON_OTA_STATE_FAILED;
        rx->last_err     = ON_OTA_ERR_DOWNGRADE;
        rx->meta         = *meta;
        rx->status_dirty = true;
        return ON_ERR_INVALID_ARG;
    }

    if (!rx->storage.begin || !rx->storage.write_chunk) {
        rx->state        = ON_OTA_STATE_FAILED;
        rx->last_err     = ON_OTA_ERR_STORAGE;
        rx->status_dirty = true;
        return ON_ERR_NOT_READY;
    }

    on_err_t rc = rx->storage.begin(rx->storage.user_ctx, meta->total_size, meta->fw_version);
    if (rc != ON_OK) {
        rx->state        = ON_OTA_STATE_FAILED;
        rx->last_err     = (rc == ON_ERR_NO_MEM) ? ON_OTA_ERR_TOO_LARGE : ON_OTA_ERR_STORAGE;
        rx->meta         = *meta;
        rx->status_dirty = true;
        return rc;
    }

    rx->meta          = *meta;
    rx->base_chunk    = 0u;
    rx->win_rx_bitmap = 0u;
    rx->rx_chunks     = 0u;
    rx->dup_chunks    = 0u;
    rx->state         = ON_OTA_STATE_RECEIVING;
    rx->last_err      = ON_OTA_ERR_NONE;
    rx->status_dirty  = true;
    return ON_OK;
}

on_err_t on_ota_rx_verify_image(on_ota_rx_t *rx, const uint8_t *auth_key_or_null) {
    if (!rx || !rx->storage.read_chunk) return ON_ERR_NOT_READY;
    rx->state = ON_OTA_STATE_VERIFYING;

    uint32_t crc = 0u;
    ota_cbcmac_t mac;
    if (auth_key_or_null) {
        cbcmac_init(&mac, auth_key_or_null, &rx->meta);
    }

    uint8_t buf[ON_OTA_MAX_CHUNK_LEN];
    uint32_t off = 0u;
    while (off < rx->meta.total_size) {
        size_t n = rx->meta.total_size - off;
        if (n > sizeof(buf)) n = sizeof(buf);
        on_err_t rc = rx->storage.read_chunk(rx->storage.user_ctx, off, buf, n);
        if (rc != ON_OK) {
            rx->state    = ON_OTA_STATE_FAILED;
            rx->last_err = ON_OTA_ERR_STORAGE;
            if (rx->storage.finalize) rx->storage.finalize(rx->storage.user_ctx, false);
            return rc;
        }
        crc = on_ota_crc32_update(crc, buf, n);
        if (auth_key_or_null) {
            cbcmac_update(&mac, buf, n);
        }
        off += (uint32_t)n;
    }

    if (crc != rx->meta.crc32) {
        rx->state    = ON_OTA_STATE_FAILED;
        rx->last_err = ON_OTA_ERR_CRC_MISMATCH;
        if (rx->storage.finalize) rx->storage.finalize(rx->storage.user_ctx, false);
        rx->status_dirty = true;
        return ON_ERR_PAY_CRC;
    }

    if (auth_key_or_null) {
        uint8_t calc_tag[ON_OTA_AUTH_TAG_LEN];
        cbcmac_finish(&mac, crc, calc_tag);
        uint8_t diff = 0u;
        for (size_t i = 0u; i < ON_OTA_AUTH_TAG_LEN; ++i) {
            diff |= (uint8_t)(calc_tag[i] ^ rx->meta.auth_tag[i]);
        }
        if (diff != 0u) {
            rx->state    = ON_OTA_STATE_FAILED;
            rx->last_err = ON_OTA_ERR_AUTH_FAIL;
            if (rx->storage.finalize) rx->storage.finalize(rx->storage.user_ctx, false);
            rx->status_dirty = true;
            return ON_ERR_DECODE_FAIL;
        }
    }

    if (rx->storage.finalize) {
        on_err_t frc = rx->storage.finalize(rx->storage.user_ctx, true);
        if (frc != ON_OK) {
            rx->state    = ON_OTA_STATE_FAILED;
            rx->last_err = ON_OTA_ERR_STORAGE;
            rx->status_dirty = true;
            return frc;
        }
    }

    rx->state        = ON_OTA_STATE_READY;
    rx->last_err     = ON_OTA_ERR_NONE;
    rx->status_dirty = true;
    return ON_OK;
}

on_err_t on_ota_rx_on_chunk(on_ota_rx_t *rx, const on_ota_chunk_t *chunk,
                            const uint8_t *auth_key_or_null) {
    if (!rx || !chunk) return ON_ERR_INVALID_ARG;
    if (rx->state != ON_OTA_STATE_RECEIVING || chunk->ota_id != rx->meta.ota_id) {
        return ON_OK;
    }
    if (chunk->chunk_idx >= rx->meta.total_chunks) {
        return ON_ERR_INVALID_ARG;
    }
    if (chunk->chunk_idx < rx->base_chunk) {
        rx->dup_chunks++;
        return ON_OK;
    }
    uint32_t win_off = (uint32_t)(chunk->chunk_idx - rx->base_chunk);
    if (win_off >= ON_OTA_WINDOW_CHUNKS) {
        /* Outside 32-chunk sliding window: ask sender to fill earlier missing chunks first */
        rx->status_dirty = true;
        return ON_OK;
    }
    if ((rx->win_rx_bitmap & (1UL << win_off)) != 0u) {
        rx->dup_chunks++;
        return ON_OK;
    }

    uint32_t byte_off = (uint32_t)chunk->chunk_idx * (uint32_t)rx->meta.chunk_size;
    uint32_t expected_len = rx->meta.chunk_size;
    if (byte_off + expected_len > rx->meta.total_size) {
        expected_len = rx->meta.total_size - byte_off;
    }
    if (chunk->data_len != expected_len) {
        return ON_ERR_INVALID_ARG;
    }

    on_err_t rc = rx->storage.write_chunk(rx->storage.user_ctx, byte_off, chunk->data, chunk->data_len);
    if (rc != ON_OK) {
        rx->state        = ON_OTA_STATE_FAILED;
        rx->last_err     = ON_OTA_ERR_STORAGE;
        rx->status_dirty = true;
        return rc;
    }

    rx->win_rx_bitmap |= (1UL << win_off);
    rx->rx_chunks++;
    while (rx->base_chunk < rx->meta.total_chunks && (rx->win_rx_bitmap & 1UL) != 0u) {
        rx->win_rx_bitmap >>= 1u;
        rx->base_chunk++;
    }

    /* Trigger periodic NACK/progress status every 8 chunks or on gap/completion */
    if (rx->base_chunk == rx->meta.total_chunks) {
        return on_ota_rx_verify_image(rx, auth_key_or_null);
    }
    if ((rx->rx_chunks & 0x07u) == 0u || win_off > 0u) {
        rx->status_dirty = true;
    }
    return ON_OK;
}

void on_ota_rx_build_status(const on_ota_rx_t *rx, uint16_t my_short_id,
                            on_ota_status_t *out_status) {
    if (!rx || !out_status) return;
    memset(out_status, 0, sizeof(*out_status));
    out_status->ota_id       = rx->meta.ota_id;
    out_status->node_id      = my_short_id & ON_SHORT_ID_MASK;
    out_status->state        = (uint8_t)rx->state;
    out_status->err_code     = (uint8_t)rx->last_err;
    out_status->next_missing = rx->base_chunk;
    out_status->rx_chunks    = rx->rx_chunks;

    uint32_t nack = 0u;
    if (rx->state == ON_OTA_STATE_RECEIVING) {
        for (uint32_t k = 0u; k < ON_OTA_WINDOW_CHUNKS; ++k) {
            if ((uint32_t)rx->base_chunk + k >= rx->meta.total_chunks) break;
            if ((rx->win_rx_bitmap & (1UL << k)) == 0u) {
                nack |= (1UL << k);
            }
        }
    }
    out_status->nack_bitmap = nack;
}

on_err_t on_ota_rx_on_commit(on_ota_rx_t *rx, const on_ota_commit_t *commit, uint16_t my_short_id) {
    if (!rx || !commit) return ON_ERR_INVALID_ARG;
    if (!target_matches(commit->target_id, my_short_id)) return ON_OK;
    if (commit->ota_id != rx->meta.ota_id) return ON_OK;

    if ((commit->flags & ON_OTA_FLAG_ABORT) != 0u) {
        rx->state        = ON_OTA_STATE_IDLE;
        rx->last_err     = ON_OTA_ERR_ABORTED;
        rx->status_dirty = true;
        return ON_OK;
    }
    if (rx->state != ON_OTA_STATE_READY || commit->crc32 != rx->meta.crc32) {
        rx->status_dirty = true;
        return ON_ERR_NOT_READY;
    }

    rx->commit_sf_seq = commit->commit_sf_seq;
    rx->commit_flags  = commit->flags;
    rx->state         = ON_OTA_STATE_COMMITTED;
    rx->status_dirty  = true;
    return ON_OK;
}

bool on_ota_rx_step_commit(on_ota_rx_t *rx, uint32_t current_sf_seq) {
    if (!rx || rx->state != ON_OTA_STATE_COMMITTED) return false;
    if (current_sf_seq < rx->commit_sf_seq) return false;

    if ((rx->commit_flags & ON_OTA_FLAG_VERIFY_ONLY) != 0u) {
        rx->state = ON_OTA_STATE_APPLIED;
        return true;
    }
    if (rx->storage.apply_and_reboot) {
        if (rx->storage.apply_and_reboot(rx->storage.user_ctx, rx->meta.fw_version) == ON_OK) {
            rx->current_fw_version = rx->meta.fw_version;
            rx->applied_version    = rx->meta.fw_version;
            rx->state              = ON_OTA_STATE_APPLIED;
            return true;
        }
        rx->state    = ON_OTA_STATE_FAILED;
        rx->last_err = ON_OTA_ERR_STORAGE;
        return false;
    }
    rx->current_fw_version = rx->meta.fw_version;
    rx->applied_version    = rx->meta.fw_version;
    rx->state              = ON_OTA_STATE_APPLIED;
    return true;
}

/* ========================================================================== */
/* Sender / Coordinator State Machine                                         */
/* ========================================================================== */

on_err_t on_ota_tx_start_mem(on_ota_tx_t *tx, uint16_t ota_id, uint32_t fw_version,
                             uint16_t target_id, uint8_t hw_compat, uint8_t flags,
                             const uint8_t *image_buf, uint32_t image_size,
                             uint16_t chunk_size, const uint8_t *auth_key_or_null) {
    if (!tx || !image_buf || image_size == 0u ||
        chunk_size == 0u || chunk_size > ON_OTA_MAX_CHUNK_LEN) {
        return ON_ERR_INVALID_ARG;
    }
    uint32_t total_chunks = (image_size + chunk_size - 1u) / chunk_size;
    if (total_chunks > 65535u) return ON_ERR_TOO_LONG;

    memset(tx, 0, sizeof(*tx));
    tx->active            = true;
    tx->image_buf         = image_buf;
    tx->meta.ota_id       = ota_id;
    tx->meta.fw_version   = fw_version;
    tx->meta.total_size   = image_size;
    tx->meta.chunk_size   = chunk_size;
    tx->meta.total_chunks = (uint16_t)total_chunks;
    tx->meta.crc32        = on_ota_crc32(image_buf, image_size);
    tx->meta.target_id    = target_id & ON_SHORT_ID_MASK;
    tx->meta.flags        = flags;
    tx->meta.hw_compat    = hw_compat;
    if (auth_key_or_null) {
        on_ota_compute_auth_tag(auth_key_or_null, &tx->meta, image_buf, tx->meta.auth_tag);
    }
    return ON_OK;
}

on_err_t on_ota_tx_start_reader(on_ota_tx_t *tx, uint16_t ota_id, uint32_t fw_version,
                                uint16_t target_id, uint8_t hw_compat, uint8_t flags,
                                uint32_t image_size, uint16_t chunk_size,
                                uint32_t precomputed_crc32,
                                const uint8_t precomputed_tag8[ON_OTA_AUTH_TAG_LEN],
                                on_err_t (*read_fn)(void *, uint32_t, uint8_t *, size_t),
                                void *read_ctx) {
    if (!tx || !read_fn || image_size == 0u ||
        chunk_size == 0u || chunk_size > ON_OTA_MAX_CHUNK_LEN) {
        return ON_ERR_INVALID_ARG;
    }
    uint32_t total_chunks = (image_size + chunk_size - 1u) / chunk_size;
    if (total_chunks > 65535u) return ON_ERR_TOO_LONG;

    memset(tx, 0, sizeof(*tx));
    tx->active            = true;
    tx->read_image        = read_fn;
    tx->read_ctx          = read_ctx;
    tx->meta.ota_id       = ota_id;
    tx->meta.fw_version   = fw_version;
    tx->meta.total_size   = image_size;
    tx->meta.chunk_size   = chunk_size;
    tx->meta.total_chunks = (uint16_t)total_chunks;
    tx->meta.crc32        = precomputed_crc32;
    tx->meta.target_id    = target_id & ON_SHORT_ID_MASK;
    tx->meta.flags        = flags;
    tx->meta.hw_compat    = hw_compat;
    if (precomputed_tag8) {
        memcpy(tx->meta.auth_tag, precomputed_tag8, ON_OTA_AUTH_TAG_LEN);
    }
    return ON_OK;
}

void on_ota_tx_on_status(on_ota_tx_t *tx, const on_ota_status_t *st, uint32_t current_sf_seq) {
    if (!tx || !st || !tx->active || st->ota_id != tx->meta.ota_id) return;

    on_ota_target_track_t *slot = NULL;
    for (uint32_t i = 0u; i < ON_OTA_MAX_TARGETS; ++i) {
        if (tx->targets[i].active && tx->targets[i].node_id == st->node_id) {
            slot = &tx->targets[i];
            break;
        }
    }
    if (!slot) {
        for (uint32_t i = 0u; i < ON_OTA_MAX_TARGETS; ++i) {
            if (!tx->targets[i].active) {
                slot = &tx->targets[i];
                slot->active  = true;
                slot->node_id = st->node_id;
                break;
            }
        }
    }
    if (slot) {
        slot->state          = st->state;
        slot->err_code       = st->err_code;
        slot->next_missing   = st->next_missing;
        slot->rx_chunks      = st->rx_chunks;
        slot->nack_bitmap    = st->nack_bitmap;
        slot->last_status_sf = current_sf_seq;
    }

    /* Schedule selective-repeat retransmissions for already-transmitted chunks reported missing */
    if (st->state == ON_OTA_STATE_RECEIVING && st->nack_bitmap != 0u) {
        uint32_t mask_sent = 0u;
        for (uint32_t k = 0u; k < ON_OTA_WINDOW_CHUNKS; ++k) {
            uint32_t idx = (uint32_t)st->next_missing + k;
            if (idx < tx->tx_cursor && ((st->nack_bitmap & (1UL << k)) != 0u)) {
                mask_sent |= (1UL << k);
            }
        }
        if (mask_sent != 0u) {
            if (tx->retx_bitmap == 0u || st->next_missing < tx->retx_base) {
                tx->retx_base   = st->next_missing;
                tx->retx_bitmap = mask_sent;
            } else if (st->next_missing == tx->retx_base) {
                tx->retx_bitmap |= mask_sent;
            } else {
                uint32_t shift = (uint32_t)(st->next_missing - tx->retx_base);
                if (shift < 32u) {
                    tx->retx_bitmap |= (mask_sent << shift);
                }
            }
        }
    }
}

static bool load_chunk_bytes(const on_ota_tx_t *tx, uint16_t chunk_idx, on_ota_chunk_t *out) {
    uint32_t off = (uint32_t)chunk_idx * (uint32_t)tx->meta.chunk_size;
    if (off >= tx->meta.total_size) return false;
    uint32_t len = tx->meta.chunk_size;
    if (off + len > tx->meta.total_size) len = tx->meta.total_size - off;

    out->ota_id    = tx->meta.ota_id;
    out->chunk_idx = chunk_idx;
    out->data_len  = (uint8_t)len;
    out->flags     = 0u;

    if (tx->image_buf) {
        memcpy(out->data, tx->image_buf + off, len);
        return true;
    }
    if (tx->read_image) {
        return tx->read_image(tx->read_ctx, off, out->data, len) == ON_OK;
    }
    return false;
}

bool on_ota_tx_next_chunk(on_ota_tx_t *tx, on_ota_chunk_t *out_chunk) {
    if (!tx || !out_chunk || !tx->active) return false;

    /* 1. Prioritize selective-repeat retransmissions requested by target NACK bitmaps */
    if (tx->retx_bitmap != 0u) {
        for (uint32_t k = 0u; k < ON_OTA_WINDOW_CHUNKS; ++k) {
            if ((tx->retx_bitmap & (1UL << k)) != 0u) {
                tx->retx_bitmap &= ~(1UL << k);
                uint16_t idx = (uint16_t)(tx->retx_base + k);
                if (idx < tx->meta.total_chunks && load_chunk_bytes(tx, idx, out_chunk)) {
                    out_chunk->flags = 0x01u; /* Retransmission flag */
                    tx->chunks_tx_total++;
                    tx->chunks_retx_total++;
                    return true;
                }
            }
        }
    }

    /* 2. Respect receiver 32-chunk sliding window flow control */
    uint16_t min_base = tx->tx_cursor;
    bool has_receiving_target = false;
    for (uint32_t i = 0u; i < ON_OTA_MAX_TARGETS; ++i) {
        if (tx->targets[i].active && tx->targets[i].state == ON_OTA_STATE_RECEIVING) {
            if (!has_receiving_target || tx->targets[i].next_missing < min_base) {
                min_base = tx->targets[i].next_missing;
            }
            has_receiving_target = true;
        }
    }
    if (has_receiving_target && tx->tx_cursor >= min_base + ON_OTA_WINDOW_CHUNKS) {
        /* Window full: proactively resend slowest target's base chunk to unblock */
        if (min_base < tx->meta.total_chunks && load_chunk_bytes(tx, min_base, out_chunk)) {
            out_chunk->flags = 0x01u;
            tx->chunks_tx_total++;
            tx->chunks_retx_total++;
            return true;
        }
        return false;
    }

    /* 3. Stream next fresh sequential chunk */
    if (tx->tx_cursor < tx->meta.total_chunks) {
        uint16_t idx = tx->tx_cursor++;
        if (load_chunk_bytes(tx, idx, out_chunk)) {
            tx->chunks_tx_total++;
            return true;
        }
    }

    /* 4. If all sequential chunks sent but a target is still missing chunks, resend its base chunk */
    if (has_receiving_target && min_base < tx->meta.total_chunks) {
        if (load_chunk_bytes(tx, min_base, out_chunk)) {
            out_chunk->flags = 0x01u;
            tx->chunks_tx_total++;
            tx->chunks_retx_total++;
            return true;
        }
    }

    return false;
}

bool on_ota_tx_all_targets_ready(const on_ota_tx_t *tx, uint32_t min_targets) {
    if (!tx || !tx->active) return false;
    uint32_t active_cnt = 0u;
    uint32_t ready_cnt  = 0u;
    for (uint32_t i = 0u; i < ON_OTA_MAX_TARGETS; ++i) {
        if (!tx->targets[i].active) continue;
        active_cnt++;
        if (tx->targets[i].state == ON_OTA_STATE_READY ||
            tx->targets[i].state == ON_OTA_STATE_COMMITTED ||
            tx->targets[i].state == ON_OTA_STATE_APPLIED) {
            ready_cnt++;
        }
    }
    return (active_cnt >= min_targets && ready_cnt == active_cnt && ready_cnt > 0u);
}
