#include "esp32_ota_flash.h"
#include <Arduino.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"

typedef struct {
    const esp_partition_t *update_part;
    esp_ota_handle_t       handle;
    bool                   active;
    bool                   finalized;
    uint32_t               total_size;
} esp32_ota_flash_ctx_t;

static esp32_ota_flash_ctx_t s_ota_flash = { nullptr, 0, false, false, 0u };

static on_err_t esp32_ota_begin(void *ctx, uint32_t total_size, uint32_t fw_version) {
    (void)fw_version;
    auto *f = static_cast<esp32_ota_flash_ctx_t *>(ctx);
    if (!f) return ON_ERR_INVALID_ARG;

    if (f->active) {
        esp_ota_abort(f->handle);
        f->active = false;
    }

    const esp_partition_t *part = esp_ota_get_next_update_partition(nullptr);
    if (!part) return ON_ERR_NOT_READY;
    if (total_size > part->size) return ON_ERR_NO_MEM;

    esp_err_t err = esp_ota_begin(part, total_size, &f->handle);
    if (err != ESP_OK) return ON_ERR_NOT_READY;

    f->update_part = part;
    f->total_size  = total_size;
    f->active      = true;
    f->finalized   = false;
    return ON_OK;
}

static on_err_t esp32_ota_write(void *ctx, uint32_t offset, const uint8_t *data, size_t len) {
    auto *f = static_cast<esp32_ota_flash_ctx_t *>(ctx);
    if (!f || !f->active || !data) return ON_ERR_INVALID_ARG;
    esp_err_t err = esp_ota_write_with_offset(f->handle, data, len, offset);
    return (err == ESP_OK) ? ON_OK : ON_ERR_NOT_READY;
}

static on_err_t esp32_ota_read(void *ctx, uint32_t offset, uint8_t *out, size_t len) {
    auto *f = static_cast<esp32_ota_flash_ctx_t *>(ctx);
    if (!f || !f->update_part || !out) return ON_ERR_INVALID_ARG;
    esp_err_t err = esp_partition_read(f->update_part, offset, out, len);
    return (err == ESP_OK) ? ON_OK : ON_ERR_NOT_READY;
}

static on_err_t esp32_ota_finalize(void *ctx, bool verified_ok) {
    auto *f = static_cast<esp32_ota_flash_ctx_t *>(ctx);
    if (!f || !f->active) return ON_ERR_NOT_READY;

    f->active = false;
    if (!verified_ok) {
        esp_ota_abort(f->handle);
        f->finalized = false;
        return ON_ERR_DECODE_FAIL;
    }
    esp_err_t err = esp_ota_end(f->handle);
    f->finalized = (err == ESP_OK);
    return f->finalized ? ON_OK : ON_ERR_DECODE_FAIL;
}

static on_err_t esp32_ota_apply_and_reboot(void *ctx, uint32_t fw_version) {
    auto *f = static_cast<esp32_ota_flash_ctx_t *>(ctx);
    if (!f || !f->finalized || !f->update_part) return ON_ERR_NOT_READY;

    esp_err_t err = esp_ota_set_boot_partition(f->update_part);
    if (err != ESP_OK) return ON_ERR_NOT_READY;

    Serial.printf("[OpenNear OTA] Partition %s armed for fw v%u.%u.%u -> rebooting!\n",
                  f->update_part->label,
                  (unsigned)((fw_version >> 16) & 0xFFu),
                  (unsigned)((fw_version >> 8) & 0xFFu),
                  (unsigned)(fw_version & 0xFFu));
    delay(50);
    esp_restart();
    return ON_OK;
}

static on_err_t esp32_ota_confirm(void *ctx) {
    (void)ctx;
    on_esp32_ota_confirm_running_app();
    return ON_OK;
}

void on_esp32_ota_storage_init(on_ota_storage_ops_t *ops_out) {
    if (!ops_out) return;
    ops_out->user_ctx         = &s_ota_flash;
    ops_out->begin            = esp32_ota_begin;
    ops_out->write_chunk      = esp32_ota_write;
    ops_out->read_chunk       = esp32_ota_read;
    ops_out->finalize         = esp32_ota_finalize;
    ops_out->apply_and_reboot = esp32_ota_apply_and_reboot;
    ops_out->confirm_boot     = esp32_ota_confirm;
}

void on_esp32_ota_confirm_running_app(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (running && esp_ota_get_state_partition(running, &state) == ESP_OK) {
        if (state == ESP_OTA_IMG_PENDING_VERIFY) {
            esp_ota_mark_app_valid_cancel_rollback();
            Serial.printf("[OpenNear OTA] Boot confirmed on partition %s (rollback cancelled)\n",
                          running->label);
        }
    }
}

on_err_t on_esp32_ota_read_staged(void *ctx, uint32_t offset, uint8_t *out, size_t len) {
    (void)ctx;
    const esp_partition_t *part = esp_ota_get_next_update_partition(nullptr);
    if (!part || !out || offset + len > part->size) return ON_ERR_INVALID_ARG;
    return (esp_partition_read(part, offset, out, len) == ESP_OK) ? ON_OK : ON_ERR_NOT_READY;
}
