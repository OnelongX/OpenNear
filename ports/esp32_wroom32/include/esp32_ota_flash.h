#ifndef OPENNEAR_ESP32_OTA_FLASH_H
#define OPENNEAR_ESP32_OTA_FLASH_H

#include "opennear_config.h"
#include "on_ota.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ESP32-WROOM-32 Native Dual-Partition (A/B) OTA Flash Backend
 *
 * Binds OpenNear's on_ota_storage_ops_t directly to ESP-IDF's esp_ota_ops:
 * - Out-of-order chunk writes via esp_ota_write_with_offset()
 * - Flash readback verification via esp_partition_read() (CRC-32 + AES-CBC-MAC)
 * - Atomic boot-partition switch via esp_ota_set_boot_partition() + esp_restart()
 * - Automatic rollback cancellation on healthy boot via esp_ota_mark_app_valid_cancel_rollback()
 */

/* Bind ESP32 A/B flash OTA callbacks into ops_out */
void     on_esp32_ota_storage_init(on_ota_storage_ops_t *ops_out);

/* Mark current running image valid (call after boot & first synced superframe) */
void     on_esp32_ota_confirm_running_app(void);

/* Read staged firmware bytes from the inactive OTA partition (for Primary G relay TX) */
on_err_t on_esp32_ota_read_staged(void *ctx, uint32_t offset, uint8_t *out, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_ESP32_OTA_FLASH_H */
