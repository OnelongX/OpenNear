#ifndef OPENNEAR_CORE_H
#define OPENNEAR_CORE_H

/*
 * OpenNear Unified C11 SDK Entry Header (v7.3.0)
 *
 * Portable across Windows, macOS, Linux, ESP-IDF, Arduino/PlatformIO,
 * Zephyr, FreeRTOS, and bare-metal Cortex-M / RISC-V targets.
 * Zero dynamic allocation, zero third-party dependencies.
 */

#include "opennear_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "on_types.h"
#include "on_hal.h"
#include "on_frame.h"
#include "on_sync.h"
#include "on_superframe.h"
#include "on_sec.h"
#include "on_ecdh.h"
#include "on_afh.h"
#include "on_amc.h"
#include "on_sar.h"
#include "on_ranging.h"
#include "on_polar.h"
#include "on_ota.h"
#include "on_gateway.h"
#include "on_mesh.h"
#include "on_mqtt.h"
#include "on_locate.h"
#include "on_node.h"

static inline const char *on_sdk_version_str(void) {
    return OPENNEAR_SDK_VERSION_STR;
}

static inline uint32_t on_sdk_version_hex(void) {
    return OPENNEAR_SDK_VERSION_HEX;
}

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_CORE_H */
