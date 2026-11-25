#ifndef OPENNEAR_CONFIG_H
#define OPENNEAR_CONFIG_H

/*
 * OpenNear SDK Compile-Time Configuration & Semantic Versioning (v7.3.0)
 *
 * Override any ON_CFG_* macro via compiler -D flags, CMake options,
 * PlatformIO build_flags, or ESP-IDF Kconfig (sdkconfig.h).
 */

#define OPENNEAR_SDK_VERSION_MAJOR 7u
#define OPENNEAR_SDK_VERSION_MINOR 3u
#define OPENNEAR_SDK_VERSION_PATCH 0u
#define OPENNEAR_SDK_VERSION_HEX   ((OPENNEAR_SDK_VERSION_MAJOR << 16) | \
                                    (OPENNEAR_SDK_VERSION_MINOR << 8)  | \
                                    (OPENNEAR_SDK_VERSION_PATCH))
#define OPENNEAR_SDK_VERSION_STR   "7.3.0"

/* Optional ESP-IDF sdkconfig.h bridge when built as an ESP-IDF component */
#if defined(ESP_PLATFORM) && defined(__has_include)
#  if __has_include("sdkconfig.h")
#    include "sdkconfig.h"
#  endif
#endif

/* Core security (AES-128-CCM AEAD + Beacon CBC-MAC + X25519 ECDH) */
#ifndef ON_CFG_SECURITY
#  ifdef CONFIG_OPENNEAR_ENABLE_SECURITY
#    define ON_CFG_SECURITY CONFIG_OPENNEAR_ENABLE_SECURITY
#  else
#    define ON_CFG_SECURITY 1
#  endif
#endif

/* Multi-hop Hybrid Mesh (3~6 hops + 256-node Epoch-TDMA) */
#ifndef ON_CFG_MESH
#  ifdef CONFIG_OPENNEAR_ENABLE_MESH
#    define ON_CFG_MESH CONFIG_OPENNEAR_ENABLE_MESH
#  else
#    define ON_CFG_MESH 1
#  endif
#endif

/* Multi-hop Authenticated Cluster OTA (A/B partition rollback) */
#ifndef ON_CFG_OTA
#  ifdef CONFIG_OPENNEAR_ENABLE_OTA
#    define ON_CFG_OTA CONFIG_OPENNEAR_ENABLE_OTA
#  else
#    define ON_CFG_OTA 1
#  endif
#endif

/* 2D Multi-Anchor Indoor Positioning, BLE Beacons, 2D Geofences & Wi-Fi CSI Radar */
#ifndef ON_CFG_LOCATE
#  ifdef CONFIG_OPENNEAR_ENABLE_LOCATE
#    define ON_CFG_LOCATE CONFIG_OPENNEAR_ENABLE_LOCATE
#  else
#    define ON_CFG_LOCATE 1
#  endif
#endif

#ifndef ON_CFG_BLE_BEACON
#  define ON_CFG_BLE_BEACON ON_CFG_LOCATE
#endif

#ifndef ON_CFG_WIFI_CSI
#  define ON_CFG_WIFI_CSI ON_CFG_LOCATE
#endif

#ifndef ON_CFG_GEOFENCE
#  define ON_CFG_GEOFENCE ON_CFG_LOCATE
#endif

/* Gateway services (Wi-Fi Web Portal, MQTT/Home Assistant, Optional TLS) */
#ifndef ON_CFG_GATEWAY_WEB
#  ifdef OPENNEAR_ENABLE_WIFI_GW
#    define ON_CFG_GATEWAY_WEB 1
#  elif defined(CONFIG_OPENNEAR_ENABLE_WIFI_GW)
#    define ON_CFG_GATEWAY_WEB CONFIG_OPENNEAR_ENABLE_WIFI_GW
#  else
#    define ON_CFG_GATEWAY_WEB 0
#  endif
#endif

#ifndef ON_CFG_GATEWAY_BLE
#  ifdef OPENNEAR_ENABLE_BLE_GW
#    define ON_CFG_GATEWAY_BLE 1
#  elif defined(CONFIG_OPENNEAR_ENABLE_BLE_GW)
#    define ON_CFG_GATEWAY_BLE CONFIG_OPENNEAR_ENABLE_BLE_GW
#  else
#    define ON_CFG_GATEWAY_BLE 0
#  endif
#endif

#ifndef ON_CFG_TLS
#  ifdef OPENNEAR_ENABLE_TLS
#    define ON_CFG_TLS 1
#  elif defined(CONFIG_OPENNEAR_ENABLE_TLS)
#    define ON_CFG_TLS CONFIG_OPENNEAR_ENABLE_TLS
#  else
#    define ON_CFG_TLS 0
#  endif
#endif

#endif /* OPENNEAR_CONFIG_H */
