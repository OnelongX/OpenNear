#ifndef OPENNEAR_ESP32_TLS_TRANSPORT_H
#define OPENNEAR_ESP32_TLS_TRANSPORT_H

#include "on_types.h"
#include <stddef.h>

#ifdef __cplusplus
class Client;
class HTTPClient;

/*
 * OpenNear v6.4 Optional MQTTS / HTTPS TLS Transport Module
 *
 * - Disabled by default (zero WiFiClientSecure / mbedTLS overhead in LAN-only builds).
 * - Enable with -DOPENNEAR_ENABLE_TLS=1 (e.g., [env:esp32_iot_mesh_gw_tls]).
 * - Supports Root CA PEM stored in NVS ("opennear_gw"/"ca_pem") or compile-time
 *   OPENNEAR_TLS_ROOT_CA_PEM for EMQX Cloud, AWS IoT, Aliyun IoT, and private brokers.
 */

bool     on_esp32_tls_is_compiled(void);
void     on_esp32_tls_init_from_nvs(void);
bool     on_esp32_tls_has_ca(void);
on_err_t on_esp32_tls_set_ca_pem(const char *pem_or_empty, size_t len);

/* Return active TCP/TLS Client for MQTT (plain WiFiClient or WiFiClientSecure when TLS enabled) */
Client  *on_esp32_tls_get_mqtt_client(bool use_tls);
void     on_esp32_tls_stop_mqtt_clients(void);

/* Begin an HTTP or HTTPS request (automatically selects WiFiClientSecure for https:// URLs) */
bool     on_esp32_tls_http_begin(HTTPClient &http, const char *url);
#endif

#endif /* OPENNEAR_ESP32_TLS_TRANSPORT_H */
