#include "esp32_tls_transport.h"

#ifdef OPENNEAR_ENABLE_WIFI_GW
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <string.h>

static WiFiClient s_plain_mqtt_tcp;

#if defined(OPENNEAR_ENABLE_TLS) && (OPENNEAR_ENABLE_TLS != 0)
#include <WiFiClientSecure.h>

#define ON_ESP32_TLS_MAX_CA_LEN 2560u

static WiFiClientSecure s_tls_mqtt_client;
static WiFiClientSecure s_tls_http_client;
static char             s_ca_pem[ON_ESP32_TLS_MAX_CA_LEN] = {0};
static bool             s_has_ca_pem = false;

static void apply_ca_config(WiFiClientSecure &cli) {
    cli.setHandshakeTimeout(8u);
    if (s_has_ca_pem && s_ca_pem[0] != '\0') {
        cli.setCACert(s_ca_pem);
    } else {
        /* Dev fallback when no Root CA PEM is provisioned yet */
        cli.setInsecure();
    }
}

bool on_esp32_tls_is_compiled(void) {
    return true;
}

void on_esp32_tls_init_from_nvs(void) {
    Preferences prefs;
    if (prefs.begin("opennear_gw", true)) {
        String ca = prefs.getString("ca_pem", "");
        prefs.end();
        if (ca.length() > 0 && ca.length() < ON_ESP32_TLS_MAX_CA_LEN) {
            memcpy(s_ca_pem, ca.c_str(), ca.length() + 1u);
            s_has_ca_pem = true;
        }
    }
#ifdef OPENNEAR_TLS_ROOT_CA_PEM
    if (!s_has_ca_pem) {
        size_t len = strlen(OPENNEAR_TLS_ROOT_CA_PEM);
        if (len > 0u && len < ON_ESP32_TLS_MAX_CA_LEN) {
            memcpy(s_ca_pem, OPENNEAR_TLS_ROOT_CA_PEM, len + 1u);
            s_has_ca_pem = true;
        }
    }
#endif
    apply_ca_config(s_tls_mqtt_client);
    apply_ca_config(s_tls_http_client);
    Serial.printf("[OpenNear TLS] MQTTS/HTTPS module armed (RootCA=%s)\n",
                  s_has_ca_pem ? "VERIFIED_PEM" : "INSECURE_DEV_FALLBACK");
}

bool on_esp32_tls_has_ca(void) {
    return s_has_ca_pem;
}

on_err_t on_esp32_tls_set_ca_pem(const char *pem_or_empty, size_t len) {
    if (len >= ON_ESP32_TLS_MAX_CA_LEN) return ON_ERR_TOO_LONG;

    if (!pem_or_empty || len == 0u) {
        s_ca_pem[0]  = '\0';
        s_has_ca_pem = false;
    } else {
        /* Basic sanity check for PEM header */
        if (strstr(pem_or_empty, "BEGIN CERTIFICATE") == nullptr) {
            return ON_ERR_INVALID_ARG;
        }
        memcpy(s_ca_pem, pem_or_empty, len);
        s_ca_pem[len] = '\0';
        s_has_ca_pem  = true;
    }

    Preferences prefs;
    if (prefs.begin("opennear_gw", false)) {
        prefs.putString("ca_pem", s_ca_pem);
        prefs.end();
    }
    s_tls_mqtt_client.stop();
    apply_ca_config(s_tls_mqtt_client);
    apply_ca_config(s_tls_http_client);
    Serial.printf("[OpenNear TLS] Root CA %s in NVS\n",
                  s_has_ca_pem ? "saved (strict verify enabled)" : "cleared");
    return ON_OK;
}

Client *on_esp32_tls_get_mqtt_client(bool use_tls) {
    if (use_tls) {
        apply_ca_config(s_tls_mqtt_client);
        return &s_tls_mqtt_client;
    }
    return &s_plain_mqtt_tcp;
}

void on_esp32_tls_stop_mqtt_clients(void) {
    if (s_plain_mqtt_tcp.connected())  s_plain_mqtt_tcp.stop();
    if (s_tls_mqtt_client.connected()) s_tls_mqtt_client.stop();
}

bool on_esp32_tls_http_begin(HTTPClient &http, const char *url) {
    if (!url || !url[0]) return false;
    if (strncmp(url, "https://", 8u) == 0) {
        apply_ca_config(s_tls_http_client);
        return http.begin(s_tls_http_client, url);
    }
    return http.begin(url);
}

#else /* OPENNEAR_ENABLE_TLS == 0 (Zero-overhead plain TCP/HTTP build) */

bool on_esp32_tls_is_compiled(void) {
    return false;
}

void on_esp32_tls_init_from_nvs(void) {}

bool on_esp32_tls_has_ca(void) {
    return false;
}

on_err_t on_esp32_tls_set_ca_pem(const char *pem_or_empty, size_t len) {
    (void)pem_or_empty;
    (void)len;
    return ON_ERR_NOT_READY;
}

Client *on_esp32_tls_get_mqtt_client(bool use_tls) {
    if (use_tls) return nullptr;
    return &s_plain_mqtt_tcp;
}

void on_esp32_tls_stop_mqtt_clients(void) {
    if (s_plain_mqtt_tcp.connected()) s_plain_mqtt_tcp.stop();
}

bool on_esp32_tls_http_begin(HTTPClient &http, const char *url) {
    if (!url || !url[0]) return false;
    if (strncmp(url, "https://", 8u) == 0) return false;
    return http.begin(url);
}

#endif /* OPENNEAR_ENABLE_TLS */
#endif /* OPENNEAR_ENABLE_WIFI_GW */
