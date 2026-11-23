#ifndef OPENNEAR_MQTT_H
#define OPENNEAR_MQTT_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v6.0 Pure-C11 OASIS MQTT 3.1.1 Codec & Home Assistant Auto-Discovery
 *
 * Zero external library dependencies, zero heap allocations.
 * Compatible with Mosquitto, EMQX, Home Assistant, Aliyun IoT, and AWS IoT.
 *
 * Topics:
 *   Discovery (retained): homeassistant/sensor/opennear_<id>/config
 *   Node State (pub):     opennear/node/<id>/state
 *   Cluster Topo (pub):   opennear/cluster/topology
 *   Node Command (sub):   opennear/cmd/<id>   (or opennear/cmd/#)
 *   Cluster OTA (sub):    opennear/ota/trigger ("<target_hex>|<ver>|<http_url>")
 */

#define ON_MQTT_PKT_CONNECT   0x10u
#define ON_MQTT_PKT_CONNACK   0x20u
#define ON_MQTT_PKT_PUBLISH   0x30u
#define ON_MQTT_PKT_SUBSCRIBE 0x82u
#define ON_MQTT_PKT_SUBACK    0x90u
#define ON_MQTT_PKT_PINGREQ   0xC0u
#define ON_MQTT_PKT_PINGRESP  0xD0u

typedef struct {
    const char *topic;
    size_t      topic_len;
    const uint8_t *payload;
    size_t      payload_len;
    uint8_t     qos;
    bool        retain;
    size_t      consumed_bytes;
} on_mqtt_pub_view_t;

/* OASIS MQTT 3.1.1 packet encoders / decoders */
size_t on_mqtt_encode_connect(const char *client_id, const char *username,
                              const char *password, uint16_t keepalive_s,
                              uint8_t *out, size_t cap);

bool   on_mqtt_decode_connack(const uint8_t *in, size_t len, uint8_t *out_return_code);

size_t on_mqtt_encode_publish(const char *topic, const uint8_t *payload, size_t payload_len,
                              bool retain, uint8_t *out, size_t cap);

bool   on_mqtt_decode_publish(const uint8_t *in, size_t len, on_mqtt_pub_view_t *out);

/* TCP stream framer for ANY OASIS MQTT 3.1.1 packet (CONNACK/SUBACK/PINGRESP/PUBLISH):
 * returns  1 if a full packet of *out_pkt_len bytes is ready at in[0],
 *          0 if more TCP bytes are needed,
 *         -1 if the variable remaining-length header is malformed. */
int    on_mqtt_next_packet(const uint8_t *in, size_t len,
                           uint8_t *out_pkt_type, size_t *out_pkt_len);

size_t on_mqtt_encode_subscribe(uint16_t pkt_id, const char *topic_filter,
                                uint8_t *out, size_t cap);

size_t on_mqtt_encode_pingreq(uint8_t out[2]);

/* Home Assistant MQTT Auto-Discovery & OpenNear IoT Topic Builders */
bool   on_mqtt_format_ha_discovery(uint16_t short_id, uint8_t hop_depth, const char *role_name,
                                   char *topic_out, size_t topic_cap,
                                   char *json_out, size_t json_cap);

bool   on_mqtt_format_ha_discovery_mac(uint16_t short_id, const uint8_t mac48[6],
                                       uint8_t hop_depth, const char *role_name,
                                       char *topic_out, size_t topic_cap,
                                       char *json_out, size_t json_cap);

bool   on_mqtt_format_node_state(uint16_t short_id, uint16_t parent_id, uint8_t hop_depth,
                                 int16_t skew_ns, const uint8_t *data, uint8_t data_len,
                                 char *topic_out, size_t topic_cap,
                                 char *json_out, size_t json_cap);

bool   on_mqtt_format_node_state_mac(uint16_t short_id, const uint8_t mac48[6],
                                     uint16_t parent_id, uint8_t hop_depth,
                                     int16_t skew_ns, const uint8_t *data, uint8_t data_len,
                                     char *topic_out, size_t topic_cap,
                                     char *json_out, size_t json_cap);

/* Parse inbound MQTT topic ("opennear/cmd/0x100" or "opennear/ota/trigger") */
bool   on_mqtt_parse_cmd_topic(const char *topic, size_t topic_len,
                               uint16_t *out_target_id, bool *out_is_ota);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_MQTT_H */
