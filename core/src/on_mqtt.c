#include "on_mqtt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static size_t encode_rem_len(size_t rem, uint8_t out[4]) {
    size_t n = 0u;
    do {
        uint8_t byte = (uint8_t)(rem & 0x7Fu);
        rem >>= 7u;
        if (rem > 0u) byte |= 0x80u;
        out[n++] = byte;
    } while (rem > 0u && n < 4u);
    return n;
}

static bool decode_rem_len(const uint8_t *in, size_t len, size_t *out_rem, size_t *out_hdr_bytes) {
    size_t mult = 1u, val = 0u, i = 0u;
    while (i < len && i < 4u) {
        uint8_t b = in[i++];
        val += (size_t)(b & 0x7Fu) * mult;
        if ((b & 0x80u) == 0u) {
            *out_rem       = val;
            *out_hdr_bytes = i;
            return true;
        }
        mult <<= 7u;
    }
    return false;
}

static size_t put_utf8(uint8_t *dst, const char *s, size_t slen) {
    dst[0] = (uint8_t)((slen >> 8) & 0xFFu);
    dst[1] = (uint8_t)(slen & 0xFFu);
    if (slen > 0u) memcpy(dst + 2, s, slen);
    return 2u + slen;
}

size_t on_mqtt_encode_connect(const char *client_id, const char *username,
                              const char *password, uint16_t keepalive_s,
                              uint8_t *out, size_t cap) {
    if (!client_id || !out) return 0u;
    size_t cid_len  = strlen(client_id);
    size_t user_len = (username && username[0]) ? strlen(username) : 0u;
    size_t pass_len = (password && password[0] && user_len > 0u) ? strlen(password) : 0u;

    uint8_t flags = 0x02u; /* Clean Session */
    if (user_len > 0u) flags |= 0x80u;
    if (pass_len > 0u) flags |= 0x40u;

    size_t rem_len = 10u + (2u + cid_len) +
                     (user_len > 0u ? (2u + user_len) : 0u) +
                     (pass_len > 0u ? (2u + pass_len) : 0u);
    uint8_t rbuf[4];
    size_t rlen = encode_rem_len(rem_len, rbuf);
    if (1u + rlen + rem_len > cap) return 0u;

    size_t pos = 0u;
    out[pos++] = ON_MQTT_PKT_CONNECT;
    memcpy(out + pos, rbuf, rlen);
    pos += rlen;

    /* Protocol Name "MQTT", Level 4 (3.1.1), Connect Flags, Keep Alive */
    pos += put_utf8(out + pos, "MQTT", 4u);
    out[pos++] = 4u;
    out[pos++] = flags;
    out[pos++] = (uint8_t)((keepalive_s >> 8) & 0xFFu);
    out[pos++] = (uint8_t)(keepalive_s & 0xFFu);

    pos += put_utf8(out + pos, client_id, cid_len);
    if (user_len > 0u) pos += put_utf8(out + pos, username, user_len);
    if (pass_len > 0u) pos += put_utf8(out + pos, password, pass_len);
    return pos;
}

bool on_mqtt_decode_connack(const uint8_t *in, size_t len, uint8_t *out_return_code) {
    if (!in || len < 4u) return false;
    if (in[0] != ON_MQTT_PKT_CONNACK || in[1] != 2u) return false;
    if (out_return_code) *out_return_code = in[3];
    return true;
}

size_t on_mqtt_encode_publish(const char *topic, const uint8_t *payload, size_t payload_len,
                              bool retain, uint8_t *out, size_t cap) {
    if (!topic || !out) return 0u;
    size_t tlen = strlen(topic);
    size_t rem_len = 2u + tlen + payload_len;
    uint8_t rbuf[4];
    size_t rlen = encode_rem_len(rem_len, rbuf);
    if (1u + rlen + rem_len > cap) return 0u;

    size_t pos = 0u;
    out[pos++] = (uint8_t)(ON_MQTT_PKT_PUBLISH | (retain ? 0x01u : 0x00u));
    memcpy(out + pos, rbuf, rlen);
    pos += rlen;
    pos += put_utf8(out + pos, topic, tlen);
    if (payload && payload_len > 0u) {
        memcpy(out + pos, payload, payload_len);
        pos += payload_len;
    }
    return pos;
}

bool on_mqtt_decode_publish(const uint8_t *in, size_t len, on_mqtt_pub_view_t *out) {
    if (!in || !out || len < 4u) return false;
    if ((in[0] & 0xF0u) != ON_MQTT_PKT_PUBLISH) return false;

    size_t rem_len = 0u, rbytes = 0u;
    if (!decode_rem_len(in + 1, len - 1u, &rem_len, &rbytes)) return false;
    size_t total = 1u + rbytes + rem_len;
    if (total > len) return false;

    size_t pos = 1u + rbytes;
    if (pos + 2u > total) return false;
    size_t tlen = ((size_t)in[pos] << 8) | (size_t)in[pos + 1u];
    pos += 2u;
    if (pos + tlen > total) return false;

    uint8_t qos = (uint8_t)((in[0] >> 1) & 0x03u);
    const char *topic = (const char *)(in + pos);
    pos += tlen;
    if (qos > 0u) {
        if (pos + 2u > total) return false;
        pos += 2u; /* Skip 2-byte Packet Identifier for QoS 1/2 */
    }

    out->topic          = topic;
    out->topic_len      = tlen;
    out->payload        = in + pos;
    out->payload_len    = total - pos;
    out->qos            = qos;
    out->retain         = ((in[0] & 0x01u) != 0u);
    out->consumed_bytes = total;
    return true;
}

size_t on_mqtt_encode_subscribe(uint16_t pkt_id, const char *topic_filter,
                                uint8_t *out, size_t cap) {
    if (!topic_filter || !out) return 0u;
    size_t tlen = strlen(topic_filter);
    size_t rem_len = 2u + 2u + tlen + 1u;
    uint8_t rbuf[4];
    size_t rlen = encode_rem_len(rem_len, rbuf);
    if (1u + rlen + rem_len > cap) return 0u;

    size_t pos = 0u;
    out[pos++] = ON_MQTT_PKT_SUBSCRIBE;
    memcpy(out + pos, rbuf, rlen);
    pos += rlen;
    out[pos++] = (uint8_t)((pkt_id >> 8) & 0xFFu);
    out[pos++] = (uint8_t)(pkt_id & 0xFFu);
    pos += put_utf8(out + pos, topic_filter, tlen);
    out[pos++] = 0x00u; /* Requested QoS 0 */
    return pos;
}

size_t on_mqtt_encode_pingreq(uint8_t out[2]) {
    if (!out) return 0u;
    out[0] = ON_MQTT_PKT_PINGREQ;
    out[1] = 0x00u;
    return 2u;
}

/* ========================================================================== */
/* Home Assistant MQTT Auto-Discovery & OpenNear IoT Topic Formatters         */
/* ========================================================================== */

bool on_mqtt_format_ha_discovery(uint16_t short_id, uint8_t hop_depth, const char *role_name,
                                 char *topic_out, size_t topic_cap,
                                 char *json_out, size_t json_cap) {
    if (!topic_out || !json_out || topic_cap < 48u || json_cap < 256u) return false;
    const char *r = (role_name && role_name[0]) ? role_name : "NODE";
    int tn = snprintf(topic_out, topic_cap,
                      "homeassistant/sensor/opennear_%03X/config",
                      (unsigned)(short_id & ON_SHORT_ID_MASK));
    if (tn <= 0 || (size_t)tn >= topic_cap) return false;

    int jn = snprintf(json_out, json_cap,
                      "{\"name\":\"OpenNear %s 0x%03X (Hop %u)\","
                      "\"uniq_id\":\"opennear_%03X\","
                      "\"stat_t\":\"opennear/node/0x%03X/state\","
                      "\"cmd_t\":\"opennear/cmd/0x%03X\","
                      "\"val_tpl\":\"{{ value_json.data }}\","
                      "\"json_attr_t\":\"opennear/node/0x%03X/state\","
                      "\"dev\":{\"ids\":[\"opennear_%03X\"],\"name\":\"OpenNear 0x%03X\","
                      "\"mf\":\"OpenNear\",\"mdl\":\"v6.0 Hybrid Mesh (%s)\"}}",
                      r, (unsigned)short_id, (unsigned)hop_depth,
                      (unsigned)short_id,
                      (unsigned)short_id,
                      (unsigned)short_id,
                      (unsigned)short_id,
                      (unsigned)short_id, (unsigned)short_id, r);
    return (jn > 0 && (size_t)jn < json_cap);
}

bool on_mqtt_format_node_state(uint16_t short_id, uint16_t parent_id, uint8_t hop_depth,
                               int16_t skew_ns, const uint8_t *data, uint8_t data_len,
                               char *topic_out, size_t topic_cap,
                               char *json_out, size_t json_cap) {
    if (!topic_out || !json_out || topic_cap < 32u || json_cap < 96u) return false;
    static const char kHex[] = "0123456789ABCDEF";
    char dhex[17];
    uint8_t n = (data && data_len <= 8u) ? data_len : (data ? 8u : 0u);
    for (uint8_t i = 0u; i < n; ++i) {
        dhex[i * 2u]      = kHex[(data[i] >> 4) & 0x0Fu];
        dhex[i * 2u + 1u] = kHex[data[i] & 0x0Fu];
    }
    dhex[n * 2u] = '\0';

    int tn = snprintf(topic_out, topic_cap, "opennear/node/0x%03X/state",
                      (unsigned)(short_id & ON_SHORT_ID_MASK));
    if (tn <= 0 || (size_t)tn >= topic_cap) return false;

    int jn = snprintf(json_out, json_cap,
                      "{\"id\":\"0x%03X\",\"parent\":\"0x%03X\",\"hop\":%u,"
                      "\"skew_ns\":%d,\"data\":\"%s\"}",
                      (unsigned)(short_id & ON_SHORT_ID_MASK),
                      (unsigned)(parent_id & ON_SHORT_ID_MASK),
                      (unsigned)hop_depth, (int)skew_ns, dhex);
    return (jn > 0 && (size_t)jn < json_cap);
}

bool on_mqtt_parse_cmd_topic(const char *topic, size_t topic_len,
                             uint16_t *out_target_id, bool *out_is_ota) {
    if (!topic || topic_len < 13u) return false;
    if (out_is_ota) *out_is_ota = false;
    if (out_target_id) *out_target_id = ON_SHORT_ID_UNASSIGNED;

    if (topic_len == 20u && memcmp(topic, "opennear/ota/trigger", 20u) == 0) {
        if (out_is_ota) *out_is_ota = true;
        return true;
    }
    if (topic_len > 13u && memcmp(topic, "opennear/cmd/", 13u) == 0) {
        char num[16];
        size_t nlen = topic_len - 13u;
        if (nlen >= sizeof(num)) nlen = sizeof(num) - 1u;
        memcpy(num, topic + 13, nlen);
        num[nlen] = '\0';
        uint16_t id = (uint16_t)(strtoul(num, NULL, 16) & ON_SHORT_ID_MASK);
        if (id == ON_SHORT_ID_UNASSIGNED) return false;
        if (out_target_id) *out_target_id = id;
        return true;
    }
    return false;
}
