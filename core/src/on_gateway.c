#include "on_gateway.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void copy_cstr(char *dst, size_t dst_cap, const char *src, size_t src_len) {
    if (!dst || dst_cap == 0u) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    size_t n = (src_len < dst_cap - 1u) ? src_len : (dst_cap - 1u);
    memcpy(dst, src, n);
    dst[n] = '\0';
}

/* ========================================================================== */
/* ON_FRAME_GW_BRIDGE Wire Codec                                              */
/* ========================================================================== */

on_err_t on_gw_bridge_pack(const on_gw_bridge_msg_t *m, uint8_t *out, size_t cap, size_t *len) {
    if (!m || !out || !len) return ON_ERR_INVALID_ARG;
    if (m->payload_len > ON_GW_MAX_PAYLOAD_LEN) return ON_ERR_TOO_LONG;
    size_t total = ON_GW_BRIDGE_HDR_LEN + (size_t)m->payload_len;
    if (cap < total) return ON_ERR_TOO_LONG;
    out[0] = m->bridge_type;
    out[1] = m->flags;
    put_u16(out + 2, m->origin_id & ON_SHORT_ID_MASK);
    put_u16(out + 4, m->seq);
    out[6] = m->payload_len;
    if (m->payload_len > 0u) {
        memcpy(out + ON_GW_BRIDGE_HDR_LEN, m->payload, m->payload_len);
    }
    *len = total;
    return ON_OK;
}

on_err_t on_gw_bridge_unpack(const uint8_t *in, size_t len, on_gw_bridge_msg_t *m) {
    if (!in || !m) return ON_ERR_INVALID_ARG;
    if (len < ON_GW_BRIDGE_HDR_LEN) return ON_ERR_TOO_SHORT;
    m->bridge_type = in[0];
    m->flags       = in[1];
    m->origin_id   = get_u16(in + 2) & ON_SHORT_ID_MASK;
    m->seq         = get_u16(in + 4);
    m->payload_len = in[6];
    if (m->payload_len > ON_GW_MAX_PAYLOAD_LEN ||
        len < ON_GW_BRIDGE_HDR_LEN + (size_t)m->payload_len) {
        return ON_ERR_TOO_SHORT;
    }
    if (m->payload_len > 0u) {
        memcpy(m->payload, in + ON_GW_BRIDGE_HDR_LEN, m->payload_len);
    }
    return ON_OK;
}

/* ========================================================================== */
/* Gateway Lifecycle & Wi-Fi / BLE Coexistence                                */
/* ========================================================================== */

void on_gw_init(on_gw_ctx_t *gw, on_gw_mode_t mode, const on_gw_transport_ops_t *ops) {
    if (!gw) return;
    memset(gw, 0, sizeof(*gw));
    gw->mode = mode;
    if (ops) {
        gw->ops = *ops;
    }
}

void on_gw_set_wifi_creds(on_gw_ctx_t *gw, const char *ssid, const char *pass,
                          const char *cloud_url) {
    if (!gw) return;
    if (ssid)      copy_cstr(gw->cfg.wifi_ssid, sizeof(gw->cfg.wifi_ssid), ssid, strlen(ssid));
    if (pass)      copy_cstr(gw->cfg.wifi_pass, sizeof(gw->cfg.wifi_pass), pass, strlen(pass));
    if (cloud_url) copy_cstr(gw->cfg.cloud_url, sizeof(gw->cfg.cloud_url), cloud_url, strlen(cloud_url));
    gw->cfg.wifi_creds_updated = true;
}

void on_gw_set_admin_token(on_gw_ctx_t *gw, const char *token) {
    if (!gw) return;
    if (token && token[0]) {
        copy_cstr(gw->cfg.admin_token, sizeof(gw->cfg.admin_token), token, strlen(token));
    } else {
        gw->cfg.admin_token[0] = '\0';
    }
}

uint8_t on_gw_on_wifi_connected(on_gw_ctx_t *gw, uint8_t ap_channel, int8_t rssi,
                                const uint8_t ip4[4]) {
    if (!gw) return 0u;
    gw->wifi_connected  = true;
    gw->wifi_ap_channel = (ap_channel >= 1u && ap_channel <= 13u) ? ap_channel : 6u;
    gw->wifi_rssi       = rssi;
    if (ip4) {
        memcpy(gw->wifi_ip, ip4, 4u);
    }
    /* Align OpenNear raw 802.11 channel directly to the connected Wi-Fi AP channel */
    gw->aligned_opennear_ch = gw->wifi_ap_channel;
    if (gw->ops.on_channel_align) {
        gw->ops.on_channel_align(gw->ops.user_ctx, gw->aligned_opennear_ch);
    }
    return gw->aligned_opennear_ch;
}

void on_gw_on_wifi_disconnected(on_gw_ctx_t *gw) {
    if (!gw) return;
    gw->wifi_connected = false;
    memset(gw->wifi_ip, 0, sizeof(gw->wifi_ip));
}

void on_gw_on_ble_state(on_gw_ctx_t *gw, bool connected, uint16_t mtu) {
    if (!gw) return;
    gw->ble_connected = connected;
    gw->ble_mtu       = connected ? (mtu >= 23u ? mtu : 23u) : 0u;
}

/* ========================================================================== */
/* Ring Buffers                                                               */
/* ========================================================================== */

static bool ring_push(on_gw_ring_t *r, const on_gw_bridge_msg_t *msg) {
    if (!r || !msg) return false;
    uint8_t t = r->tail;
    uint8_t next = (uint8_t)((t + 1u) & (ON_GW_RING_DEPTH - 1u));
    if (next == r->head) {
        r->drops++;
        return false;
    }
    r->items[t] = *msg;
    r->tail = next;
    return true;
}

static bool ring_pop(on_gw_ring_t *r, on_gw_bridge_msg_t *out) {
    if (!r || !out) return false;
    uint8_t h = r->head;
    if (h == r->tail) return false;
    *out = r->items[h];
    r->head = (uint8_t)((h + 1u) & (ON_GW_RING_DEPTH - 1u));
    return true;
}

bool on_gw_push_uplink(on_gw_ctx_t *gw, const on_gw_bridge_msg_t *msg) {
    return gw ? ring_push(&gw->uplink_q, msg) : false;
}

bool on_gw_pop_uplink(on_gw_ctx_t *gw, on_gw_bridge_msg_t *out) {
    return gw ? ring_pop(&gw->uplink_q, out) : false;
}

bool on_gw_push_downlink(on_gw_ctx_t *gw, const on_gw_bridge_msg_t *msg) {
    return gw ? ring_push(&gw->downlink_q, msg) : false;
}

bool on_gw_pop_downlink(on_gw_ctx_t *gw, on_gw_bridge_msg_t *out) {
    return gw ? ring_pop(&gw->downlink_q, out) : false;
}

/* ========================================================================== */
/* BLE GATT & Cloud Control Text Parser                                       */
/* ========================================================================== */

static const char *find_sep(const char *s, const char *end, char sep) {
    while (s < end) {
        if (*s == sep) return s;
        ++s;
    }
    return NULL;
}

static bool parse_mac_span(const char *p, const char *end, uint8_t out_mac[6]) {
    size_t nibs = 0u;
    uint8_t cur = 0u;
    for (const char *q = p; q < end; ++q) {
        if (*q == ':' || *q == '-') continue;
        int v = -1;
        if (*q >= '0' && *q <= '9') v = *q - '0';
        else if (*q >= 'A' && *q <= 'F') v = 10 + (*q - 'A');
        else if (*q >= 'a' && *q <= 'f') v = 10 + (*q - 'a');
        if (v < 0 || nibs >= 12u) return false;
        if ((nibs & 1u) == 0u) cur = (uint8_t)(v << 4);
        else out_mac[nibs >> 1] = (uint8_t)(cur | (uint8_t)v);
        nibs++;
    }
    return nibs == 12u;
}

static bool consttime_eq(const char *a, const char *b, size_t n) {
    unsigned char diff = 0u;
    for (size_t i = 0u; i < n; ++i) {
        diff |= (unsigned char)(a[i] ^ b[i]);
    }
    return diff == 0u;
}

on_err_t on_gw_handle_control_text(on_gw_ctx_t *gw, const char *text, size_t len, bool from_ble) {
    if (!gw || !text || len < 5u) return ON_ERR_INVALID_ARG;
    const char *end = text + len;
    while (end > text && (end[-1] == '\r' || end[-1] == '\n' || end[-1] == ' ')) {
        --end;
    }
    size_t clean_len = (size_t)(end - text);
    if (clean_len < 5u) return ON_ERR_INVALID_ARG;

    /* Optional Gateway Admin Token authentication ("AUTH:<token>|<command>") */
    if (gw->cfg.admin_token[0] != '\0') {
        bool is_readonly = (clean_len == 5u &&
                            (memcmp(text, "TOPO?", 5u) == 0 || memcmp(text, "DATA?", 5u) == 0));
        if (!is_readonly) {
            if (clean_len <= 6u || memcmp(text, "AUTH:", 5u) != 0) {
                return ON_ERR_DECODE_FAIL;
            }
            const char *tok_start = text + 5;
            const char *bar = find_sep(tok_start, end, '|');
            size_t expect_len = strlen(gw->cfg.admin_token);
            if (!bar || (size_t)(bar - tok_start) != expect_len ||
                !consttime_eq(tok_start, gw->cfg.admin_token, expect_len)) {
                return ON_ERR_DECODE_FAIL;
            }
            text      = bar + 1;
            clean_len = (size_t)(end - text);
            if (clean_len < 5u) return ON_ERR_INVALID_ARG;
        }
    }

    if (from_ble) gw->ble_cmds_parsed++;

    /* Format 1: WIFI:<ssid>|<password>[|<cloud_url>] */
    if (clean_len > 5u && memcmp(text, "WIFI:", 5u) == 0) {
        const char *p1 = text + 5;
        const char *bar1 = find_sep(p1, end, '|');
        if (!bar1 || bar1 == p1) return ON_ERR_INVALID_ARG;
        copy_cstr(gw->cfg.wifi_ssid, sizeof(gw->cfg.wifi_ssid), p1, (size_t)(bar1 - p1));

        const char *p2 = bar1 + 1;
        const char *bar2 = find_sep(p2, end, '|');
        if (bar2) {
            copy_cstr(gw->cfg.wifi_pass, sizeof(gw->cfg.wifi_pass), p2, (size_t)(bar2 - p2));
            const char *p3 = bar2 + 1;
            copy_cstr(gw->cfg.cloud_url, sizeof(gw->cfg.cloud_url), p3, (size_t)(end - p3));
        } else {
            copy_cstr(gw->cfg.wifi_pass, sizeof(gw->cfg.wifi_pass), p2, (size_t)(end - p2));
        }
        gw->cfg.wifi_creds_updated = true;
        gw->wifi_prov_count++;
        if (gw->ops.wifi_connect) {
            gw->ops.wifi_connect(gw->ops.user_ctx, gw->cfg.wifi_ssid, gw->cfg.wifi_pass);
        }
        return ON_OK;
    }

    /* Format 2: OTA:<target_hex>|<version_u32>|<http_url> */
    if (clean_len > 4u && memcmp(text, "OTA:", 4u) == 0) {
        const char *p1 = text + 4;
        const char *bar1 = find_sep(p1, end, '|');
        if (!bar1 || bar1 == p1) return ON_ERR_INVALID_ARG;
        char numbuf[16];
        copy_cstr(numbuf, sizeof(numbuf), p1, (size_t)(bar1 - p1));
        uint16_t target_id = (uint16_t)(strtoul(numbuf, NULL, 16) & ON_SHORT_ID_MASK);

        const char *p2 = bar1 + 1;
        const char *bar2 = find_sep(p2, end, '|');
        if (!bar2 || bar2 == p2) return ON_ERR_INVALID_ARG;
        copy_cstr(numbuf, sizeof(numbuf), p2, (size_t)(bar2 - p2));
        uint32_t ver = (uint32_t)strtoul(numbuf, NULL, 0);

        const char *p3 = bar2 + 1;
        if (p3 >= end) return ON_ERR_INVALID_ARG;
        copy_cstr(gw->cfg.ota_url, sizeof(gw->cfg.ota_url), p3, (size_t)(end - p3));
        gw->cfg.ota_target_id      = target_id;
        gw->cfg.ota_target_ver     = ver;
        gw->cfg.ota_pull_requested = true;
        return ON_OK;
    }

    /* Format 3: CMD:<target_hex>|<payload_text> */
    if (clean_len > 4u && memcmp(text, "CMD:", 4u) == 0) {
        const char *p1 = text + 4;
        const char *bar1 = find_sep(p1, end, '|');
        if (!bar1 || bar1 == p1) return ON_ERR_INVALID_ARG;
        char numbuf[16];
        copy_cstr(numbuf, sizeof(numbuf), p1, (size_t)(bar1 - p1));
        uint16_t target_id = (uint16_t)(strtoul(numbuf, NULL, 16) & ON_SHORT_ID_MASK);

        const char *p2 = bar1 + 1;
        size_t plen = (size_t)(end - p2);
        if (plen > ON_GW_MAX_PAYLOAD_LEN) plen = ON_GW_MAX_PAYLOAD_LEN;

        on_gw_bridge_msg_t msg;
        memset(&msg, 0, sizeof(msg));
        msg.bridge_type = from_ble ? (uint8_t)ON_GW_MSG_BLE_CMD_DOWN : (uint8_t)ON_GW_MSG_CLOUD_CMD_DOWN;
        msg.origin_id   = target_id;
        msg.seq         = ++gw->tx_seq;
        msg.payload_len = (uint8_t)plen;
        if (plen > 0u) memcpy(msg.payload, p2, plen);
        return on_gw_push_downlink(gw, &msg) ? ON_OK : ON_ERR_NO_MEM;
    }

    /* Format 4: TOPO? / DATA? -> Query full cluster topology & live node data */
    if ((clean_len == 5u && memcmp(text, "TOPO?", 5u) == 0) ||
        (clean_len == 5u && memcmp(text, "DATA?", 5u) == 0)) {
        gw->cfg.topo_query_requested = true;
        gw->cfg.topo_include_data    = (text[0] == 'D');
        return ON_OK;
    }

    /* Format 5: MQTT:<host>|<port>[|<user>|<pass>[|<tls:0|1>]] or MQTTS:<host>|<port>[|<user>|<pass>] */
    if ((clean_len > 5u && memcmp(text, "MQTT:", 5u) == 0) ||
        (clean_len > 6u && memcmp(text, "MQTTS:", 6u) == 0)) {
        bool explicit_tls = (text[4] == 'S');
        const char *p1 = text + (explicit_tls ? 6 : 5);
        const char *bar1 = find_sep(p1, end, '|');
        if (!bar1 || bar1 == p1) return ON_ERR_INVALID_ARG;
        copy_cstr(gw->cfg.mqtt_host, sizeof(gw->cfg.mqtt_host), p1, (size_t)(bar1 - p1));
        const char *p2 = bar1 + 1;
        const char *bar2 = find_sep(p2, end, '|');
        char pbuf[16];
        copy_cstr(pbuf, sizeof(pbuf), p2, (size_t)((bar2 ? bar2 : end) - p2));
        unsigned long port = strtoul(pbuf, NULL, 10);
        uint16_t def_port = explicit_tls ? 8883u : 1883u;
        gw->cfg.mqtt_port    = (port > 0u && port <= 65535u) ? (uint16_t)port : def_port;
        gw->cfg.mqtt_use_tls = explicit_tls || (gw->cfg.mqtt_port == 8883u);
        gw->cfg.mqtt_user[0] = '\0';
        gw->cfg.mqtt_pass[0] = '\0';
        if (bar2) {
            const char *p3 = bar2 + 1;
            const char *bar3 = find_sep(p3, end, '|');
            copy_cstr(gw->cfg.mqtt_user, sizeof(gw->cfg.mqtt_user), p3, (size_t)((bar3 ? bar3 : end) - p3));
            if (bar3) {
                const char *p4 = bar3 + 1;
                const char *bar4 = find_sep(p4, end, '|');
                copy_cstr(gw->cfg.mqtt_pass, sizeof(gw->cfg.mqtt_pass), p4, (size_t)((bar4 ? bar4 : end) - p4));
                if (bar4) {
                    const char *p5 = bar4 + 1;
                    if (p5 < end) {
                        gw->cfg.mqtt_use_tls = (*p5 == '1' || *p5 == 't' || *p5 == 'T');
                    }
                }
            }
        }
        gw->cfg.mqtt_creds_updated = true;
        return ON_OK;
    }

    /* Format 6: TOKEN:<new_token> -> Set/rotate (or clear if empty) Gateway Admin Token */
    if (clean_len >= 6u && memcmp(text, "TOKEN:", 6u) == 0) {
        const char *p1 = text + 6;
        char tbuf[sizeof(gw->cfg.admin_token)];
        copy_cstr(tbuf, sizeof(tbuf), p1, (size_t)(end - p1));
        on_gw_set_admin_token(gw, tbuf);
        gw->cfg.admin_token_updated = true;
        return ON_OK;
    }

    /* Format 7: RESET_WIFI -> Clear saved Wi-Fi STA credentials & reopen SoftAP setup portal */
    if (clean_len == 10u && memcmp(text, "RESET_WIFI", 10u) == 0) {
        gw->cfg.wifi_ssid[0] = '\0';
        gw->cfg.wifi_pass[0] = '\0';
        gw->cfg.wifi_reset_requested = true;
        return ON_OK;
    }

    /* Format 8: REBOOT -> Trigger graceful gateway restart */
    if (clean_len == 6u && memcmp(text, "REBOOT", 6u) == 0) {
        gw->cfg.reboot_requested = true;
        return ON_OK;
    }

    /* Format 9: NETKEY:[<32hex>] -> Set custom AES-128 cluster master key (empty clears to default) */
    if (clean_len >= 7u && memcmp(text, "NETKEY:", 7u) == 0) {
        const char *hex = text + 7;
        size_t hlen = (size_t)(end - hex);
        if (hlen == 0u) {
            memset(gw->cfg.net_key, 0, sizeof(gw->cfg.net_key));
            gw->cfg.net_key_custom  = false;
            gw->cfg.net_key_updated = true;
            return ON_OK;
        }
        if (hlen != 32u) return ON_ERR_INVALID_ARG;
        uint8_t kbuf[16];
        for (size_t i = 0u; i < 16u; ++i) {
            int hi = -1, lo = -1;
            char c1 = hex[i * 2u], c2 = hex[i * 2u + 1u];
            if (c1 >= '0' && c1 <= '9') hi = c1 - '0';
            else if (c1 >= 'A' && c1 <= 'F') hi = 10 + (c1 - 'A');
            else if (c1 >= 'a' && c1 <= 'f') hi = 10 + (c1 - 'a');
            if (c2 >= '0' && c2 <= '9') lo = c2 - '0';
            else if (c2 >= 'A' && c2 <= 'F') lo = 10 + (c2 - 'A');
            else if (c2 >= 'a' && c2 <= 'f') lo = 10 + (c2 - 'a');
            if (hi < 0 || lo < 0) return ON_ERR_INVALID_ARG;
            kbuf[i] = (uint8_t)((hi << 4) | lo);
        }
        memcpy(gw->cfg.net_key, kbuf, 16u);
        gw->cfg.net_key_custom  = true;
        gw->cfg.net_key_updated = true;
        return ON_OK;
    }

    /* Format 10: ALIAS:<MAC_HEX_OR_COLON>|<friendly_name> -> Assign/clear MAC-anchored node alias */
    if (clean_len > 6u && memcmp(text, "ALIAS:", 6u) == 0) {
        const char *p1 = text + 6;
        const char *bar1 = find_sep(p1, end, '|');
        if (!bar1 || bar1 == p1) return ON_ERR_INVALID_ARG;
        uint8_t mac[6];
        if (!parse_mac_span(p1, bar1, mac)) return ON_ERR_INVALID_ARG;
        memcpy(gw->cfg.alias_mac, mac, 6u);
        copy_cstr(gw->cfg.alias_name, sizeof(gw->cfg.alias_name), bar1 + 1, (size_t)(end - (bar1 + 1)));
        gw->cfg.alias_updated = true;
        return ON_OK;
    }

    /* Format 11: TAG:<MAC>|<tag1,tag2> -> Assign/clear dynamic metadata tags for a node */
    if (clean_len > 4u && memcmp(text, "TAG:", 4u) == 0) {
        const char *p1 = text + 4;
        const char *bar1 = find_sep(p1, end, '|');
        if (!bar1 || bar1 == p1) return ON_ERR_INVALID_ARG;
        uint8_t mac[6];
        if (!parse_mac_span(p1, bar1, mac)) return ON_ERR_INVALID_ARG;
        memcpy(gw->cfg.tag_mac, mac, 6u);
        copy_cstr(gw->cfg.tag_list, sizeof(gw->cfg.tag_list), bar1 + 1, (size_t)(end - (bar1 + 1)));
        gw->cfg.tag_updated = true;
        return ON_OK;
    }

    /* Format 12: CMD_TAG:<tag>|<payload> -> Group command to all nodes carrying <tag> */
    if (clean_len > 8u && memcmp(text, "CMD_TAG:", 8u) == 0) {
        const char *p1 = text + 8;
        const char *bar1 = find_sep(p1, end, '|');
        if (!bar1 || bar1 == p1) return ON_ERR_INVALID_ARG;
        copy_cstr(gw->cfg.cmd_tag, sizeof(gw->cfg.cmd_tag), p1, (size_t)(bar1 - p1));
        const char *p2 = bar1 + 1;
        size_t plen = (size_t)(end - p2);
        if (plen == 0u) return ON_ERR_INVALID_ARG;
        if (plen > ON_GW_MAX_PAYLOAD_LEN) plen = ON_GW_MAX_PAYLOAD_LEN;
        memcpy(gw->cfg.cmd_tag_payload, p2, plen);
        gw->cfg.cmd_tag_payload_len = (uint8_t)plen;
        gw->cfg.cmd_tag_requested   = true;
        return ON_OK;
    }

    /* Format 13: ANCHOR:<MAC>|<room>|<x_cm>|<y_cm> (empty room removes anchor) */
    if (clean_len > 7u && memcmp(text, "ANCHOR:", 7u) == 0) {
        const char *p1 = text + 7;
        const char *bar1 = find_sep(p1, end, '|');
        if (!bar1 || bar1 == p1) return ON_ERR_INVALID_ARG;
        uint8_t mac[6];
        if (!parse_mac_span(p1, bar1, mac)) return ON_ERR_INVALID_ARG;
        memcpy(gw->cfg.anchor_mac, mac, 6u);
        const char *p2 = bar1 + 1;
        const char *bar2 = find_sep(p2, end, '|');
        copy_cstr(gw->cfg.anchor_room, sizeof(gw->cfg.anchor_room), p2,
                  (size_t)((bar2 ? bar2 : end) - p2));
        gw->cfg.anchor_x_cm = 0;
        gw->cfg.anchor_y_cm = 0;
        if (bar2) {
            const char *p3 = bar2 + 1;
            const char *bar3 = find_sep(p3, end, '|');
            char nbuf[16];
            copy_cstr(nbuf, sizeof(nbuf), p3, (size_t)((bar3 ? bar3 : end) - p3));
            gw->cfg.anchor_x_cm = (int32_t)strtol(nbuf, NULL, 10);
            if (bar3) {
                copy_cstr(nbuf, sizeof(nbuf), bar3 + 1, (size_t)(end - (bar3 + 1)));
                gw->cfg.anchor_y_cm = (int32_t)strtol(nbuf, NULL, 10);
            }
        }
        gw->cfg.anchor_updated = true;
        return ON_OK;
    }

    /* Format 14: BLESCAN:<0|1> -> Toggle passive BLE iBeacon/Eddystone scanner */
    if (clean_len >= 9u && memcmp(text, "BLESCAN:", 8u) == 0) {
        char c = text[8];
        if (c != '0' && c != '1') return ON_ERR_INVALID_ARG;
        gw->cfg.ble_scan_enabled = (c == '1');
        gw->cfg.ble_scan_updated = true;
        return ON_OK;
    }

    /* Format 15: FENCE:<name>|<x_cm>|<y_cm>|<radius_cm> (radius_cm == 0 removes fence) */
    if (clean_len > 6u && memcmp(text, "FENCE:", 6u) == 0) {
        const char *p1 = text + 6;
        const char *b1 = find_sep(p1, end, '|');
        if (!b1 || b1 == p1) return ON_ERR_INVALID_ARG;
        copy_cstr(gw->cfg.fence_name, sizeof(gw->cfg.fence_name), p1, (size_t)(b1 - p1));
        gw->cfg.fence_x_cm      = 0;
        gw->cfg.fence_y_cm      = 0;
        gw->cfg.fence_radius_cm = 0u;
        const char *p2 = b1 + 1;
        const char *b2 = find_sep(p2, end, '|');
        char nbuf[16];
        copy_cstr(nbuf, sizeof(nbuf), p2, (size_t)((b2 ? b2 : end) - p2));
        gw->cfg.fence_x_cm = (int32_t)strtol(nbuf, NULL, 10);
        if (b2) {
            const char *p3 = b2 + 1;
            const char *b3 = find_sep(p3, end, '|');
            copy_cstr(nbuf, sizeof(nbuf), p3, (size_t)((b3 ? b3 : end) - p3));
            gw->cfg.fence_y_cm = (int32_t)strtol(nbuf, NULL, 10);
            if (b3) {
                copy_cstr(nbuf, sizeof(nbuf), b3 + 1, (size_t)(end - (b3 + 1)));
                long r = strtol(nbuf, NULL, 10);
                gw->cfg.fence_radius_cm = (r > 0 && r <= 30000) ? (uint16_t)r : 0u;
            }
        }
        gw->cfg.fence_updated = true;
        return ON_OK;
    }

    /* Format 16: RULE:<id>|<trig>|<zone>|<csi_th>|<tag>|<cmd> (trig == 0 removes rule) */
    if (clean_len > 5u && memcmp(text, "RULE:", 5u) == 0) {
        const char *p1 = text + 5;
        const char *b1 = find_sep(p1, end, '|');
        if (!b1 || b1 == p1) return ON_ERR_INVALID_ARG;
        char nbuf[16];
        copy_cstr(nbuf, sizeof(nbuf), p1, (size_t)(b1 - p1));
        unsigned long rid = strtoul(nbuf, NULL, 10);
        if (rid == 0u || rid > 8u) return ON_ERR_INVALID_ARG;
        const char *p2 = b1 + 1;
        const char *b2 = find_sep(p2, end, '|');
        copy_cstr(nbuf, sizeof(nbuf), p2, (size_t)((b2 ? b2 : end) - p2));
        unsigned long trig = strtoul(nbuf, NULL, 10);
        if (trig > 6u) return ON_ERR_INVALID_ARG;
        gw->cfg.rule_id           = (uint8_t)rid;
        gw->cfg.rule_trig_type    = (uint8_t)trig;
        gw->cfg.rule_zone[0]      = '\0';
        gw->cfg.rule_csi_thresh   = 40u;
        gw->cfg.rule_cmd_tag[0]   = '\0';
        gw->cfg.rule_cmd_payload[0] = '\0';
        if (trig > 0u) {
            if (!b2) return ON_ERR_INVALID_ARG;
            const char *p3 = b2 + 1;
            const char *b3 = find_sep(p3, end, '|');
            if (!b3 || b3 == p3) return ON_ERR_INVALID_ARG;
            copy_cstr(gw->cfg.rule_zone, sizeof(gw->cfg.rule_zone), p3, (size_t)(b3 - p3));
            const char *p4 = b3 + 1;
            const char *b4 = find_sep(p4, end, '|');
            if (!b4) return ON_ERR_INVALID_ARG;
            copy_cstr(nbuf, sizeof(nbuf), p4, (size_t)(b4 - p4));
            unsigned long th = strtoul(nbuf, NULL, 10);
            gw->cfg.rule_csi_thresh = (th > 0u && th <= 100u) ? (uint8_t)th : 40u;
            const char *p5 = b4 + 1;
            const char *b5 = find_sep(p5, end, '|');
            if (!b5 || b5 + 1 >= end) return ON_ERR_INVALID_ARG;
            copy_cstr(gw->cfg.rule_cmd_tag, sizeof(gw->cfg.rule_cmd_tag), p5, (size_t)(b5 - p5));
            copy_cstr(gw->cfg.rule_cmd_payload, sizeof(gw->cfg.rule_cmd_payload), b5 + 1, (size_t)(end - (b5 + 1)));
        }
        gw->cfg.rule_updated = true;
        return ON_OK;
    }

    /* Format 17: LOCDEMO:<0|1|2> -> Start/stop/step 4-Room Geofence + Wi-Fi CSI Showcase Demo */
    if (clean_len >= 9u && memcmp(text, "LOCDEMO:", 8u) == 0) {
        char c = text[8];
        if (c < '0' || c > '2') return ON_ERR_INVALID_ARG;
        gw->cfg.loc_demo_mode    = (uint8_t)(c - '0');
        gw->cfg.loc_demo_updated = true;
        return ON_OK;
    }

    /* Format 18: LRMODE:<0|1|2> -> Switch 4ms/54M Fast (0), 20ms/1M 500m (1), or 20ms/250K 700m+ LR (2) */
    if (clean_len >= 8u && memcmp(text, "LRMODE:", 7u) == 0) {
        char c = text[7];
        if (c < '0' || c > '2') return ON_ERR_INVALID_ARG;
        gw->cfg.lr_mode         = (uint8_t)(c - '0');
        gw->cfg.lr_mode_updated = true;
        return ON_OK;
    }

    return ON_ERR_INVALID_ARG;
}

size_t on_gw_format_wifi_prov(const on_gw_ctx_t *gw, char *out_buf, size_t out_cap) {
    if (!gw || !out_buf || out_cap < 16u || gw->cfg.wifi_ssid[0] == '\0') return 0u;
    int n = 0;
    if (gw->cfg.cloud_url[0] != '\0') {
        n = snprintf(out_buf, out_cap, "WIFI:%s|%s|%s",
                     gw->cfg.wifi_ssid, gw->cfg.wifi_pass, gw->cfg.cloud_url);
    } else {
        n = snprintf(out_buf, out_cap, "WIFI:%s|%s",
                     gw->cfg.wifi_ssid, gw->cfg.wifi_pass);
    }
    if (n <= 0 || (size_t)n >= out_cap || (size_t)n > ON_GW_MAX_PAYLOAD_LEN) return 0u;
    return (size_t)n;
}

size_t on_gw_format_mqtt_prov(const on_gw_ctx_t *gw, char *out_buf, size_t out_cap) {
    if (!gw || !out_buf || out_cap < 16u || gw->cfg.mqtt_host[0] == '\0') return 0u;
    uint16_t port = gw->cfg.mqtt_port ? gw->cfg.mqtt_port : (gw->cfg.mqtt_use_tls ? 8883u : 1883u);
    int n = snprintf(out_buf, out_cap, "MQTT:%s|%u|%s|%s|%u",
                     gw->cfg.mqtt_host, (unsigned)port,
                     gw->cfg.mqtt_user, gw->cfg.mqtt_pass,
                     gw->cfg.mqtt_use_tls ? 1u : 0u);
    if (n <= 0 || (size_t)n >= out_cap || (size_t)n > ON_GW_MAX_PAYLOAD_LEN) return 0u;
    return (size_t)n;
}

/* ========================================================================== */
/* Cloud JSON Formatter & Network Poller                                      */
/* ========================================================================== */

size_t on_gw_format_uplink_json(const on_gw_ctx_t *gw, const on_gw_bridge_msg_t *msg,
                                int16_t skew_ns, char *out_buf, size_t out_cap) {
    if (!gw || !msg || !out_buf || out_cap < 64u) return 0u;
    static const char kHex[] = "0123456789ABCDEF";
    char hex_pay[ON_GW_MAX_PAYLOAD_LEN * 2u + 1u];
    size_t plen = (msg->payload_len <= ON_GW_MAX_PAYLOAD_LEN) ? msg->payload_len : ON_GW_MAX_PAYLOAD_LEN;
    for (size_t i = 0u; i < plen; ++i) {
        hex_pay[i * 2u]      = kHex[(msg->payload[i] >> 4) & 0x0Fu];
        hex_pay[i * 2u + 1u] = kHex[msg->payload[i] & 0x0Fu];
    }
    hex_pay[plen * 2u] = '\0';

    int hop = (msg->flags & ON_GW_FLAG_RELAYED_2HOP) ? 2 : 1;
    int n = snprintf(out_buf, out_cap,
                     "{\"node\":\"0x%03X\",\"type\":%u,\"hop\":%d,\"seq\":%u,"
                     "\"skew_ns\":%d,\"wifi_ch\":%u,\"ble\":%u,\"data_hex\":\"%s\"}",
                     (unsigned)msg->origin_id,
                     (unsigned)msg->bridge_type,
                     hop,
                     (unsigned)msg->seq,
                     (int)skew_ns,
                     (unsigned)gw->wifi_ap_channel,
                     gw->ble_connected ? 1u : 0u,
                     hex_pay);
    return (n > 0 && (size_t)n < out_cap) ? (size_t)n : 0u;
}

uint32_t on_gw_poll_network(on_gw_ctx_t *gw) {
    if (!gw) return 0u;
    uint32_t sent = 0u;
    on_gw_bridge_msg_t msg;
    char json[384];

    while (on_gw_pop_uplink(gw, &msg)) {
        size_t jlen = on_gw_format_uplink_json(gw, &msg, 0, json, sizeof(json));
        bool delivered = false;

        if ((gw->mode == ON_GW_MODE_WIFI_STA || gw->mode == ON_GW_MODE_WIFI_AND_BLE) &&
            gw->wifi_connected && gw->ops.cloud_post_json && gw->cfg.cloud_url[0] != '\0' && jlen > 0u) {
            if (gw->ops.cloud_post_json(gw->ops.user_ctx, gw->cfg.cloud_url, json) == ON_OK) {
                delivered = true;
            }
        }
        if ((gw->mode == ON_GW_MODE_BLE_GATT || gw->mode == ON_GW_MODE_WIFI_AND_BLE) &&
            gw->ble_connected && gw->ops.ble_notify && jlen > 0u) {
            if (gw->ops.ble_notify(gw->ops.user_ctx, (const uint8_t *)json, jlen) == ON_OK) {
                delivered = true;
            }
        }
        if (gw->mqtt_connected && gw->ops.mqtt_publish && jlen > 0u) {
            char mtopic[48];
            snprintf(mtopic, sizeof(mtopic), "opennear/node/0x%03X/state", (unsigned)msg.origin_id);
            if (gw->ops.mqtt_publish(gw->ops.user_ctx, mtopic, json, false) == ON_OK) {
                delivered = true;
            }
        }
        if (delivered) {
            gw->uplink_forwarded++;
            sent++;
        }
    }
    return sent;
}
