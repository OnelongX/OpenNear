#ifndef OPENNEAR_GATEWAY_H
#define OPENNEAR_GATEWAY_H

#include "on_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OpenNear v5.0 Public Network Wi-Fi STA & BLE GATT Gateway Bridge
 *
 * Bridges an isolated real-time OpenNear cluster (1-hop + 2-hop Sub-G edge nodes)
 * to the public Internet (Wi-Fi STA HTTP/MQTT/OTA) and mobile devices (BLE GATT):
 *
 * - Dynamic Wi-Fi AP Channel Alignment: tracks the router's home channel (1..13)
 *   so raw 802.11 OpenNear frames and public TCP/IP coexist on one 2.4 GHz radio.
 * - Zero-Touch BLE Wi-Fi Provisioning: parses WIFI:<ssid>|<pass>|<cloud_url>,
 *   OTA:<target_hex>|<ver>|<url>, and CMD:<target_hex>|<text> over BLE GATT.
 * - Lock-free SPSC ring buffers decoupling Core-1 real-time OpenNear TDMA from
 *   Core-0 blocking TCP/HTTP/TLS and BLE GATT stacks.
 */

#define ON_GW_MAX_PAYLOAD_LEN   96u
#define ON_GW_BRIDGE_HDR_LEN    7u
#define ON_GW_RING_DEPTH        16u
#define ON_GW_SSID_MAX_LEN      33u
#define ON_GW_PASS_MAX_LEN      65u
#define ON_GW_URL_MAX_LEN       128u

typedef enum {
    ON_GW_MODE_OFF          = 0,
    ON_GW_MODE_WIFI_STA     = 1,
    ON_GW_MODE_BLE_GATT     = 2,
    ON_GW_MODE_WIFI_AND_BLE = 3
} on_gw_mode_t;

typedef enum {
    ON_GW_MSG_TELEMETRY_UP   = 0x01, /* Terminal -> G -> Cloud HTTP/MQTT */
    ON_GW_MSG_CLOUD_CMD_DOWN = 0x02, /* Cloud HTTP/MQTT -> G -> Terminal */
    ON_GW_MSG_BLE_NOTIFY_UP  = 0x03, /* Terminal -> G -> Phone BLE GATT */
    ON_GW_MSG_BLE_CMD_DOWN   = 0x04, /* Phone BLE GATT -> G -> Terminal */
    ON_GW_MSG_WIFI_PROV      = 0x05, /* BLE Wi-Fi provisioning config */
    ON_GW_MSG_OTA_TRIGGER    = 0x06, /* Cloud/BLE trigger to pull & push OTA */
    ON_GW_MSG_TOPO_REQ       = 0x07, /* v5.1: Any-node request for cluster topology & data */
    ON_GW_MSG_TOPO_RESP      = 0x08, /* v5.1: Paginated cluster topology & data snapshot from G */
    ON_GW_MSG_MAC_REPORT     = 0x09, /* v6.5: Sub-G -> G 2-hop child hardware MAC48 registration */
    ON_GW_MSG_RSSI_REPORT    = 0x0A  /* v7.0: Anchor -> G overheard peer RSSI observation batch */
} on_gw_msg_type_t;

#define ON_GW_FLAG_RELAYED_2HOP  0x01u
#define ON_GW_FLAG_ACK_REQUIRED  0x02u

/* Wire payload for ON_FRAME_GW_BRIDGE (0xC) */
typedef struct {
    uint8_t  bridge_type;                  /* on_gw_msg_type_t */
    uint8_t  flags;
    uint16_t origin_id;                    /* 12-bit source/target terminal Short-ID */
    uint16_t seq;
    uint8_t  payload_len;
    uint8_t  payload[ON_GW_MAX_PAYLOAD_LEN];
} on_gw_bridge_msg_t;

/* Provisioned Wi-Fi & Cloud endpoints */
typedef struct {
    char     wifi_ssid[ON_GW_SSID_MAX_LEN];
    char     wifi_pass[ON_GW_PASS_MAX_LEN];
    char     cloud_url[ON_GW_URL_MAX_LEN];
    char     ota_url[ON_GW_URL_MAX_LEN];
    char     mqtt_host[64];
    uint16_t mqtt_port;
    char     mqtt_user[32];
    char     mqtt_pass[32];
    bool     mqtt_use_tls;     /* true = MQTTS over TLS (default port 8883), requires OPENNEAR_ENABLE_TLS=1 */
    char     admin_token[32];  /* Optional auth PIN/token for HTTP /api/cmd & BLE 0xFFE2 */
    uint16_t ota_target_id;
    uint32_t ota_target_ver;
    bool     wifi_creds_updated;
    bool     mqtt_creds_updated;
    bool     admin_token_updated;
    bool     ota_pull_requested;
    bool     topo_query_requested;
    bool     topo_include_data;
    bool     wifi_reset_requested;
    bool     reboot_requested;
    uint8_t  net_key[16];      /* v6.7 Optional custom AES-128 cluster master key (NETKEY:<32hex>) */
    bool     net_key_custom;
    bool     net_key_updated;
    uint8_t  alias_mac[6];     /* v6.7 MAC-anchored node alias update (ALIAS:<MAC>|<name>) */
    char     alias_name[24];
    bool     alias_updated;
    uint8_t  tag_mac[6];       /* v7.0 MAC-anchored node tags (TAG:<MAC>|<tag1,tag2>) */
    char     tag_list[32];
    bool     tag_updated;
    char     cmd_tag[24];      /* v7.0 Tag group command (CMD_TAG:<tag>|<payload>) */
    uint8_t  cmd_tag_payload[ON_GW_MAX_PAYLOAD_LEN];
    uint8_t  cmd_tag_payload_len;
    bool     cmd_tag_requested;
    uint8_t  anchor_mac[6];    /* v7.0 Dynamic anchor config (ANCHOR:<MAC>|<room>|<x_cm>|<y_cm>) */
    char     anchor_room[20];
    int32_t  anchor_x_cm;
    int32_t  anchor_y_cm;
    bool     anchor_updated;
} on_gw_config_t;

/* Lock-free SPSC ring buffer for cross-core/cross-task decoupling (ON_GW_RING_DEPTH must be power of 2) */
typedef struct {
    on_gw_bridge_msg_t items[ON_GW_RING_DEPTH];
    volatile uint8_t   head;
    volatile uint8_t   tail;
    volatile uint32_t  drops;
} on_gw_ring_t;

static inline uint8_t on_gw_ring_count(const on_gw_ring_t *r) {
    if (!r) return 0u;
    return (uint8_t)((r->tail - r->head) & (ON_GW_RING_DEPTH - 1u));
}

/* Platform transport callbacks (ESP32 Wi-Fi HTTP + BLE GATT, or host test mock) */
typedef struct {
    void *user_ctx;
    on_err_t (*wifi_connect)(void *ctx, const char *ssid, const char *pass);
    on_err_t (*cloud_post_json)(void *ctx, const char *url, const char *json_body);
    on_err_t (*ble_notify)(void *ctx, const uint8_t *data, size_t len);
    on_err_t (*mqtt_publish)(void *ctx, const char *topic, const char *payload, bool retain);
    on_err_t (*on_channel_align)(void *ctx, uint8_t wifi_ch);
    on_err_t (*on_downlink_delivered)(void *ctx, const on_gw_bridge_msg_t *msg);
} on_gw_transport_ops_t;

typedef struct {
    on_gw_mode_t          mode;
    on_gw_config_t        cfg;
    on_gw_transport_ops_t ops;

    /* Live public-network coexistence state */
    bool                  wifi_connected;
    uint8_t               wifi_ap_channel;    /* 1..13 when STA connected, 0 otherwise */
    int8_t                wifi_rssi;
    uint8_t               wifi_ip[4];
    bool                  ble_connected;
    uint16_t              ble_mtu;
    bool                  mqtt_connected;
    uint8_t               aligned_opennear_ch; /* Channel synchronized with Wi-Fi AP */

    /* Bidirectional bridge queues */
    on_gw_ring_t          uplink_q;           /* OpenNear -> Wi-Fi Cloud / BLE */
    on_gw_ring_t          downlink_q;         /* Wi-Fi Cloud / BLE -> OpenNear */

    /* Counters */
    uint16_t              tx_seq;
    uint32_t              uplink_forwarded;
    uint32_t              downlink_forwarded;
    uint32_t              ble_cmds_parsed;
    uint32_t              wifi_prov_count;
} on_gw_ctx_t;

/* Wire codec for ON_FRAME_GW_BRIDGE */
on_err_t on_gw_bridge_pack(const on_gw_bridge_msg_t *m, uint8_t *out, size_t cap, size_t *len);
on_err_t on_gw_bridge_unpack(const uint8_t *in, size_t len, on_gw_bridge_msg_t *m);

/* Initialization & configuration */
void     on_gw_init(on_gw_ctx_t *gw, on_gw_mode_t mode, const on_gw_transport_ops_t *ops);
void     on_gw_set_wifi_creds(on_gw_ctx_t *gw, const char *ssid, const char *pass,
                              const char *cloud_url);
void     on_gw_set_admin_token(on_gw_ctx_t *gw, const char *token);

/* Link state & Wi-Fi AP channel coexistence hooks */
uint8_t  on_gw_on_wifi_connected(on_gw_ctx_t *gw, uint8_t ap_channel, int8_t rssi,
                                 const uint8_t ip4[4]);
void     on_gw_on_wifi_disconnected(on_gw_ctx_t *gw);
void     on_gw_on_ble_state(on_gw_ctx_t *gw, bool connected, uint16_t mtu);

/* Ring buffer helpers */
bool     on_gw_push_uplink(on_gw_ctx_t *gw, const on_gw_bridge_msg_t *msg);
bool     on_gw_pop_uplink(on_gw_ctx_t *gw, on_gw_bridge_msg_t *out);
bool     on_gw_push_downlink(on_gw_ctx_t *gw, const on_gw_bridge_msg_t *msg);
bool     on_gw_pop_downlink(on_gw_ctx_t *gw, on_gw_bridge_msg_t *out);

/* Parse BLE GATT / Cloud text command ("WIFI:...", "OTA:...", "CMD:...") */
on_err_t on_gw_handle_control_text(on_gw_ctx_t *gw, const char *text, size_t len, bool from_ble);

/* Format an uplink bridge message as JSON for public HTTP/MQTT cloud POST */
size_t   on_gw_format_uplink_json(const on_gw_ctx_t *gw, const on_gw_bridge_msg_t *msg,
                                  int16_t skew_ns, char *out_buf, size_t out_cap);

/* Flush pending uplink queue to public Wi-Fi Cloud & BLE GATT (run on Core 0 / network task) */
uint32_t on_gw_poll_network(on_gw_ctx_t *gw);

#ifdef __cplusplus
}
#endif

#endif /* OPENNEAR_GATEWAY_H */
