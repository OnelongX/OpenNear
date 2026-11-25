/*
 * OpenNear SDK Example 02: Zero-Config Multi-Gateway with Embedded Web Portal
 *
 * Flash this identical sketch onto 2~4 ESP32 boards:
 *   - Configure Wi-Fi / MQTT once on Board #1 via http://192.168.4.1
 *   - Power on Board #2..#4 -> they automatically join, clone Wi-Fi/MQTT/rules
 *     over AES-128-CCM (0x05 WIFI_PROV), and take over in 150 ms if Board #1 fails!
 */

#include <OpenNear.h>

void setup() {
    Serial.begin(115200);

    OpenNear.begin(OpenNearConfig::Gateway()
        .withBleName("MySmartHome-GW")
        .onCommand([](const char *cmd, size_t len) {
            Serial.printf("[MyGW] Local/Group CMD: %.*s\n", (int)len, cmd);
        })
        .onRoleChange([](on_role_t role, uint16_t id) {
            Serial.printf("[MyGW] Role -> %s (0x%03X)\n",
                          role == ON_ROLE_G ? "ACTIVE_PRIMARY_G" : "HOT_STANDBY_GW",
                          (unsigned)id);
        }));
}

void loop() {
    OpenNear.poll();
    delay(2);
}
