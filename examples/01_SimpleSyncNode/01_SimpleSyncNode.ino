/*
 * OpenNear SDK Example 01: Simple Self-Healing Sync Node (10 Lines of Code)
 *
 * Auto-joins Primary G (1-hop), Sub-G (2-hop wall-blocked), or elects itself
 * as Primary G in leaderless mode. Sends custom telemetry & handles commands.
 */

#include <OpenNear.h>

static const int RELAY_PIN = 26;

static void onCustomCommand(const char *cmd, size_t len) {
    Serial.printf("[MyNode] RX Command (%u B): %.*s\n", (unsigned)len, (int)len, cmd);
    if (strncmp(cmd, "RELAY:1", len) == 0) {
        digitalWrite(RELAY_PIN, HIGH);
    } else if (strncmp(cmd, "RELAY:0", len) == 0) {
        digitalWrite(RELAY_PIN, LOW);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    OpenNear.begin(OpenNearConfig::Node()
        .withActGpio(2)
        .onCommand(onCustomCommand)
        .onRoleChange([](on_role_t role, uint16_t id) {
            Serial.printf("[MyNode] Role changed -> role=%d short_id=0x%03X\n", (int)role, (unsigned)id);
        }));
}

void loop() {
    OpenNear.poll();

    static uint32_t last_ms = 0;
    if (millis() - last_ms >= 2000u) {
        last_ms = millis();
        char payload[9];
        snprintf(payload, sizeof(payload), "ADC=%04d", analogRead(34));
        OpenNear.sendTelemetry(payload);
    }
    delay(2);
}
