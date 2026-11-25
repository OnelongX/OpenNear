/*
 * OpenNear SDK Example 03: Offline 2D Geofence + Wi-Fi CSI Radar Callback
 *
 * Runs 100% locally on ESP32 with zero cloud / zero Home Assistant dependency.
 * Fires your C++ callback whenever a target enters a 2D circular geofence or
 * when IEEE 802.11bf Wi-Fi CSI detects human movement in a room.
 */

#include <OpenNear.h>

static void onEdgeRuleFired(uint8_t rule_id, const char *zone, const char *tag, const char *cmd) {
    Serial.printf("[EdgeRule] R#%u triggered in '%s' -> tag='%s' cmd='%s'\n",
                  (unsigned)rule_id, zone, tag, cmd);
}

void setup() {
    Serial.begin(115200);

    if (OpenNear.begin(OpenNearConfig::Gateway().onRuleFired(onEdgeRuleFired))) {
        /* Define a 1.4m circular geofence around StudyDesk at (4.7m, 0.5m) */
        OpenNear.setFence("StudyDesk", 470, 50, 140);

        /* Rule #1: Entering StudyDesk turns on DeskLamp relay offline */
        OpenNear.setRule(1, ON_RULE_TRIG_IN_FENCE, "StudyDesk", 0, "*", "RELAY:1");

        /* Rule #2: Wi-Fi CSI human motion >= 45 in Living triggers welcome pulse */
        OpenNear.setRule(2, ON_RULE_TRIG_CSI_MOTION, "Living", 45, "*", "MODE:PULSE");
    }
}

void loop() {
    OpenNear.poll();
    delay(2);
}
