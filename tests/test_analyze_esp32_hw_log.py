"""Unit tests for tools/analyze_esp32_hw_log.py and failover regex compatibility."""

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests"))

import analyze_esp32_hw_log as hwlog
import test_esp32_failover_hw as failover_hw


HEALTHY_V63_LOG = """
0000.100 [OpenNear Portal] SoftAP 'OpenNear-Setup-A1B2' active -> connect & open http://192.168.4.1
0000.200 [OpenNear] ESP32-WROOM-32 booted as PRIMARY G (0x001) on Wi-Fi ch=6 (54M 0xD0, RTT off, backup after 12 missed beacons and 150 ms silence)
0004.500 [OpenNear GW] Wi-Fi ONLINE http://192.168.1.55/ ch=6 rssi=-48
0005.100 [OpenNear MQTT] Connected to mqtts://broker.emqx.io:8883 (HA Discovery + cmd/# armed)
0009.550 [OpenNear Portal] Fallback SoftAP closed; Web Portal stays live on LAN STA IP
0010.200 [OpenNear OTA] Boot confirmed on partition ota_0 (rollback cancelled)
0011.000 [OpenNear v6.2] SF=2750 role=1 id=0x001 hop=0 slot=0 nco=0.00ppm rtt=0ns harq=0 acts=2750
0011.005 [OpenNear TX] accepted=2750 failed=3
0011.010 [OpenNear Link] sync_rx=0 data_tx=0 data_rx=5400 missed=0 takeovers=0 backup=0x002 rejoins=0 leases=0
0012.000 [OpenNear Cluster] {"sf":3000,"g":"0x001","backup":"0x002","sub_g":"0x003","ch_map":"0xFFFF","cached":0,"count":10,"nodes":[]}
""".strip().splitlines()


FAULTY_TERMINAL_LOG = """
0000.100 [OpenNear] ESP32-WROOM-32 booted as AUTO-JOIN TERMINAL on Wi-Fi ch=6
0001.000 [OpenNear v6.2] SF=250 role=2 id=0x002 hop=1 slot=0 nco=-18.50ppm rtt=0ns harq=2 acts=250
0001.005 [OpenNear TX] accepted=200 failed=40
0001.010 [OpenNear Link] sync_rx=240 data_tx=200 data_rx=0 missed=9 takeovers=0 backup=0x002 rejoins=3 leases=0
0002.000 [OpenNear v6.2] SF=500 role=2 id=0x002 hop=1 slot=0 nco=-45.20ppm rtt=0ns harq=5 acts=480
0002.005 [OpenNear TX] accepted=300 failed=80
0002.010 [OpenNear Link] sync_rx=460 data_tx=300 data_rx=0 missed=10 takeovers=0 backup=0x002 rejoins=4 leases=0
0003.100 Brownout detector was triggered
0004.000 [OpenNear] ESP32-WROOM-32 booted as AUTO-JOIN TERMINAL on Wi-Fi ch=6
""".strip().splitlines()


class AnalyzeEsp32HwLogTests(unittest.TestCase):
    def test_healthy_v63_gateway_log(self):
        ba = hwlog.parse_log_lines(HEALTHY_V63_LOG, "healthy_gw.log")
        s = ba.summarize()
        self.assertEqual(s["health_score"], 100)
        self.assertEqual(s["boot_count"], 1)
        self.assertEqual(s["fatal_count"], 0)
        self.assertEqual(s["last_id"], "0x001")
        self.assertEqual(s["last_cluster_nodes"], 10)
        self.assertTrue(s["gateway"]["ota_boot_confirmed"])
        self.assertEqual(s["gateway"]["wifi_online_events"], 1)
        self.assertEqual(s["gateway"]["mqtt_connect_events"], 1)
        self.assertEqual(s["findings"][0]["code"], "HEALTHY")
        md = hwlog.render_markdown([s])
        self.assertIn("healthy_gw.log", md)
        self.assertIn("100/100", md)

    def test_faulty_log_detects_root_causes(self):
        ba = hwlog.parse_log_lines(FAULTY_TERMINAL_LOG, "faulty_node.log")
        s = ba.summarize()
        codes = {f["code"] for f in s["findings"]}
        self.assertLess(s["health_score"], 40)
        self.assertIn("FATAL_CRASH", codes)
        self.assertIn("MULTI_REBOOT", codes)
        self.assertIn("HIGH_TX_FAIL", codes)
        self.assertIn("BEACON_MISS_BURST", codes)
        self.assertIn("REJOIN_CHURN", codes)

    def test_failover_hw_regex_matches_v3_and_v62(self):
        pat = failover_hw.PATTERNS[0]
        m3 = pat.search("[OpenNear v3.0] SF=100 role=1 id=0x001 hop=0 slot=0")
        m6 = pat.search("[OpenNear v6.5] SF=200 role=2 id=0x004 hop=1 slot=2")
        self.assertIsNotNone(m3)
        self.assertIsNotNone(m6)
        self.assertEqual(m6.groups(), ("200", "2", "0x004"))

    def test_coex_modem_sleep_abort_detected(self):
        lines = [
            "0000.100 [OpenNear Portal] SoftAP 'OpenNear-Setup-1234' active -> connect & open http://192.168.4.1",
            "0000.250 Error! Should enable WiFi modem sleep when both WiFi and Bluetooth are enabled!!!!!!",
            "0000.255 abort() was called at PC 0x40123456 on core 0",
        ]
        s = hwlog.parse_log_lines(lines, "coex_crash.log").summarize()
        codes = {f["code"] for f in s["findings"]}
        self.assertIn("FATAL_CRASH", codes)
        self.assertGreaterEqual(s["fatal_count"], 2)


if __name__ == "__main__":
    unittest.main()
