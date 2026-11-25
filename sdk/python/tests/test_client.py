"""Unit tests for the zero-dependency OpenNear Python SDK using a mock HTTP gateway."""

from __future__ import annotations

import json
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

SDK_ROOT = Path(__file__).resolve().parents[1]
if str(SDK_ROOT) not in sys.path:
    sys.path.insert(0, str(SDK_ROOT))

from opennear import OpenNearAuthError, OpenNearClient, OpenNearError, RuleTrigger, __version__


class _MockGatewayState:
    def __init__(self) -> None:
        self.token: str = "secret123"
        self.commands: List[str] = []
        self.tls_ca_uploads: List[str] = []


def _make_handler(state: _MockGatewayState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
            return

        def _send_json(self, code: int, obj: Dict[str, Any]) -> None:
            raw = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self) -> None:  # noqa: N802
            routes: Dict[str, Dict[str, Any]] = {
                "/api/status": {"ok": True, "fw": "v7.3.0", "role": "GW_PRIMARY", "ha_priority": 1},
                "/api/scan": {"aps": [{"ssid": "LabWiFi", "rssi": -48}]},
                "/api/topology": {"nodes": [{"id": "0x1002", "hops": 1, "rssi": -55}]},
                "/api/data": {"items": [{"id": "0x1002", "payload": "T=24.5"}]},
                "/api/aliases": {"aliases": {"AA:BB:CC:DD:EE:01": "living_room_gw"}},
                "/api/tags": {"tags": {"0x1002": "lights,living_room"}},
                "/api/locate": {
                    "anchors": [{"mac": "AA:BB:CC:DD:EE:01", "room": "living_room", "x_cm": 0, "y_cm": 0}],
                    "fences": [{"name": "desk", "x_cm": 200, "y_cm": 150, "r_cm": 100}],
                    "csi": {"score": 18, "motion": False},
                    "rules": [],
                },
            }
            if self.path in routes:
                self._send_json(200, routes[self.path])
            else:
                self._send_json(404, {"ok": False, "error": "not_found"})

        def do_POST(self) -> None:  # noqa: N802
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length).decode("utf-8")
            hdr_tok = self.headers.get("X-OpenNear-Token", "")
            if state.token and hdr_tok != state.token:
                self._send_json(401, {"ok": False, "error": "unauthorized"})
                return

            if self.path == "/api/cmd":
                if body == "FAIL_ME":
                    self._send_json(400, {"ok": False, "error": "bad_cmd"})
                    return
                state.commands.append(body)
                if body.startswith("TOKEN:"):
                    state.token = body[len("TOKEN:"):]
                self._send_json(200, {"ok": True})
            elif self.path == "/api/tls_ca":
                state.tls_ca_uploads.append(body)
                self._send_json(200, {"ok": True, "ca_len": len(body)})
            else:
                self._send_json(404, {"ok": False, "error": "not_found"})

    return Handler


class TestOpenNearPythonSDK(unittest.TestCase):
    def setUp(self) -> None:
        self.state = _MockGatewayState()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), _make_handler(self.state))
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.client = OpenNearClient(f"http://127.0.0.1:{self.port}", token="secret123")

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    def test_version_and_get_endpoints(self) -> None:
        self.assertEqual(__version__, "7.3.0")
        self.assertEqual(self.client.status()["role"], "GW_PRIMARY")
        self.assertEqual(self.client.scan_wifi()["aps"][0]["ssid"], "LabWiFi")
        self.assertEqual(self.client.topology()["nodes"][0]["id"], "0x1002")
        self.assertEqual(self.client.data()["items"][0]["payload"], "T=24.5")
        self.assertIn("AA:BB:CC:DD:EE:01", self.client.aliases()["aliases"])
        self.assertIn("0x1002", self.client.tags()["tags"])
        self.assertEqual(self.client.locate()["fences"][0]["name"], "desk")

    def test_auth_rejection(self) -> None:
        bad = OpenNearClient(f"http://127.0.0.1:{self.port}", token="wrong")
        with self.assertRaises(OpenNearAuthError) as ctx:
            bad.start_demo()
        self.assertEqual(ctx.exception.status_code, 401)

    def test_control_commands(self) -> None:
        self.client.set_wifi("MySSID", "MyPass", "https://hook.local/ingest")
        self.client.set_mqtt("broker.local", 8883, "u1", "p1", tls=True)
        self.client.set_netkey("0123456789abcdef0123456789abcdef")
        self.client.set_alias("0x1002", "Lamp")
        self.client.set_tags("0x1002", ["lights", "living_room"])
        self.client.send_cmd(0x1002, "LED_ON")
        self.client.send_tag("lights", "BLINK")
        self.client.set_anchor("AA:BB:CC:DD:EE:01", "living_room", 100, 200)
        self.client.remove_anchor("AA:BB:CC:DD:EE:01")
        self.client.set_fence("desk", 220, 150, 120)
        self.client.remove_fence("desk")
        self.client.set_rule(1, RuleTrigger.IN_FENCE, zone="desk", csi_threshold=45, tag="lights", cmd="LED_ON")
        self.client.remove_rule(1)
        self.client.set_ble_scan(True)
        self.client.start_demo()
        self.client.step_demo()
        self.client.stop_demo()
        self.client.trigger_ota(0xFFFF, "7.3.1", "http://192.168.1.2/fw.bin")
        self.client.reset_wifi()
        self.client.reboot()

        self.assertEqual(
            self.state.commands,
            [
                "WIFI:MySSID|MyPass|https://hook.local/ingest",
                "MQTT:broker.local|8883|u1|p1|1",
                "NETKEY:0123456789abcdef0123456789abcdef",
                "ALIAS:0x1002|Lamp",
                "TAG:0x1002|lights,living_room",
                "CMD:1002|LED_ON",
                "CMD_TAG:lights|BLINK",
                "ANCHOR:AA:BB:CC:DD:EE:01|living_room|100|200",
                "ANCHOR:AA:BB:CC:DD:EE:01||0|0",
                "FENCE:desk|220|150|120",
                "FENCE:desk|0|0|0",
                "RULE:1|3|desk|45|lights|LED_ON",
                "RULE:1|0||40|*|",
                "BLESCAN:1",
                "LOCDEMO:1",
                "LOCDEMO:2",
                "LOCDEMO:0",
                "OTA:FFFF|7.3.1|http://192.168.1.2/fw.bin",
                "RESET_WIFI",
                "REBOOT",
            ],
        )

    def test_token_rotation_and_tls_ca_and_errors(self) -> None:
        self.client.set_token("rotated456")
        self.assertEqual(self.client.token, "rotated456")
        self.client.upload_tls_ca("-----BEGIN CERTIFICATE-----\nABC\n-----END CERTIFICATE-----\n")
        self.client.clear_tls_ca()
        self.assertEqual(len(self.state.tls_ca_uploads), 2)
        self.assertEqual(self.state.tls_ca_uploads[1], "")

        with self.assertRaises(OpenNearError) as ctx:
            self.client.raw_cmd("FAIL_ME")
        self.assertEqual(ctx.exception.status_code, 400)

    def test_input_validation(self) -> None:
        with self.assertRaises(ValueError):
            self.client.set_rule(9, RuleTrigger.IN_FENCE)
        with self.assertRaises(ValueError):
            self.client.set_netkey("short")
        with self.assertRaises(ValueError):
            self.client.set_wifi("bad|ssid", "pass")


if __name__ == "__main__":
    unittest.main()
