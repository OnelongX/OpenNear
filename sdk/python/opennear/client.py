"""Zero-dependency HTTP REST client for OpenNear v7.3+ Gateways."""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from enum import IntEnum
from typing import Any, Dict, Iterable, Optional, Union

__version__ = "7.3.0"


class OpenNearError(RuntimeError):
    """Base exception raised when an OpenNear Gateway request fails."""

    def __init__(self, message: str, status_code: Optional[int] = None, body: Any = None) -> None:
        super().__init__(message)
        self.status_code = status_code
        self.body = body


class OpenNearAuthError(OpenNearError):
    """Raised when the gateway rejects a control request with HTTP 401 Unauthorized."""


class RuleTrigger(IntEnum):
    """Offline edge automation trigger types supported by OpenNear v7.2+ gateways."""

    DELETE = 0
    ENTER_ROOM = 1
    LEAVE_ROOM = 2
    IN_FENCE = 3
    OUT_FENCE = 4
    CSI_MOTION = 5
    CSI_QUIET = 6


def _format_node_id(target_id: Union[int, str]) -> str:
    if isinstance(target_id, int):
        if target_id < 0 or target_id > 0xFFFF:
            raise ValueError(f"target_id out of 16-bit range: {target_id}")
        return f"{target_id:04X}"
    s = target_id.strip()
    if s.lower().startswith("0x"):
        s = s[2:]
    if not s:
        raise ValueError("target_id must not be empty")
    return s.upper()


def _validate_no_pipe_or_newline(name: str, value: str) -> str:
    if "|" in value or "\n" in value or "\r" in value:
        raise ValueError(f"{name} must not contain '|', '\\r', or '\\n'")
    return value


class OpenNearClient:
    """Synchronous zero-dependency client for the OpenNear Gateway HTTP REST API.

    Parameters
    ----------
    base_url:
        Gateway URL, e.g. ``"http://192.168.4.1"`` or ``"http://192.168.1.50:80"``.
    token:
        Optional API access token sent in the ``X-OpenNear-Token`` header for
        ``POST /api/cmd`` and ``POST /api/tls_ca``.
    timeout:
        Socket timeout in seconds (default ``5.0``).
    """

    def __init__(
        self,
        base_url: str = "http://192.168.4.1",
        token: Optional[str] = None,
        timeout: float = 5.0,
    ) -> None:
        if not base_url:
            raise ValueError("base_url must not be empty")
        if "://" not in base_url:
            base_url = f"http://{base_url}"
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.timeout = float(timeout)

    def _request_json(
        self,
        method: str,
        path: str,
        body: Optional[str] = None,
        auth: bool = False,
    ) -> Dict[str, Any]:
        url = f"{self.base_url}{path}"
        headers: Dict[str, str] = {"Accept": "application/json"}
        data: Optional[bytes] = None
        if body is not None:
            data = body.encode("utf-8")
            headers["Content-Type"] = "text/plain; charset=utf-8"
        if auth and self.token:
            headers["X-OpenNear-Token"] = self.token

        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
                status_code = resp.status
        except urllib.error.HTTPError as exc:
            raw_err = exc.read().decode("utf-8", errors="replace")
            parsed_err: Any = raw_err
            try:
                parsed_err = json.loads(raw_err)
            except ValueError:
                pass
            if exc.code == 401:
                raise OpenNearAuthError(
                    f"Unauthorized ({path}): check X-OpenNear-Token",
                    status_code=401,
                    body=parsed_err,
                ) from exc
            raise OpenNearError(
                f"HTTP {exc.code} on {method} {path}: {raw_err}",
                status_code=exc.code,
                body=parsed_err,
            ) from exc
        except urllib.error.URLError as exc:
            raise OpenNearError(f"Connection failed for {url}: {exc.reason}") from exc

        try:
            payload = json.loads(raw) if raw.strip() else {}
        except ValueError as exc:
            raise OpenNearError(
                f"Invalid JSON from {method} {path}: {raw[:200]}",
                status_code=status_code,
                body=raw,
            ) from exc

        if isinstance(payload, dict) and payload.get("ok") is False:
            raise OpenNearError(
                f"Gateway rejected {path}: {payload.get('error', payload)}",
                status_code=status_code,
                body=payload,
            )
        return payload

    # ------------------------------------------------------------------
    # Read-only telemetry & discovery endpoints (GET)
    # ------------------------------------------------------------------

    def status(self) -> Dict[str, Any]:
        """GET /api/status — gateway role, Wi-Fi/MQTT state, sync stats, HA priority."""
        return self._request_json("GET", "/api/status")

    def scan_wifi(self) -> Dict[str, Any]:
        """GET /api/scan — nearby 2.4 GHz Wi-Fi AP scan list."""
        return self._request_json("GET", "/api/scan")

    def topology(self) -> Dict[str, Any]:
        """GET /api/topology — live mesh nodes, parent hops, RSSI, roles."""
        return self._request_json("GET", "/api/topology")

    def data(self) -> Dict[str, Any]:
        """GET /api/data — latest sensor telemetry reported by mesh nodes."""
        return self._request_json("GET", "/api/data")

    def aliases(self) -> Dict[str, Any]:
        """GET /api/aliases — MAC/NodeID to human-readable alias map."""
        return self._request_json("GET", "/api/aliases")

    def tags(self) -> Dict[str, Any]:
        """GET /api/tags — node tag groups."""
        return self._request_json("GET", "/api/tags")

    def locate(self) -> Dict[str, Any]:
        """GET /api/locate — 2D trilateration positions, anchors, geofences, CSI radar & rules."""
        return self._request_json("GET", "/api/locate")

    # ------------------------------------------------------------------
    # Control plane (POST /api/cmd & POST /api/tls_ca)
    # ------------------------------------------------------------------

    def raw_cmd(self, command: str) -> Dict[str, Any]:
        """POST /api/cmd with a raw text command (e.g. ``'LOCDEMO:1'``)."""
        if not command or not command.strip():
            raise ValueError("command must not be empty")
        return self._request_json("POST", "/api/cmd", body=command.strip(), auth=True)

    def set_wifi(self, ssid: str, password: str, cloud_url: str = "") -> Dict[str, Any]:
        """Configure Wi-Fi STA credentials (and optional cloud webhook URL).

        Automatically air-synced to all N-1 standby gateways when active.
        """
        _validate_no_pipe_or_newline("ssid", ssid)
        _validate_no_pipe_or_newline("password", password)
        _validate_no_pipe_or_newline("cloud_url", cloud_url)
        cmd = f"WIFI:{ssid}|{password}"
        if cloud_url:
            cmd = f"{cmd}|{cloud_url}"
        return self.raw_cmd(cmd)

    def set_mqtt(
        self,
        host: str,
        port: int = 1883,
        user: str = "",
        password: str = "",
        tls: bool = False,
    ) -> Dict[str, Any]:
        """Configure MQTT/MQTTS broker and Home Assistant MQTT Auto-Discovery."""
        _validate_no_pipe_or_newline("host", host)
        _validate_no_pipe_or_newline("user", user)
        _validate_no_pipe_or_newline("password", password)
        if not (1 <= int(port) <= 65535):
            raise ValueError(f"port out of range: {port}")
        return self.raw_cmd(f"MQTT:{host}|{int(port)}|{user}|{password}|{1 if tls else 0}")

    def set_token(self, new_token: str) -> Dict[str, Any]:
        """Set or clear the gateway HTTP API ``X-OpenNear-Token`` and update this client."""
        _validate_no_pipe_or_newline("new_token", new_token)
        res = self.raw_cmd(f"TOKEN:{new_token}")
        self.token = new_token or None
        return res

    def set_netkey(self, hex_key_32: str = "") -> Dict[str, Any]:
        """Set a 32-hex-char (128-bit) ChaCha20+SipHash mesh network key, or empty to reset."""
        key = hex_key_32.strip()
        if key:
            if len(key) != 32:
                raise ValueError("hex_key_32 must be empty or exactly 32 hex characters")
            int(key, 16)
        return self.raw_cmd(f"NETKEY:{key}")

    def set_alias(self, mac_or_id: str, alias: str) -> Dict[str, Any]:
        """Assign a human-readable alias to a node or BLE device."""
        _validate_no_pipe_or_newline("mac_or_id", mac_or_id)
        _validate_no_pipe_or_newline("alias", alias)
        return self.raw_cmd(f"ALIAS:{mac_or_id}|{alias}")

    def set_tags(self, mac_or_id: str, tags: Union[str, Iterable[str]]) -> Dict[str, Any]:
        """Assign comma-separated tags (e.g. ``['lights', 'living_room']``) to a node."""
        _validate_no_pipe_or_newline("mac_or_id", mac_or_id)
        if isinstance(tags, str):
            tag_str = tags
        else:
            tag_str = ",".join(t.strip() for t in tags if t and t.strip())
        _validate_no_pipe_or_newline("tags", tag_str)
        return self.raw_cmd(f"TAG:{mac_or_id}|{tag_str}")

    def send_cmd(self, target_id: Union[int, str], payload: str) -> Dict[str, Any]:
        """Send a point-to-point downlink command to a specific 16-bit node ID (e.g. ``0x1002``)."""
        _validate_no_pipe_or_newline("payload", payload)
        return self.raw_cmd(f"CMD:{_format_node_id(target_id)}|{payload}")

    def send_tag(self, tag: str, payload: str) -> Dict[str, Any]:
        """Broadcast a downlink command to all nodes carrying ``tag`` (or ``'*'`` for all)."""
        _validate_no_pipe_or_newline("tag", tag)
        _validate_no_pipe_or_newline("payload", payload)
        return self.raw_cmd(f"CMD_TAG:{tag}|{payload}")

    def set_anchor(
        self,
        mac_or_id: str,
        room: str,
        x_cm: int = 0,
        y_cm: int = 0,
    ) -> Dict[str, Any]:
        """Register or update a fixed positioning anchor coordinate (centimeters)."""
        _validate_no_pipe_or_newline("mac_or_id", mac_or_id)
        _validate_no_pipe_or_newline("room", room)
        return self.raw_cmd(f"ANCHOR:{mac_or_id}|{room}|{int(x_cm)}|{int(y_cm)}")

    def remove_anchor(self, mac_or_id: str) -> Dict[str, Any]:
        """Delete a registered positioning anchor by leaving its room empty."""
        _validate_no_pipe_or_newline("mac_or_id", mac_or_id)
        return self.raw_cmd(f"ANCHOR:{mac_or_id}||0|0")

    def set_fence(self, name: str, x_cm: int, y_cm: int, radius_cm: int) -> Dict[str, Any]:
        """Create or update a circular 2D geofence (centimeters)."""
        _validate_no_pipe_or_newline("name", name)
        if int(radius_cm) < 0:
            raise ValueError("radius_cm must be >= 0")
        return self.raw_cmd(f"FENCE:{name}|{int(x_cm)}|{int(y_cm)}|{int(radius_cm)}")

    def remove_fence(self, name: str) -> Dict[str, Any]:
        """Delete a 2D geofence by setting radius_cm to 0."""
        return self.set_fence(name, 0, 0, 0)

    def set_rule(
        self,
        rule_id: int,
        trigger: Union[RuleTrigger, int],
        zone: str = "",
        csi_threshold: int = 40,
        tag: str = "*",
        cmd: str = "",
    ) -> Dict[str, Any]:
        """Create or update an offline edge automation rule (``rule_id`` in ``1..8``)."""
        rid = int(rule_id)
        if not (1 <= rid <= 8):
            raise ValueError("rule_id must be between 1 and 8")
        trig_val = int(trigger)
        if not (0 <= trig_val <= 6):
            raise ValueError("trigger must be between 0 and 6 (see RuleTrigger)")
        _validate_no_pipe_or_newline("zone", zone)
        _validate_no_pipe_or_newline("tag", tag)
        _validate_no_pipe_or_newline("cmd", cmd)
        return self.raw_cmd(f"RULE:{rid}|{trig_val}|{zone}|{int(csi_threshold)}|{tag}|{cmd}")

    def remove_rule(self, rule_id: int) -> Dict[str, Any]:
        """Delete an offline automation rule (``rule_id`` in ``1..8``)."""
        return self.set_rule(rule_id, RuleTrigger.DELETE)

    def set_ble_scan(self, enabled: bool) -> Dict[str, Any]:
        """Enable or disable gateway BLE iBeacon / tag passive scanning."""
        return self.raw_cmd(f"BLESCAN:{1 if enabled else 0}")

    def start_demo(self) -> Dict[str, Any]:
        """Start the built-in interactive 2D positioning + geofence + CSI radar demo."""
        return self.raw_cmd("LOCDEMO:1")

    def step_demo(self) -> Dict[str, Any]:
        """Advance the interactive demo by one deterministic scenario step."""
        return self.raw_cmd("LOCDEMO:2")

    def stop_demo(self) -> Dict[str, Any]:
        """Stop the interactive demo and clear simulated demo state."""
        return self.raw_cmd("LOCDEMO:0")

    def trigger_ota(self, target_id: Union[int, str], version: str, url: str) -> Dict[str, Any]:
        """Trigger a synchronized Mesh OTA update for ``target_id`` (use ``0xFFFF`` for broadcast)."""
        _validate_no_pipe_or_newline("version", version)
        _validate_no_pipe_or_newline("url", url)
        return self.raw_cmd(f"OTA:{_format_node_id(target_id)}|{version}|{url}")

    def upload_tls_ca(self, pem_cert: str) -> Dict[str, Any]:
        """POST /api/tls_ca — upload a PEM root CA certificate for MQTTS verification."""
        return self._request_json("POST", "/api/tls_ca", body=pem_cert, auth=True)

    def clear_tls_ca(self) -> Dict[str, Any]:
        """POST /api/tls_ca with empty body to clear the stored custom CA certificate."""
        return self._request_json("POST", "/api/tls_ca", body="", auth=True)

    def reset_wifi(self) -> Dict[str, Any]:
        """Clear saved Wi-Fi credentials and return gateway to Captive Portal AP mode."""
        return self.raw_cmd("RESET_WIFI")

    def reboot(self) -> Dict[str, Any]:
        """Reboot the gateway."""
        return self.raw_cmd("REBOOT")
