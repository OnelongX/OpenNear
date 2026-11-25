# OpenNear Python SDK (`v7.3.0`)

Zero-dependency Python 3.9+ client for controlling and monitoring **OpenNear** ESP32 / multi-gateway wireless mesh networks over the Gateway HTTP REST API.

## Install

```bash
pip install ./sdk/python
# or editable install during development:
pip install -e ./sdk/python
```

No third-party packages are required — only Python's standard library (`urllib.request`, `json`).

## Quick Start

```python
from opennear import OpenNearClient, RuleTrigger

gw = OpenNearClient("http://192.168.4.1", token="my-secret-token")

# Inspect gateway health, mesh topology, and live 2D positions
print(gw.status())
print(gw.topology())
print(gw.locate())

# Configure a 2D geofence and an offline automation rule (synced automatically to all standby gateways)
gw.set_anchor("AA:BB:CC:DD:EE:01", room="living_room", x_cm=0, y_cm=0)
gw.set_fence("desk_zone", x_cm=220, y_cm=150, radius_cm=120)
gw.set_rule(
    rule_id=1,
    trigger=RuleTrigger.IN_FENCE,
    zone="desk_zone",
    tag="lights",
    cmd="LED_ON",
)

# Trigger a tag group or a single node
gw.send_tag("lights", "BLINK")
gw.send_cmd(0x1002, "LED_OFF")
```

## Run Unit Tests

```bash
python -m unittest discover -s sdk/python/tests -v
```
