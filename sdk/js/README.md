# OpenNear JavaScript / TypeScript SDK (`v7.3.0`)

Zero-dependency ESM SDK for **Node.js 18+** and **modern browsers** using the standard `fetch` API, complete with TypeScript `.d.ts` definitions.

## Install

```bash
npm install ./sdk/js
```

## Quick Start

```js
import { OpenNearClient, RuleTrigger } from "opennear-sdk";

const gw = new OpenNearClient("http://192.168.4.1", {
  token: "my-secret-token",
  timeoutMs: 5000,
});

// Query gateway health, mesh topology, and live 2D positions
const status = await gw.getStatus();
const locate = await gw.getLocate();
console.log(status.role, locate.fences);

// Configure a 2D geofence and an offline automation rule (auto-synced to all standby gateways)
await gw.setAnchor("AA:BB:CC:DD:EE:01", "living_room", 0, 0);
await gw.setFence("desk_zone", 220, 150, 120);
await gw.setRule(1, RuleTrigger.IN_FENCE, {
  zone: "desk_zone",
  tag: "lights",
  cmd: "LED_ON",
});

// Trigger tag group or single node
await gw.sendTag("lights", "BLINK");
await gw.sendCmd(0x1002, "LED_OFF");
```

## Run Unit Tests

```bash
node --test sdk/js/test/client.test.mjs
```
