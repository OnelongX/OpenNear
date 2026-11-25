import test from "node:test";
import assert from "node:assert/strict";
import http from "node:http";
import {
  OpenNearAuthError,
  OpenNearClient,
  OpenNearError,
  RuleTrigger,
  SDK_VERSION,
} from "../src/index.mjs";

function startMockGateway() {
  const state = {
    token: "secret123",
    commands: [],
    tlsCaUploads: [],
  };

  const server = http.createServer((req, res) => {
    const sendJson = (code, obj) => {
      const payload = JSON.stringify(obj);
      res.writeHead(code, {
        "Content-Type": "application/json",
        "Content-Length": Buffer.byteLength(payload),
      });
      res.end(payload);
    };

    if (req.method === "GET") {
      const routes = {
        "/api/status": { ok: true, fw: "v7.3.0", role: "GW_PRIMARY", ha_priority: 1 },
        "/api/scan": { aps: [{ ssid: "LabWiFi", rssi: -48 }] },
        "/api/topology": { nodes: [{ id: "0x1002", hops: 1, rssi: -55 }] },
        "/api/data": { items: [{ id: "0x1002", payload: "T=24.5" }] },
        "/api/aliases": { aliases: { "AA:BB:CC:DD:EE:01": "living_room_gw" } },
        "/api/tags": { tags: { "0x1002": "lights,living_room" } },
        "/api/locate": {
          anchors: [{ mac: "AA:BB:CC:DD:EE:01", room: "living_room", x_cm: 0, y_cm: 0 }],
          fences: [{ name: "desk", x_cm: 200, y_cm: 150, r_cm: 100 }],
          csi: { score: 18, motion: false },
          rules: [],
        },
      };
      if (req.url in routes) {
        sendJson(200, routes[req.url]);
      } else {
        sendJson(404, { ok: false, error: "not_found" });
      }
      return;
    }

    if (req.method === "POST") {
      let body = "";
      req.on("data", (chunk) => {
        body += chunk.toString("utf-8");
      });
      req.on("end", () => {
        const hdrTok = req.headers["x-opennear-token"] ?? "";
        if (state.token && hdrTok !== state.token) {
          sendJson(401, { ok: false, error: "unauthorized" });
          return;
        }
        if (req.url === "/api/cmd") {
          if (body === "FAIL_ME") {
            sendJson(400, { ok: false, error: "bad_cmd" });
            return;
          }
          state.commands.push(body);
          if (body.startsWith("TOKEN:")) {
            state.token = body.slice("TOKEN:".length);
          }
          sendJson(200, { ok: true });
          return;
        }
        if (req.url === "/api/tls_ca") {
          state.tlsCaUploads.push(body);
          sendJson(200, { ok: true, ca_len: body.length });
          return;
        }
        sendJson(404, { ok: false, error: "not_found" });
      });
    }
  });

  return new Promise((resolve) => {
    server.listen(0, "127.0.0.1", () => {
      const { port } = server.address();
      resolve({
        state,
        baseUrl: `http://127.0.0.1:${port}`,
        close: () => new Promise((r) => server.close(r)),
      });
    });
  });
}

test("OpenNear JS SDK GET endpoints and version", async () => {
  const gw = await startMockGateway();
  try {
    assert.equal(SDK_VERSION, "7.3.0");
    const client = new OpenNearClient(gw.baseUrl, { token: "secret123" });
    assert.equal((await client.getStatus()).role, "GW_PRIMARY");
    assert.equal((await client.scanWifi()).aps[0].ssid, "LabWiFi");
    assert.equal((await client.getTopology()).nodes[0].id, "0x1002");
    assert.equal((await client.getData()).items[0].payload, "T=24.5");
    assert.equal((await client.getAliases()).aliases["AA:BB:CC:DD:EE:01"], "living_room_gw");
    assert.equal((await client.getTags()).tags["0x1002"], "lights,living_room");
    assert.equal((await client.getLocate()).fences[0].name, "desk");
  } finally {
    await gw.close();
  }
});

test("OpenNear JS SDK rejects unauthorized token with OpenNearAuthError", async () => {
  const gw = await startMockGateway();
  try {
    const bad = new OpenNearClient(gw.baseUrl, { token: "wrong" });
    await assert.rejects(() => bad.startDemo(), (err) => {
      assert.ok(err instanceof OpenNearAuthError);
      assert.equal(err.statusCode, 401);
      return true;
    });
  } finally {
    await gw.close();
  }
});

test("OpenNear JS SDK control commands, token rotation, TLS CA & validation", async () => {
  const gw = await startMockGateway();
  try {
    const client = new OpenNearClient(gw.baseUrl, { token: "secret123" });
    await client.setWifi("MySSID", "MyPass", "https://hook.local/ingest");
    await client.setMqtt("broker.local", { port: 8883, user: "u1", password: "p1", tls: true });
    await client.setNetKey("0123456789abcdef0123456789abcdef");
    await client.setAlias("0x1002", "Lamp");
    await client.setTags("0x1002", ["lights", "living_room"]);
    await client.sendCmd(0x1002, "LED_ON");
    await client.sendTag("lights", "BLINK");
    await client.setAnchor("AA:BB:CC:DD:EE:01", "living_room", 100, 200);
    await client.removeAnchor("AA:BB:CC:DD:EE:01");
    await client.setFence("desk", 220, 150, 120);
    await client.removeFence("desk");
    await client.setRule(1, RuleTrigger.IN_FENCE, {
      zone: "desk",
      csiThreshold: 45,
      tag: "lights",
      cmd: "LED_ON",
    });
    await client.removeRule(1);
    await client.setBleScan(true);
    await client.startDemo();
    await client.stepDemo();
    await client.stopDemo();
    await client.triggerOta(0xffff, "7.3.1", "http://192.168.1.2/fw.bin");
    await client.resetWifi();
    await client.reboot();

    assert.deepEqual(gw.state.commands, [
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
    ]);

    await client.setToken("rotated456");
    assert.equal(client.token, "rotated456");
    await client.uploadTlsCa("-----BEGIN CERTIFICATE-----\nABC\n-----END CERTIFICATE-----\n");
    await client.clearTlsCa();
    assert.equal(gw.state.tlsCaUploads.length, 2);
    assert.equal(gw.state.tlsCaUploads[1], "");

    await assert.rejects(() => client.rawCmd("FAIL_ME"), (err) => {
      assert.ok(err instanceof OpenNearError);
      assert.equal(err.statusCode, 400);
      return true;
    });

    assert.throws(() => client.setRule(9, RuleTrigger.IN_FENCE), RangeError);
    assert.throws(() => client.setNetKey("short"), Error);
    assert.throws(() => client.setWifi("bad|ssid", "pass"), Error);
  } finally {
    await gw.close();
  }
});
