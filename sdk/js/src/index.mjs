/**
 * OpenNear JavaScript / TypeScript ESM SDK (v7.3.0)
 * Zero-dependency Fetch API client for OpenNear Gateway HTTP REST API.
 */

export const SDK_VERSION = "7.3.0";

/** Offline edge automation trigger types supported by OpenNear v7.2+ gateways. */
export const RuleTrigger = Object.freeze({
  DELETE: 0,
  ENTER_ROOM: 1,
  LEAVE_ROOM: 2,
  IN_FENCE: 3,
  OUT_FENCE: 4,
  CSI_MOTION: 5,
  CSI_QUIET: 6,
});

export class OpenNearError extends Error {
  /**
   * @param {string} message
   * @param {number | null} [statusCode]
   * @param {unknown} [body]
   */
  constructor(message, statusCode = null, body = null) {
    super(message);
    this.name = "OpenNearError";
    this.statusCode = statusCode;
    this.body = body;
  }
}

export class OpenNearAuthError extends OpenNearError {
  /**
   * @param {string} message
   * @param {unknown} [body]
   */
  constructor(message, body = null) {
    super(message, 401, body);
    this.name = "OpenNearAuthError";
  }
}

/**
 * @param {number | string} targetId
 * @returns {string}
 */
function formatNodeId(targetId) {
  let val;
  if (typeof targetId === "number") {
    if (!Number.isInteger(targetId)) {
      throw new RangeError(`targetId must be an integer: ${targetId}`);
    }
    val = targetId;
  } else {
    const s = String(targetId).trim().replace(/^0x/i, "");
    if (!s || !/^[0-9a-fA-F]+$/.test(s)) {
      throw new Error(`Invalid hex targetId: ${targetId}`);
    }
    val = Number.parseInt(s, 16);
  }
  if (val === 0xffff) {
    val = 0x0fff; // Convenience alias for 12-bit ON_SHORT_ID_BROADCAST (0xFFF)
  }
  if (val <= 0 || val > 0x0fff) {
    throw new RangeError(`targetId out of 12-bit OpenNear Short-ID range (0x001..0xFFF): ${targetId}`);
  }
  return val.toString(16).toUpperCase().padStart(3, "0");
}

/**
 * @param {string} mac
 * @returns {string}
 */
function normalizeMac48(mac) {
  validateField("mac", mac);
  const hexOnly = String(mac).trim().replace(/[:-]/g, "");
  if (!/^[0-9a-fA-F]{12}$/.test(hexOnly)) {
    throw new Error(`mac must be a 48-bit MAC address (12 hex digits), got: ${mac}`);
  }
  const up = hexOnly.toUpperCase();
  return `${up.slice(0, 2)}:${up.slice(2, 4)}:${up.slice(4, 6)}:${up.slice(6, 8)}:${up.slice(8, 10)}:${up.slice(10, 12)}`;
}

/**
 * @param {number | string} version
 * @returns {string}
 */
function formatOtaVersion(version) {
  if (typeof version === "number") {
    if (!Number.isInteger(version) || version <= 0 || version > 0xffffffff) {
      throw new RangeError(`version out of 32-bit range: ${version}`);
    }
    return `0x${version.toString(16).toUpperCase().padStart(8, "0")}`;
  }
  const s = validateField("version", String(version).trim());
  if (!s) {
    throw new Error("version must not be empty");
  }
  if (s.includes(".")) {
    const parts = s.replace(/^[vV]/, "").split(".");
    if (parts.length !== 2 && parts.length !== 3) {
      throw new Error(`Invalid semantic version string: ${version}`);
    }
    const major = Number.parseInt(parts[0], 10);
    const minor = Number.parseInt(parts[1], 10);
    const patch = parts.length === 3 ? Number.parseInt(parts[2], 10) : 0;
    if (
      ![major, minor, patch].every((n) => Number.isInteger(n) && n >= 0 && n <= 255)
    ) {
      throw new RangeError(`Semantic version components must be 0..255: ${version}`);
    }
    const u32 = ((major << 16) | (minor << 8) | patch) >>> 0;
    return `0x${u32.toString(16).toUpperCase().padStart(8, "0")}`;
  }
  const val = s.toLowerCase().startsWith("0x")
    ? Number.parseInt(s.slice(2), 16)
    : Number.parseInt(s, 10);
  if (!Number.isInteger(val) || val <= 0 || val > 0xffffffff) {
    throw new RangeError(`version out of 32-bit range: ${version}`);
  }
  return `0x${val.toString(16).toUpperCase().padStart(8, "0")}`;
}

/**
 * @param {string} name
 * @param {string} value
 * @returns {string}
 */
function validateField(name, value) {
  const s = String(value);
  if (s.includes("|") || s.includes("\r") || s.includes("\n")) {
    throw new Error(`${name} must not contain '|', '\\r', or '\\n'`);
  }
  return s;
}

export class OpenNearClient {
  /**
   * @param {string} [baseUrl]
   * @param {{ token?: string | null, timeoutMs?: number, fetchImpl?: typeof fetch }} [options]
   */
  constructor(baseUrl = "http://192.168.4.1", options = {}) {
    if (!baseUrl) {
      throw new Error("baseUrl must not be empty");
    }
    const normalized = baseUrl.includes("://") ? baseUrl : `http://${baseUrl}`;
    this.baseUrl = normalized.replace(/\/+$/, "");
    this.token = options.token ?? null;
    this.timeoutMs = options.timeoutMs ?? 5000;
    this._fetch = options.fetchImpl ?? globalThis.fetch;
    if (typeof this._fetch !== "function") {
      throw new Error("Global fetch() is unavailable; pass options.fetchImpl or use Node.js 18+");
    }
  }

  /**
   * @param {string} method
   * @param {string} path
   * @param {{ body?: string | null, auth?: boolean }} [opts]
   * @returns {Promise<Record<string, any>>}
   */
  async _requestJson(method, path, opts = {}) {
    const { body = null, auth = false } = opts;
    const headers = { Accept: "application/json" };
    if (body !== null) {
      headers["Content-Type"] = "text/plain; charset=utf-8";
    }
    if (auth && this.token) {
      headers["X-OpenNear-Token"] = this.token;
    }

    const controller = new AbortController();
    const timer = this.timeoutMs > 0 ? setTimeout(() => controller.abort(), this.timeoutMs) : null;

    let resp;
    try {
      resp = await this._fetch(`${this.baseUrl}${path}`, {
        method,
        headers,
        body: body !== null ? body : undefined,
        signal: controller.signal,
      });
    } catch (err) {
      throw new OpenNearError(`Connection failed for ${method} ${path}: ${err?.message ?? err}`);
    } finally {
      if (timer !== null) clearTimeout(timer);
    }

    const text = await resp.text();
    let parsed = null;
    try {
      parsed = text.trim() ? JSON.parse(text) : {};
    } catch {
      parsed = text;
    }

    if (resp.status === 401) {
      throw new OpenNearAuthError(`Unauthorized (${path}): check X-OpenNear-Token`, parsed);
    }
    if (!resp.ok) {
      throw new OpenNearError(`HTTP ${resp.status} on ${method} ${path}: ${text}`, resp.status, parsed);
    }
    if (typeof parsed !== "object" || parsed === null) {
      throw new OpenNearError(`Invalid JSON from ${method} ${path}: ${text.slice(0, 200)}`, resp.status, text);
    }
    if (parsed.ok === false) {
      throw new OpenNearError(`Gateway rejected ${path}: ${parsed.error ?? text}`, resp.status, parsed);
    }
    return parsed;
  }

  // ------------------------------------------------------------------
  // Read-only telemetry & discovery endpoints (GET)
  // ------------------------------------------------------------------

  getStatus() {
    return this._requestJson("GET", "/api/status");
  }

  scanWifi() {
    return this._requestJson("GET", "/api/scan");
  }

  getTopology() {
    return this._requestJson("GET", "/api/topology");
  }

  getData() {
    return this._requestJson("GET", "/api/data");
  }

  getAliases() {
    return this._requestJson("GET", "/api/aliases");
  }

  getTags() {
    return this._requestJson("GET", "/api/tags");
  }

  getLocate() {
    return this._requestJson("GET", "/api/locate");
  }

  // ------------------------------------------------------------------
  // Control plane (POST /api/cmd & POST /api/tls_ca)
  // ------------------------------------------------------------------

  /**
   * @param {string} command
   */
  rawCmd(command) {
    const trimmed = String(command ?? "").trim();
    if (!trimmed) {
      throw new Error("command must not be empty");
    }
    return this._requestJson("POST", "/api/cmd", { body: trimmed, auth: true });
  }

  /**
   * @param {string} ssid
   * @param {string} password
   * @param {string} [cloudUrl]
   */
  setWifi(ssid, password, cloudUrl = "") {
    validateField("ssid", ssid);
    validateField("password", password);
    validateField("cloudUrl", cloudUrl);
    const cmd = cloudUrl ? `WIFI:${ssid}|${password}|${cloudUrl}` : `WIFI:${ssid}|${password}`;
    return this.rawCmd(cmd);
  }

  /**
   * @param {string} host
   * @param {{ port?: number, user?: string, password?: string, tls?: boolean }} [opts]
   */
  setMqtt(host, opts = {}) {
    const { port = 1883, user = "", password = "", tls = false } = opts;
    validateField("host", host);
    validateField("user", user);
    validateField("password", password);
    const p = Number(port);
    if (!Number.isInteger(p) || p < 1 || p > 65535) {
      throw new RangeError(`port out of range: ${port}`);
    }
    return this.rawCmd(`MQTT:${host}|${p}|${user}|${password}|${tls ? 1 : 0}`);
  }

  /**
   * @param {string} newToken
   */
  async setToken(newToken) {
    validateField("newToken", newToken);
    const res = await this.rawCmd(`TOKEN:${newToken}`);
    this.token = newToken || null;
    return res;
  }

  /**
   * @param {string} [hexKey32]
   */
  setNetKey(hexKey32 = "") {
    const key = String(hexKey32).trim();
    if (key && !/^[0-9a-fA-F]{32}$/.test(key)) {
      throw new Error("hexKey32 must be empty or exactly 32 hex characters");
    }
    return this.rawCmd(`NETKEY:${key}`);
  }

  /**
   * @param {string} mac
   * @param {string} alias
   */
  setAlias(mac, alias) {
    const normMac = normalizeMac48(mac);
    validateField("alias", alias);
    return this.rawCmd(`ALIAS:${normMac}|${alias}`);
  }

  /**
   * @param {string} mac
   * @param {string | string[]} tags
   */
  setTags(mac, tags) {
    const normMac = normalizeMac48(mac);
    const tagStr = Array.isArray(tags)
      ? tags.map((t) => String(t).trim()).filter(Boolean).join(",")
      : String(tags);
    validateField("tags", tagStr);
    return this.rawCmd(`TAG:${normMac}|${tagStr}`);
  }

  /**
   * @param {number | string} targetId
   * @param {string} payload
   */
  sendCmd(targetId, payload) {
    validateField("payload", payload);
    return this.rawCmd(`CMD:${formatNodeId(targetId)}|${payload}`);
  }

  /**
   * @param {string} tag
   * @param {string} payload
   */
  sendTag(tag, payload) {
    validateField("tag", tag);
    validateField("payload", payload);
    return this.rawCmd(`CMD_TAG:${tag}|${payload}`);
  }

  /**
   * @param {string} mac
   * @param {string} room
   * @param {number} [xCm]
   * @param {number} [yCm]
   */
  setAnchor(mac, room, xCm = 0, yCm = 0) {
    const normMac = normalizeMac48(mac);
    validateField("room", room);
    return this.rawCmd(`ANCHOR:${normMac}|${room}|${Math.trunc(xCm)}|${Math.trunc(yCm)}`);
  }

  /**
   * @param {string} mac
   */
  removeAnchor(mac) {
    const normMac = normalizeMac48(mac);
    return this.rawCmd(`ANCHOR:${normMac}||0|0`);
  }

  /**
   * @param {string} name
   * @param {number} xCm
   * @param {number} yCm
   * @param {number} radiusCm
   */
  setFence(name, xCm, yCm, radiusCm) {
    validateField("name", name);
    const r = Math.trunc(radiusCm);
    if (r < 0) {
      throw new RangeError("radiusCm must be >= 0");
    }
    return this.rawCmd(`FENCE:${name}|${Math.trunc(xCm)}|${Math.trunc(yCm)}|${r}`);
  }

  /**
   * @param {string} name
   */
  removeFence(name) {
    return this.setFence(name, 0, 0, 0);
  }

  /**
   * @param {number} ruleId
   * @param {number} trigger
   * @param {{ zone?: string, csiThreshold?: number, tag?: string, cmd?: string }} [opts]
   */
  setRule(ruleId, trigger, opts = {}) {
    const rid = Math.trunc(ruleId);
    if (rid < 1 || rid > 8) {
      throw new RangeError("ruleId must be between 1 and 8");
    }
    const trig = Math.trunc(trigger);
    if (trig < 0 || trig > 6) {
      throw new RangeError("trigger must be between 0 and 6 (see RuleTrigger)");
    }
    if (trig === 0) {
      return this.rawCmd(`RULE:${rid}|0`);
    }
    const { zone = "*", csiThreshold = 40, tag = "*", cmd = "" } = opts;
    const z = validateField("zone", String(zone).trim() || "*");
    const t = validateField("tag", String(tag).trim() || "*");
    const c = validateField("cmd", String(cmd).trim());
    if (!c) {
      throw new Error("cmd must not be empty when creating an active rule");
    }
    return this.rawCmd(`RULE:${rid}|${trig}|${z}|${Math.trunc(csiThreshold)}|${t}|${c}`);
  }

  /**
   * @param {number} ruleId
   */
  removeRule(ruleId) {
    return this.setRule(ruleId, RuleTrigger.DELETE);
  }

  /**
   * @param {boolean} enabled
   */
  setBleScan(enabled) {
    return this.rawCmd(`BLESCAN:${enabled ? 1 : 0}`);
  }

  startDemo() {
    return this.rawCmd("LOCDEMO:1");
  }

  stepDemo() {
    return this.rawCmd("LOCDEMO:2");
  }

  stopDemo() {
    return this.rawCmd("LOCDEMO:0");
  }

  /**
   * @param {number | string} targetId
   * @param {number | string} version
   * @param {string} url
   */
  triggerOta(targetId, version, url) {
    validateField("url", url);
    return this.rawCmd(`OTA:${formatNodeId(targetId)}|${formatOtaVersion(version)}|${url}`);
  }

  /**
   * @param {string} pemCert
   */
  uploadTlsCa(pemCert) {
    return this._requestJson("POST", "/api/tls_ca", { body: String(pemCert), auth: true });
  }

  clearTlsCa() {
    return this._requestJson("POST", "/api/tls_ca", { body: "", auth: true });
  }

  resetWifi() {
    return this.rawCmd("RESET_WIFI");
  }

  reboot() {
    return this.rawCmd("REBOOT");
  }
}
