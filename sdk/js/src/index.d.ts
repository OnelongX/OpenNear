export declare const SDK_VERSION: "7.3.0";

export declare const RuleTrigger: Readonly<{
  DELETE: 0;
  ENTER_ROOM: 1;
  LEAVE_ROOM: 2;
  IN_FENCE: 3;
  OUT_FENCE: 4;
  CSI_MOTION: 5;
  CSI_QUIET: 6;
}>;

export type RuleTriggerValue = (typeof RuleTrigger)[keyof typeof RuleTrigger];

export interface OpenNearClientOptions {
  token?: string | null;
  timeoutMs?: number;
  fetchImpl?: typeof fetch;
}

export interface MqttOptions {
  port?: number;
  user?: string;
  password?: string;
  tls?: boolean;
}

export interface RuleOptions {
  zone?: string;
  csiThreshold?: number;
  tag?: string;
  cmd?: string;
}

export declare class OpenNearError extends Error {
  statusCode: number | null;
  body: unknown;
  constructor(message: string, statusCode?: number | null, body?: unknown);
}

export declare class OpenNearAuthError extends OpenNearError {
  constructor(message: string, body?: unknown);
}

export declare class OpenNearClient {
  baseUrl: string;
  token: string | null;
  timeoutMs: number;

  constructor(baseUrl?: string, options?: OpenNearClientOptions);

  getStatus(): Promise<Record<string, any>>;
  scanWifi(): Promise<Record<string, any>>;
  getTopology(): Promise<Record<string, any>>;
  getData(): Promise<Record<string, any>>;
  getAliases(): Promise<Record<string, any>>;
  getTags(): Promise<Record<string, any>>;
  getLocate(): Promise<Record<string, any>>;

  rawCmd(command: string): Promise<Record<string, any>>;
  setWifi(ssid: string, password: string, cloudUrl?: string): Promise<Record<string, any>>;
  setMqtt(host: string, opts?: MqttOptions): Promise<Record<string, any>>;
  setToken(newToken: string): Promise<Record<string, any>>;
  setNetKey(hexKey32?: string): Promise<Record<string, any>>;
  setAlias(mac: string, alias: string): Promise<Record<string, any>>;
  setTags(mac: string, tags: string | string[]): Promise<Record<string, any>>;
  sendCmd(targetId: number | string, payload: string): Promise<Record<string, any>>;
  sendTag(tag: string, payload: string): Promise<Record<string, any>>;
  setAnchor(mac: string, room: string, xCm?: number, yCm?: number): Promise<Record<string, any>>;
  removeAnchor(mac: string): Promise<Record<string, any>>;
  setFence(name: string, xCm: number, yCm: number, radiusCm: number): Promise<Record<string, any>>;
  removeFence(name: string): Promise<Record<string, any>>;
  setRule(ruleId: number, trigger: RuleTriggerValue | number, opts?: RuleOptions): Promise<Record<string, any>>;
  removeRule(ruleId: number): Promise<Record<string, any>>;
  setBleScan(enabled: boolean): Promise<Record<string, any>>;
  startDemo(): Promise<Record<string, any>>;
  stepDemo(): Promise<Record<string, any>>;
  stopDemo(): Promise<Record<string, any>>;
  triggerOta(targetId: number | string, version: number | string, url: string): Promise<Record<string, any>>;
  uploadTlsCa(pemCert: string): Promise<Record<string, any>>;
  clearTlsCa(): Promise<Record<string, any>>;
  resetWifi(): Promise<Record<string, any>>;
  reboot(): Promise<Record<string, any>>;
}
