#!/usr/bin/env python3
"""OpenNear ESP32-WROOM-32 Real-Hardware Serial Log Analyzer (v6.3).

Parses single-board or multi-board UART logs from OpenNear ESP32 firmware
(v3.0 ~ v6.3+), extracts radio/sync/failover/Wi-Fi/MQTT/BLE/OTA/crash metrics,
generates root-cause diagnostics & tuning recommendations, and optionally exports
structured JSON or Markdown reports. Also supports direct live COM-port capture.

Usage:
  python tools/analyze_esp32_hw_log.py path/to/esp32.log
  python tools/analyze_esp32_hw_log.py build/hardware-esp32/ --md report.md --json report.json
  python tools/analyze_esp32_hw_log.py --port COM3 --baud 115200 --duration 60 --save capture.log
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
from pathlib import Path
import re
from statistics import mean, median
import sys
import time
from typing import Any, Dict, List, Optional, Tuple


# Firmware line matchers (compatible across OpenNear v3.0 ~ v6.3+)
RE_HOST_TS = re.compile(r"^(\d+(?:\.\d+)?)\s+(.*)$")
RE_STATUS = re.compile(
    r"\[OpenNear v(\d+\.\d+)\]\s+SF=(\d+)\s+role=(\d+)\s+id=(0x[0-9A-Fa-f]+)"
    r"\s+hop=(-?\d+)\s+slot=(\d+)\s+nco=([-+]?\d+(?:\.\d+)?)ppm"
    r"\s+rtt=(-?\d+)ns\s+harq=(\d+)\s+acts=(\d+)"
)
RE_TX = re.compile(r"\[OpenNear TX\]\s+accepted=(\d+)\s+failed=(\d+)")
RE_LINK = re.compile(
    r"\[OpenNear Link\]\s+sync_rx=(\d+)\s+data_tx=(\d+)\s+data_rx=(\d+)"
    r"\s+missed=(\d+)\s+takeovers=(\d+)\s+backup=(0x[0-9A-Fa-f]+)"
    r"\s+rejoins=(\d+)(?:\s+leases=(\d+))?"
)
RE_BOOT = re.compile(r"\[OpenNear\]\s+ESP32-WROOM-32 booted as (.+)")
RE_WIFI_ON = re.compile(
    r"\[OpenNear GW\]\s+Wi-Fi ONLINE\s+(?:http://|ip=)(\d+\.\d+\.\d+\.\d+)/?\s+ch=(\d+)\s+rssi=(-?\d+)"
)
RE_WIFI_OFF = re.compile(r"\[OpenNear GW\]\s+Wi-Fi disconnected")
RE_SOFTAP_ON = re.compile(r"\[OpenNear Portal\]\s+SoftAP '([^']+)' active")
RE_SOFTAP_OFF = re.compile(r"\[OpenNear Portal\]\s+Fallback SoftAP closed")
RE_MQTT_ON = re.compile(r"\[OpenNear MQTT\]\s+Connected to ([^:\s]+):(\d+)")
RE_BLE_CONN = re.compile(r"\[OpenNear BLE\]\s+Mobile/PC client connected")
RE_BLE_DISC = re.compile(r"\[OpenNear BLE\]\s+Client disconnected")
RE_OTA_PULL = re.compile(r"\[OpenNear OTA\]\s+Pulling firmware from (\S+)")
RE_OTA_STAGE = re.compile(r"\[OpenNear OTA\]\s+Staged (\d+) B \(CRC32=(0x[0-9A-Fa-f]+)\)")
RE_OTA_COMMIT = re.compile(r"\[OpenNear OTA\]\s+All targets READY")
RE_OTA_REBOOT = re.compile(r"\[OpenNear OTA\]\s+Partition (\S+) armed for fw (v[\d.]+)")
RE_OTA_CONFIRM = re.compile(r"\[OpenNear OTA\]\s+Boot confirmed on partition (\S+)")
RE_CLUSTER_JSON = re.compile(r"\[OpenNear Cluster\]\s+(\{.*\})$")
RE_ACTTRACE = re.compile(r"\[ACTTRACE\]\s+(.+)$")

FATAL_MARKERS = (
    "Guru Meditation",
    "LoadProhibited",
    "StoreProhibited",
    "InstrFetchProhibited",
    "Task watchdog got triggered",
    "Interrupt wdt timeout",
    "Brownout detector was triggered",
    "abort() was called",
    "HAL init failed",
    "Backtrace:",
    "stack overflow",
)

ROLE_NAMES = {0: "UNJOINED", 1: "G (Primary)", 2: "T (Terminal)", 3: "SUB_G (Relay)"}


@dataclass
class StatusSample:
    line_no: int
    host_s: Optional[float]
    fw_ver: str
    sf: int
    role: int
    short_id: str
    hop: int
    slot: int
    nco_ppm: float
    rtt_ns: int
    harq: int
    acts: int


@dataclass
class BoardAnalysis:
    source: str
    total_lines: int = 0
    duration_s: float = 0.0
    fw_versions: List[str] = field(default_factory=list)
    boots: List[Tuple[int, Optional[float], str]] = field(default_factory=list)
    fatals: List[Tuple[int, Optional[float], str]] = field(default_factory=list)
    samples: List[StatusSample] = field(default_factory=list)
    tx_samples: List[Tuple[int, Optional[float], int, int]] = field(default_factory=list)
    link_samples: List[Tuple[int, Optional[float], Dict[str, Any]]] = field(default_factory=list)
    events: List[Tuple[int, Optional[float], str, str]] = field(default_factory=list)
    last_cluster_json: Optional[Dict[str, Any]] = None
    act_trace_count: int = 0
    act_exec_us: List[int] = field(default_factory=list)

    def summarize(self) -> Dict[str, Any]:
        health = 100
        findings: List[Dict[str, str]] = []

        def add_finding(sev: str, code: str, msg: str, penalty: int = 0) -> None:
            nonlocal health
            findings.append({"severity": sev, "code": code, "message": msg})
            health = max(0, health - penalty)

        # 1. Crash / reboot health
        if self.fatals:
            add_finding(
                "CRITICAL",
                "FATAL_CRASH",
                f"检测到 {len(self.fatals)} 条致命异常/看门狗/欠压日志（首条位于第 {self.fatals[0][0]} 行：{self.fatals[0][2][:90]}）。",
                penalty=45,
            )
        if len(self.boots) > 1:
            add_finding(
                "WARN",
                "MULTI_REBOOT",
                f"日志中出现 {len(self.boots)} 次启动记录（若非主动复位/OTA 升级，请检查供电电流与看门狗）。",
                penalty=15,
            )

        # 2. Radio TX health
        tx_acc = self.tx_samples[-1][2] if self.tx_samples else 0
        tx_fail = self.tx_samples[-1][3] if self.tx_samples else 0
        tx_total = tx_acc + tx_fail
        tx_fail_pct = (100.0 * tx_fail / tx_total) if tx_total > 0 else 0.0
        if tx_total > 20 and tx_fail_pct > 5.0:
            add_finding(
                "CRITICAL" if tx_fail_pct > 15.0 else "WARN",
                "HIGH_TX_FAIL",
                f"原始 802.11 TX 失败率偏高：{tx_fail}/{tx_total} ({tx_fail_pct:.2f}%)。建议避开拥堵 Wi-Fi 信道或确认已启用 54M PHY 速率。",
                penalty=25 if tx_fail_pct > 15.0 else 10,
            )

        # 3. Sync / NCO / Role stability
        roles_seen = sorted({ROLE_NAMES.get(s.role, str(s.role)) for s in self.samples})
        nco_vals = [s.nco_ppm for s in self.samples if s.role != 1]  # Non-G tracks crystal offset
        nco_stats: Dict[str, Any] = {}
        if nco_vals:
            nco_med = median(nco_vals)
            nco_span = max(nco_vals) - min(nco_vals)
            nco_stats = {
                "count": len(nco_vals),
                "median_ppm": round(nco_med, 3),
                "mean_ppm": round(mean(nco_vals), 3),
                "min_ppm": round(min(nco_vals), 3),
                "max_ppm": round(max(nco_vals), 3),
                "span_ppm": round(nco_span, 3),
            }
            if abs(nco_med) > 40.0:
                add_finding(
                    "WARN",
                    "LARGE_XTAL_OFFSET",
                    f"晶振频偏中位数 {nco_med:+.2f} ppm（超过 ±40 ppm 常见范围），请检查板载 40 MHz 晶振精度或供电温升。",
                    penalty=8,
                )
            if nco_span > 12.0 and len(nco_vals) >= 10:
                add_finding(
                    "WARN",
                    "NCO_JITTER",
                    f"NCO 频偏波动范围达 {nco_span:.2f} ppm（{min(nco_vals):+.2f} ~ {max(nco_vals):+.2f} ppm），说明存在强 Wi-Fi 队列排队抖动或反复失锁重抓。",
                    penalty=10,
                )

        # 4. Link counters (sync_rx, missed, takeovers, rejoins, leases)
        last_link = self.link_samples[-1][2] if self.link_samples else {}
        max_missed = max((x[2]["missed"] for x in self.link_samples), default=0)
        takeovers = last_link.get("takeovers", 0)
        rejoins = last_link.get("rejoins", 0)
        leases = last_link.get("leases", 0)
        sync_rx = last_link.get("sync_rx", 0)
        data_tx = last_link.get("data_tx", 0)
        data_rx = last_link.get("data_rx", 0)

        if max_missed >= 8:
            add_finding(
                "WARN",
                "BEACON_MISS_BURST",
                f"观测到单次连续漏收同步帧峰值 missed={max_missed}（ESP32 接管门槛默认 12 帧），请检查天线摆放或同频路由器流量。",
                penalty=10,
            )
        if rejoins >= 3:
            add_finding(
                "WARN",
                "REJOIN_CHURN",
                f"节点共发生 {rejoins} 次重新入网（rejoins），若主控未重启则可能是上行租约超时或干扰过强。",
                penalty=12,
            )
        if not self.samples and self.total_lines > 0:
            add_finding(
                "WARN",
                "NO_OPENNEAR_HEARTBEAT",
                "日志中未解析到 [OpenNear vX.Y] 1 Hz 状态行，请确认串口波特率（115200）及烧录固件版本。",
                penalty=20,
            )

        # 5. Gateway / OTA / Connectivity summary
        wifi_ups = [e for e in self.events if e[2] == "WIFI_ONLINE"]
        wifi_downs = [e for e in self.events if e[2] == "WIFI_DISCONNECTED"]
        mqtt_ups = [e for e in self.events if e[2] == "MQTT_CONNECTED"]
        ota_confirms = [e for e in self.events if e[2] == "OTA_BOOT_CONFIRMED"]
        if len(wifi_downs) >= 3:
            add_finding(
                "WARN",
                "WIFI_FLAPPING",
                f"公网 Wi-Fi 断线重连 {len(wifi_downs)} 次，建议检查路由器信号强度（RSSI）或供电电容。",
                penalty=10,
            )

        if not findings:
            add_finding(
                "OK",
                "HEALTHY",
                "未发现崩溃、高丢包、频繁重连或显著时钟失锁，链路与网关运行健康。",
                penalty=0,
            )

        act_summary: Dict[str, Any] = {}
        if self.act_exec_us:
            s_exec = sorted(self.act_exec_us)
            p95 = s_exec[min(len(s_exec) - 1, int(len(s_exec) * 0.95))]
            act_summary = {
                "samples": len(s_exec),
                "median_exec_us": round(median(s_exec), 2),
                "p95_exec_us": p95,
                "max_exec_us": s_exec[-1],
            }

        return {
            "source": self.source,
            "health_score": health,
            "total_lines": self.total_lines,
            "duration_s": round(self.duration_s, 3),
            "fw_versions": self.fw_versions,
            "boot_count": len(self.boots),
            "fatal_count": len(self.fatals),
            "roles_seen": roles_seen,
            "last_role": ROLE_NAMES.get(self.samples[-1].role, "UNKNOWN") if self.samples else "UNKNOWN",
            "last_id": self.samples[-1].short_id if self.samples else "N/A",
            "last_sf": self.samples[-1].sf if self.samples else 0,
            "last_hop": self.samples[-1].hop if self.samples else -1,
            "nco_stats": nco_stats,
            "tx": {
                "accepted": tx_acc,
                "failed": tx_fail,
                "fail_pct": round(tx_fail_pct, 3),
            },
            "link": {
                "sync_rx": sync_rx,
                "data_tx": data_tx,
                "data_rx": data_rx,
                "max_missed": max_missed,
                "takeovers": takeovers,
                "rejoins": rejoins,
                "leases_reclaimed": leases,
                "last_backup": last_link.get("backup", "N/A"),
            },
            "gateway": {
                "wifi_online_events": len(wifi_ups),
                "wifi_disconnect_events": len(wifi_downs),
                "mqtt_connect_events": len(mqtt_ups),
                "ota_boot_confirmed": len(ota_confirms) > 0,
                "timeline": [
                    {"line": ln, "t_s": ts, "type": et, "detail": det}
                    for ln, ts, et, det in self.events
                ],
            },
            "act_trace": act_summary,
            "last_cluster_nodes": (
                self.last_cluster_json.get("count", 0) if self.last_cluster_json else None
            ),
            "findings": findings,
        }


def parse_log_lines(lines: List[str], source: str) -> BoardAnalysis:
    ba = BoardAnalysis(source=source)
    first_ts: Optional[float] = None
    last_ts: Optional[float] = None
    fw_set = set()

    for idx, raw in enumerate(lines, start=1):
        ba.total_lines = idx
        line = raw.strip()
        if not line:
            continue

        host_s: Optional[float] = None
        m_ts = RE_HOST_TS.match(line)
        if m_ts:
            try:
                host_s = float(m_ts.group(1))
                msg = m_ts.group(2)
                if first_ts is None:
                    first_ts = host_s
                last_ts = host_s
            except ValueError:
                msg = line
        else:
            msg = line

        if any(marker in msg for marker in FATAL_MARKERS):
            ba.fatals.append((idx, host_s, msg))

        if m := RE_BOOT.search(msg):
            ba.boots.append((idx, host_s, m.group(1)))
            ba.events.append((idx, host_s, "BOOT", m.group(1)))
            continue

        if m := RE_STATUS.search(msg):
            ver = f"v{m.group(1)}"
            fw_set.add(ver)
            ba.samples.append(
                StatusSample(
                    line_no=idx,
                    host_s=host_s,
                    fw_ver=ver,
                    sf=int(m.group(2)),
                    role=int(m.group(3)),
                    short_id=m.group(4),
                    hop=int(m.group(5)),
                    slot=int(m.group(6)),
                    nco_ppm=float(m.group(7)),
                    rtt_ns=int(m.group(8)),
                    harq=int(m.group(9)),
                    acts=int(m.group(10)),
                )
            )
            continue

        if m := RE_TX.search(msg):
            ba.tx_samples.append((idx, host_s, int(m.group(1)), int(m.group(2))))
            continue

        if m := RE_LINK.search(msg):
            ba.link_samples.append(
                (
                    idx,
                    host_s,
                    {
                        "sync_rx": int(m.group(1)),
                        "data_tx": int(m.group(2)),
                        "data_rx": int(m.group(3)),
                        "missed": int(m.group(4)),
                        "takeovers": int(m.group(5)),
                        "backup": m.group(6),
                        "rejoins": int(m.group(7)),
                        "leases": int(m.group(8)) if m.group(8) is not None else 0,
                    },
                )
            )
            continue

        if m := RE_WIFI_ON.search(msg):
            ba.events.append((idx, host_s, "WIFI_ONLINE", f"ip={m.group(1)} ch={m.group(2)} rssi={m.group(3)}dBm"))
        elif RE_WIFI_OFF.search(msg):
            ba.events.append((idx, host_s, "WIFI_DISCONNECTED", msg))
        elif m := RE_SOFTAP_ON.search(msg):
            ba.events.append((idx, host_s, "SOFTAP_STARTED", m.group(1)))
        elif RE_SOFTAP_OFF.search(msg):
            ba.events.append((idx, host_s, "SOFTAP_CLOSED", "switched to WIFI_STA"))
        elif m := RE_MQTT_ON.search(msg):
            ba.events.append((idx, host_s, "MQTT_CONNECTED", f"{m.group(1)}:{m.group(2)}"))
        elif RE_BLE_CONN.search(msg):
            ba.events.append((idx, host_s, "BLE_CONNECTED", "GATT client connected"))
        elif RE_BLE_DISC.search(msg):
            ba.events.append((idx, host_s, "BLE_DISCONNECTED", "GATT client disconnected"))
        elif m := RE_OTA_PULL.search(msg):
            ba.events.append((idx, host_s, "OTA_PULL", m.group(1)))
        elif m := RE_OTA_STAGE.search(msg):
            ba.events.append((idx, host_s, "OTA_STAGED", f"{m.group(1)}B CRC32={m.group(2)}"))
        elif RE_OTA_COMMIT.search(msg):
            ba.events.append((idx, host_s, "OTA_COMMIT", "synchronized cluster commit scheduled"))
        elif m := RE_OTA_REBOOT.search(msg):
            ba.events.append((idx, host_s, "OTA_REBOOT", f"part={m.group(1)} fw={m.group(2)}"))
        elif m := RE_OTA_CONFIRM.search(msg):
            ba.events.append((idx, host_s, "OTA_BOOT_CONFIRMED", f"part={m.group(1)}"))
        elif m := RE_CLUSTER_JSON.search(msg):
            try:
                ba.last_cluster_json = json.loads(m.group(1))
                ba.events.append(
                    (idx, host_s, "CLUSTER_SNAPSHOT", f"nodes={ba.last_cluster_json.get('count', '?')}")
                )
            except json.JSONDecodeError:
                pass
        elif m := RE_ACTTRACE.search(msg):
            parts = m.group(1).split(",")
            if len(parts) >= 10:
                try:
                    pre_us = int(parts[8])
                    post_us = int(parts[9])
                    ba.act_trace_count += 1
                    ba.act_exec_us.append(max(0, post_us - pre_us))
                except ValueError:
                    pass

    ba.fw_versions = sorted(fw_set)
    if first_ts is not None and last_ts is not None and last_ts >= first_ts:
        ba.duration_s = last_ts - first_ts
    elif len(ba.samples) >= 2:
        ba.duration_s = float(len(ba.samples) - 1)
    return ba


def render_markdown(summaries: List[Dict[str, Any]]) -> str:
    out: List[str] = ["# OpenNear ESP32 真机日志诊断报告", ""]
    for s in summaries:
        out.append(f"## 节点日志：`{s['source']}`（健康分：**{s['health_score']}/100**）")
        out.append("")
        out.append("| 指标 | 实测值 |")
        out.append("|---|---|")
        out.append(f"| 固件版本 / 角色 / Short-ID | `{', '.join(s['fw_versions']) or 'N/A'}` / `{s['last_role']}` / `{s['last_id']}` (Hop {s['last_hop']}) |")
        out.append(f"| 日志行数 / 时长 / 启动次数 / 致命异常 | `{s['total_lines']}` 行 / `{s['duration_s']} s` / `{s['boot_count']}` 次启动 / `{s['fatal_count']}` 条异常 |")
        tx = s["tx"]
        out.append(f"| 原始 802.11 TX（成功 / 失败 / 失败率） | `{tx['accepted']}` / `{tx['failed']}` / **{tx['fail_pct']}%** |")
        lk = s["link"]
        out.append(f"| 链路统计（sync_rx / data_tx / data_rx） | `{lk['sync_rx']}` / `{lk['data_tx']}` / `{lk['data_rx']}` |")
        out.append(f"| 自愈指标（max_missed / takeovers / rejoins / leases） | `{lk['max_missed']}` / `{lk['takeovers']}` / `{lk['rejoins']}` / `{lk['leases_reclaimed']}` |")
        if s["nco_stats"]:
            n = s["nco_stats"]
            out.append(f"| NCO 晶振频偏（中位数 / 波动范围） | `{n['median_ppm']:+.3f} ppm` / 跨度 `{n['span_ppm']:.3f} ppm` (`{n['min_ppm']:+.2f}` ~ `{n['max_ppm']:+.2f}`) |")
        gw = s["gateway"]
        out.append(f"| 网关与 OTA（Wi-Fi 上/断线 / MQTT / OTA 确认） | `{gw['wifi_online_events']}` 上 / `{gw['wifi_disconnect_events']}` 断 / `{gw['mqtt_connect_events']}` MQTT / OTA确认=`{gw['ota_boot_confirmed']}` |")
        if s["last_cluster_nodes"] is not None:
            out.append(f"| 最近集群拓扑快照节点数 | `{s['last_cluster_nodes']}` 节点 |")
        out.append("")
        out.append("### 诊断结论与调优建议")
        for f in s["findings"]:
            out.append(f"- **[{f['severity']}] `{f['code']}`**：{f['message']}")
        if gw["timeline"]:
            out.append("")
            out.append("### 关键事件时间线")
            for ev in gw["timeline"][:25]:
                t_str = f"{ev['t_s']:.3f}s" if ev["t_s"] is not None else f"L{ev['line']}"
                out.append(f"- `{t_str}` **{ev['type']}**: {ev['detail']}")
        out.append("")
    return "\n".join(out)


def capture_live_serial(port: str, baud: int, duration_s: float, save_path: Optional[Path]) -> List[str]:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError("实时串口抓包需要 pyserial，请先运行: python -m pip install pyserial") from exc

    lines: List[str] = []
    start = time.monotonic()
    print(f"[OpenNear Analyzer] 正在监听 {port} @ {baud} baud，持续 {duration_s:.1f} s（Ctrl+C 可提前结束）...")
    with serial.Serial(port, baud, timeout=0.2) as ser:
        try:
            while time.monotonic() - start < duration_s:
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                ts = time.monotonic() - start
                stamped = f"{ts:08.3f} {text}"
                lines.append(stamped)
                if len(lines) % 15 == 0:
                    print(f"  ...已采集 {len(lines)} 行 ({ts:.1f}s)")
        except KeyboardInterrupt:
            print("\n[OpenNear Analyzer] 已手动停止采集，开始分析...")

    if save_path:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        save_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        print(f"[OpenNear Analyzer] 原始串口日志已保存至: {save_path}")
    return lines


def collect_log_files(inputs: List[str]) -> List[Path]:
    files: List[Path] = []
    for item in inputs:
        p = Path(item)
        if p.is_dir():
            for child in sorted(p.rglob("*")):
                if child.is_file() and child.suffix.lower() in (".log", ".txt"):
                    files.append(child)
        elif p.is_file():
            files.append(p)
        else:
            raise FileNotFoundError(f"找不到日志路径: {item}")
    return files


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="OpenNear ESP32 真机串口日志自动化分析工具")
    ap.add_argument("inputs", nargs="*", help="串口日志文件或目录路径（支持多板并行对比）")
    ap.add_argument("--port", help="实时监听的串口号（例如 COM3）")
    ap.add_argument("--baud", type=int, default=115200, help="串口波特率（默认 115200）")
    ap.add_argument("--duration", type=float, default=30.0, help="实时采集时长（秒，默认 30）")
    ap.add_argument("--save", type=Path, help="将实时采集的原始串口日志保存到指定文件")
    ap.add_argument("--json", type=Path, help="导出结构化 JSON 分析报告")
    ap.add_argument("--md", type=Path, help="导出 Markdown 诊断报告")
    args = ap.parse_args(argv)

    analyses: List[BoardAnalysis] = []
    if args.port:
        lines = capture_live_serial(args.port, args.baud, args.duration, args.save)
        analyses.append(parse_log_lines(lines, f"live:{args.port}"))
    if args.inputs:
        for path in collect_log_files(args.inputs):
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
            analyses.append(parse_log_lines(lines, str(path)))

    if not analyses:
        ap.print_help()
        return 2

    summaries = [a.summarize() for a in analyses]
    md = render_markdown(summaries)
    print(md)

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(summaries, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\n[OK] JSON 报告已导出: {args.json}")
    if args.md:
        args.md.parent.mkdir(parents=True, exist_ok=True)
        args.md.write_text(md, encoding="utf-8")
        print(f"[OK] Markdown 报告已导出: {args.md}")

    return 1 if any(s["fatal_count"] > 0 for s in summaries) else 0


if __name__ == "__main__":
    sys.exit(main())
