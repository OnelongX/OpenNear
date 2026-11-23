"""Measure two GPIO2 waveforms timestamped by a single ESP32 MCPWM timer."""

import argparse
from bisect import bisect_left
import csv
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import re
from statistics import median
import time


EDGE = re.compile(r'\[GPIOCAP\] (\d+),([01]),([RF]),(\d+)')
CLOCK = re.compile(r'\[GPIOCAP\] hz=(\d+) p=32 t=33')
MOD = 1 << 32


@dataclass(frozen=True)
class Edge:
    seq: int
    channel: int
    polarity: str
    ticks: int


def extend_ticks(edges):
    """Unwrap the shared 32-bit clock; one ISR can deliver two edges out of order."""
    newest = None
    result = []
    for edge in edges:
        if newest is None:
            ticks = edge.ticks
        else:
            delta = (edge.ticks - newest + MOD // 2) % MOD - MOD // 2
            ticks = newest + delta
        newest = ticks if newest is None else max(newest, ticks)
        result.append((ticks, edge))
    return sorted(result, key=lambda item: item[0])


def pair_edges(extended, window_ticks):
    """Match nearest same-polarity edges, one-to-one; reject ambiguous candidates."""
    pairs = []
    for polarity in ('R', 'F'):
        p = [(ticks, e) for ticks, e in extended if e.channel == 0 and e.polarity == polarity]
        t = [(ticks, e) for ticks, e in extended if e.channel == 1 and e.polarity == polarity]
        p_times = [ticks for ticks, _ in p]
        t_times = [ticks for ticks, _ in t]
        for p_ticks, p_edge in p:
            j = bisect_left(t_times, p_ticks - window_ticks)
            if j == len(t) or t_times[j] > p_ticks + window_ticks:
                continue
            if j + 1 < len(t) and t_times[j + 1] <= p_ticks + window_ticks:
                continue
            t_ticks, t_edge = t[j]
            k = bisect_left(p_times, t_ticks - window_ticks)
            if k + 1 < len(p) and p_times[k + 1] <= t_ticks + window_ticks:
                continue
            pairs.append((p_ticks, t_ticks, p_edge, t_edge))
    return sorted(pairs, key=lambda item: item[0])


def distribution(values):
    if not values:
        return {'count': 0}
    absolute = sorted(map(abs, values))

    def percentile(percent):
        return absolute[(len(absolute) * percent + 99) // 100 - 1]

    return {'count': len(values), 'median_signed_us': median(values),
            'median_abs_us': median(absolute), 'p95_abs_us': percentile(95),
            'p99_abs_us': percentile(99), 'max_abs_us': absolute[-1]}


def collect(port_name, duration, raw_path):
    import serial

    port = serial.Serial(port=None, baudrate=115200, timeout=0.2)
    port.port, port.dtr, port.rts = port_name, False, False
    edges = []
    clock = None
    start = last_edge_at = time.monotonic()
    stop = start + duration
    warnings = []
    with port, raw_path.open('w', encoding='utf-8') as raw:
        port.reset_input_buffer()
        while time.monotonic() < stop:
            line = port.readline().decode('ascii', 'replace').strip()
            if not line:
                continue
            now = time.monotonic()
            raw.write(f'{now - start:.4f} {line}\n')
            if m := CLOCK.fullmatch(line):
                hz = int(m[1])
                if clock is not None and clock != hz:
                    warnings.append('APB clock changed')
                clock = hz
            elif m := EDGE.fullmatch(line):
                if clock is None:
                    continue
                e = Edge(int(m[1]), int(m[2]), m[3], int(m[4]))
                if edges and now - last_edge_at >= MOD / (2 * clock):
                    warnings.append('idle gap too long to unwrap the hardware clock')
                edges.append(e)
                last_edge_at = now
            elif line.startswith('[GPIOCAP]') or 'booted as' in line or 'Guru Meditation' in line:
                warnings.append(line)
    return clock, edges, warnings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='UART of the primary running esp32_node_g_capture')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--window-us', type=int, default=10000)
    parser.add_argument('--output-dir', type=Path, default=Path('build/gpio-sync'))
    args = parser.parse_args()
    if not (2 <= args.seconds <= 3600) or not (1 <= args.window_us < 40000):
        parser.error('seconds must be 2..3600, window-us must be 1..39999')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.output_dir.resolve() / datetime.now().strftime('gpio-sync-%Y%m%d-%H%M%S-%f')
    raw = stem.with_suffix('.log')
    output = stem.with_suffix('.csv')
    clock, edges, warnings = collect(args.port, args.seconds, raw)
    missing = sum((b.seq - a.seq - 1) % MOD for a, b in zip(edges, edges[1:]))
    if not clock:
        warnings.append('no hardware clock reported; verify capture firmware and COM port')
    if missing:
        warnings.append('capture sequence gaps; no complete measurement')
    pairs = pair_edges(extend_ticks(edges), args.window_us * clock // 1000000) if clock else []
    counts = [sum(e.channel == i for e in edges) for i in (0, 1)]
    if len(pairs) < 10:
        warnings.append('too few matched edges; verify both signals and common ground')
    deltas = {'R': [], 'F': []}
    with output.open('w', newline='', encoding='utf-8') as dest:
        writer = csv.writer(dest)
        writer.writerow(('edge', 'primary_seq', 'terminal_seq', 'primary_ticks', 'terminal_ticks', 'terminal_minus_primary_us'))
        for p_ticks, t_ticks, p, t in pairs:
            skew = (t_ticks - p_ticks) * 1000000 / clock
            deltas[p.polarity].append(skew)
            writer.writerow((p.polarity, p.seq, t.seq, p_ticks, t_ticks, f'{skew:.4f}'))
    report = {'valid': not warnings, 'port': args.port, 'seconds': args.seconds, 'capture_hz': clock,
              'tick_ns': 1000000000 / clock if clock else None, 'window_us': args.window_us,
              'primary_edges': counts[0], 'terminal_edges': counts[1], 'pairs': len(pairs),
              'unmatched_primary': counts[0] - len(pairs), 'unmatched_terminal': counts[1] - len(pairs),
              'missing_serial_records': missing, 'warnings': warnings,
              'all': distribution(deltas['R'] + deltas['F']), 'rising': distribution(deltas['R']),
              'falling': distribution(deltas['F']), 'raw_log': str(raw), 'paired_csv': str(output)}
    summary = stem.with_suffix('.json')
    summary.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    print(f'Summary: {summary}')
    return 0 if report['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
