"""Decompose dual-board ACT skew from simultaneous GPIO capture and firmware traces."""
import argparse
from bisect import bisect_left
from collections import Counter
import csv
from itertools import pairwise
from dataclasses import dataclass
import json
from pathlib import Path
from statistics import median

from measure_esp32_gpio_sync import CLOCK, EDGE, Edge, distribution, extend_ticks, pair_edges

TRACE = '[ACTTRACE] '


@dataclass(frozen=True)
class Act:
    seq: int
    sf: int
    level: int
    role: int
    sf_us: int
    act_offset_us: int
    plan: int
    step: int
    pre: int
    post: int
    beacon_sf: int
    rx_hw: int
    rx: int
    cb: int
    dequeue: int
    accept: int
    scheduled: int = 0  # Backward-compatible with baseline UART logs.

    @property
    def polarity(self):
        return 'R' if self.level else 'F'


def read_log(path):
    captures, acts, clock, warnings = {}, [], None, []
    for line in path.read_text(encoding='utf-8').splitlines():
        when, _, msg = line.partition(' ')
        host = float(when)
        if m := CLOCK.fullmatch(msg):
            hz = int(m[1])
            if clock and clock != hz:
                warnings.append('capture clock changed')
            clock = hz
        elif m := EDGE.fullmatch(msg):
            edge = Edge(int(m[1]), int(m[2]), m[3], int(m[4]))
            captures[edge.seq] = (edge, host)
        elif msg.startswith(TRACE):
            try:
                acts.append((Act(*map(int, msg[len(TRACE):].split(','))), host))
            except (TypeError, ValueError):
                warnings.append(msg)
        elif 'booted as' in msg or 'Guru Meditation' in msg:
            warnings.append(msg)
    return captures, acts, clock, warnings


def seq_gaps(items):
    return sum((b.seq - a.seq - 1) % (1 << 32) for a, b in pairwise(items))


def densest_offset(values, width):
    values = sorted(values)
    start, best_first, best_last = 0, 0, 0
    for end, item in enumerate(values):
        while item - values[start] > width:
            start += 1
        if end - start + 1 > best_last - best_first:
            best_first, best_last = start, end + 1
    if best_last - best_first < 20:
        raise ValueError('Not enough unambiguous UART timing seeds')
    return median(values[best_first:best_last])


def align_traces(physical, acts):
    # Host times seed each match but are not used to calculate skew or phase offsets.
    # MCPWM and each esp_timer then form a stable linear relation over one short run.
    by_pol = {p: sorted((us, e.seq) for us, e, _ in physical if e.polarity == p) for p in ('R', 'F')}
    lookup = {p: [us for us, _ in xs] for p, xs in by_pol.items()}
    by_host = sorted((host, us, e.polarity) for us, e, host in physical)
    host_times = [host for host, _, _ in by_host]
    candidates = []
    for act, host in acts[:200]:
        j = bisect_left(host_times, host - 0.055)
        for cap_host, us, pol in by_host[j:j + 12]:
            if cap_host > host + 0.055:
                break
            if pol == act.polarity:
                candidates.append(us - act.post)
    offset = densest_offset(candidates, 400)

    def match(scale, base, tolerance):
        selected = {}
        for act, _ in acts:
            expect = scale * act.post + base
            ys = lookup[act.polarity]
            i = bisect_left(ys, expect)
            near = [j for j in (i - 1, i) if 0 <= j < len(ys)]
            if not near:
                continue
            best = min(near, key=lambda j: abs(ys[j] - expect))
            error = abs(ys[best] - expect)
            if error > tolerance:
                continue
            us, seq = by_pol[act.polarity][best]
            if seq not in selected or error < selected[seq][2]:
                selected[seq] = (act, us, error)
        return selected

    rough = list(match(1, offset, 6000).values())
    if len(rough) < max(50, int(len(acts) * .80)):
        raise ValueError('Too few rough local clock matches')
    # Median in early and late groups tolerates occasional preemption between pad write and timestamp.
    rough.sort(key=lambda row: row[0].post)
    bucket = max(10, len(rough) // 8)

    def fit(group):
        return median(a.post for a, _, _ in group), median(us - a.post for a, us, _ in group)

    x0, o0 = fit(rough[:bucket]); x1, o1 = fit(rough[-bucket:])
    scale = 1 + (o1 - o0) / (x1 - x0)
    selected = match(scale, o0 - (scale - 1) * x0, 60)
    if len(selected) < max(50, int(len(acts) * .90)) or abs(scale - 1) > .0005:
        raise ValueError('Too few precise local clock matches or inconsistent clock rate')
    return selected, scale


def analyze(primary, terminal):
    caps, g_acts, hz, warnings = read_log(primary)
    _, t_acts, _, t_warnings = read_log(terminal)
    if warnings or t_warnings or not hz or not g_acts or not t_acts:
        raise ValueError(f'Missing capture/trace data or invalid session: {warnings + t_warnings}')
    if set(a.role for a, _ in g_acts) != {1} or set(a.role for a, _ in t_acts) != {2}:
        raise ValueError('Roles changed; a separate steady-state trace is needed')
    gaps = [seq_gaps(c[0] for c in caps.values()), seq_gaps(a for a, _ in g_acts), seq_gaps(a for a, _ in t_acts)]
    if any(gaps):
        raise ValueError(f'UART/trace gaps: {gaps}')
    extended = extend_ticks(e for e, _ in caps.values())
    physical = [[(ticks * 1e6 / hz, edge, caps[edge.seq][1]) for ticks, edge in extended if edge.channel == channel]
                for channel in (0, 1)]
    gm, gscale = align_traces(physical[0], g_acts)
    tm, tscale = align_traces(physical[1], t_acts)
    window_us = 30000
    pairs = pair_edges(extended, window_us * hz // 1_000_000)
    rows = []
    for p_ticks, t_ticks, p_edge, t_edge in pairs:
        if p_edge.seq not in gm or t_edge.seq not in tm:
            continue
        g, g_at, _ = gm[p_edge.seq]
        t, t_at, _ = tm[t_edge.seq]
        if not (t.rx and t.cb and t.accept and g.sf_us == t.sf_us):
            continue
        assert g_at == p_ticks * 1e6 / hz and t_at == t_ticks * 1e6 / hz
        g_plan = g_at - (g.post - g.plan) * gscale
        t_plan = t_at - (t.post - t.plan) * tscale
        frame_delta = t.sf - g.sf
        g_beacon_nominal = g_plan + ((t.beacon_sf - g.sf) * g.sf_us - g.act_offset_us) * gscale
        rx_mapped_gap = t_at - (t.post - t.rx) * tscale - g_beacon_nominal
        plan_gap = t_plan - g_plan
        clock_phase = plan_gap - (frame_delta * g.sf_us + t.act_offset_us - g.act_offset_us) * gscale
        rows.append(dict(polarity=g.polarity, g_sf=g.sf, t_sf=t.sf, t_beacon_sf=t.beacon_sf, frame_delta=frame_delta,
                         physical_us=t_at-g_at, plan_gap_us=plan_gap, clock_phase_us=clock_phase,
                         t_scheduled=t.scheduled, sf_us=g.sf_us,
                         g_dispatch_us=g.pre-g.plan, t_dispatch_us=t.pre-t.plan,
                         g_write_us=g.post-g.pre, t_write_us=t.post-t.pre,
                         g_planned_to_write_us=g.post-g.plan, t_planned_to_write_us=t.post-t.plan,
                         rx_mapped_gap_us=rx_mapped_gap,
                         rx_callback_gap_us=t_at-(t.post-t.cb)*tscale-g_beacon_nominal,
                         cb_to_dequeue_us=t.dequeue-t.cb, dequeue_to_accept_us=t.accept-t.dequeue,
                         rx_mapped_to_cb_us=t.cb-t.rx,
                         phase_minus_rx_mapped_us=clock_phase-rx_mapped_gap))
    if len(rows) < len(pairs) * .85 or not rows:
        raise ValueError(f'Too few complete trace pairs: {len(rows)} of {len(pairs)}')
    groups = {key: distribution([r[key] for r in rows]) for key in rows[0] if key.endswith('_us')}
    result = dict(capture_hz=hz, window_us=window_us, primary_edges=len(physical[0]), terminal_edges=len(physical[1]),
                  edge_pairs=len(pairs), diagnosed_pairs=len(rows), trace_records=[len(g_acts), len(t_acts)],
                  clock_label_matches=[len(gm), len(tm)], sequences_missing=gaps,
                  affine_label_error_us=[distribution([row[2] for row in labels.values()]) for labels in (gm, tm)],
                  frame_delta_counts=dict(sorted(Counter(r['frame_delta'] for r in rows).items())),
                  terminal_scheduled_pairs=sum(r['t_scheduled'] for r in rows),
                  gpio_terminal_late_by_sf=sum(r['physical_us'] >= r['sf_us'] for r in rows),
                  gpio_within_1ms=sum(abs(r['physical_us']) <= 1000 for r in rows),
                  unpaired_edges=[len(edges) - len(pairs) for edges in physical],
                  same_frame_gpio_us=distribution([r['physical_us'] for r in rows if r['frame_delta'] == 0]),
                  summary_us=groups, inputs=[str(primary), str(terminal)])
    return result, rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('primary_log', type=Path)
    p.add_argument('terminal_log', type=Path)
    args = p.parse_args()
    p_log, t_log = args.primary_log.resolve(), args.terminal_log.resolve()
    result, rows = analyze(p_log, t_log)
    base = p_log.with_name(p_log.stem.removesuffix('-primary'))
    csv_path = base.with_suffix('.csv')
    with csv_path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    result['paired_csv'] = str(csv_path)
    out = base.with_suffix('.json')
    out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(out)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
