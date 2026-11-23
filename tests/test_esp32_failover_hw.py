"""Exercise OpenNear's primary/backup on two RTS-reset ESP32 development boards."""

import argparse
from contextlib import ExitStack
from datetime import datetime
import json
from pathlib import Path
import re
import threading
import time

import serial

PATTERNS = (
    re.compile(r'\[OpenNear v3\.0\] SF=(\d+) role=(\d+) id=(0x[0-9A-Fa-f]+)'),
    re.compile(r'\[OpenNear TX\] accepted=(\d+) failed=(\d+)'),
    re.compile(r'\[OpenNear Link\] sync_rx=(\d+) data_tx=(\d+) data_rx=(\d+) missed=(\d+) takeovers=(\d+) backup=(0x[0-9A-Fa-f]+) rejoins=(\d+)'),
)
FIELDS = ('sf', 'role', 'id', 'accepted', 'failed', 'sync_rx', 'data_tx', 'data_rx', 'missed', 'takeovers', 'backup', 'rejoins')


def positive_int(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError('must be positive')
    return value


class TestRun:
    def __init__(self, args):
        self.args = args
        self.start = time.monotonic()
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.events = []
        self.state = {role: dict(boots=0, seq=0, at=None, report={}, errors=[]) for role in ('primary', 'backup')}
        self.prefix = args.output_dir.resolve() / datetime.now().strftime('esp32-hardware-%Y%m%d-%H%M%S-%f')
        self.prefix.parent.mkdir(parents=True, exist_ok=True)
        self.paths = {role: self.prefix.with_name(f'{self.prefix.name}-{role}.log') for role in self.state}
        self.summary = self.prefix.with_name(f'{self.prefix.name}-summary.json')

    def elapsed(self):
        return round(time.monotonic() - self.start, 3)

    def read(self, role, port):
        try:
            with self.paths[role].open('w', encoding='utf-8', buffering=1) as log:
                pending = b''
                parts = []
                while not self.stop.is_set():
                    pending += port.read(max(1, port.in_waiting))
                    while b'\n' in pending:
                        raw, pending = pending.split(b'\n', 1)
                        line = raw.decode('utf-8', 'replace').strip()
                        at = self.elapsed()
                        log.write(f'{at:08.3f} {line}\n')
                        with self.lock:
                            state = self.state[role]
                            if 'booted as' in line:
                                state['boots'] += 1
                                state['report'] = {}
                                parts.clear()
                            if any(x in line for x in ('Guru Meditation', 'watchdog got triggered', 'HAL init failed', 'Brownout detector was triggered', 'abort() was called')):
                                state['errors'].append(line)
                            if PATTERNS[0].search(line):
                                parts.clear()
                            if parts and at - parts[0][0] > 0.5:
                                parts.clear()
                            match = PATTERNS[len(parts)].search(line)
                            if match:
                                parts.append((at, tuple(int(x, 0) for x in match.groups())))
                                if len(parts) == len(PATTERNS):
                                    state['seq'] += 1
                                    state['at'] = at
                                    state['report'] = dict(zip(FIELDS, (x for _, values in parts for x in values)))
                                    parts.clear()
        except Exception as exc:
            with self.lock:
                self.state[role]['errors'].append(f'serial reader: {exc!r}')

    def snapshot(self):
        with self.lock:
            return {role: {**s, 'report': dict(s['report']), 'errors': list(s['errors'])}
                    for role, s in self.state.items()}

    def checked(self):
        snap = self.snapshot()
        if any(s['errors'] for s in snap.values()):
            raise RuntimeError(f'Firmware or serial reader error: {snap}')
        return snap

    def log(self, event, snap=None, **extra):
        item = dict(elapsed_s=self.elapsed(), event=event, **extra)
        if snap is not None:
            item['boards'] = snap
        self.events.append(item)
        print(json.dumps(item, ensure_ascii=True), flush=True)

    def wait_for(self, label, condition, timeout):
        end = time.monotonic() + timeout
        while True:
            snap = self.checked()
            if condition(snap):
                return snap
            if time.monotonic() >= end:
                raise RuntimeError(f'Timed out waiting for {label}: {snap}')
            time.sleep(0.025)

    @staticmethod
    def paired(snap):
        g, t = (snap[role]['report'] for role in ('primary', 'backup'))
        addr = t.get('id', 0)
        return (addr > 1 and g.get('role') == 1 and g.get('id') == 1 and t.get('role') == 2
                and g.get('backup') == t.get('backup') == addr and g.get('data_rx', 0) > 0
                and t.get('sync_rx', 0) > 0)

    def stable(self, label, baseline, seconds):
        end = time.monotonic() + seconds
        progress = {key: (baseline[role]['report'][key], time.monotonic())
                    for role, key in (('primary', 'data_rx'), ('backup', 'sync_rx'), ('backup', 'data_tx'))}
        while True:
            snap = self.checked()
            if not self.paired(snap):
                raise RuntimeError(f'{label}: lost paired roles: {snap}')
            now = time.monotonic()
            for role, board in snap.items():
                old = baseline[role]
                if (board['boots'] != old['boots'] or self.elapsed() - board['at'] > 4
                        or any(board['report'][k] != old['report'][k] for k in ('role', 'id', 'backup', 'takeovers', 'rejoins'))):
                    raise RuntimeError(f'{label}: unexpected change or stale serial report: {snap}')
            for role, key in (('primary', 'data_rx'), ('backup', 'sync_rx'), ('backup', 'data_tx')):
                count = snap[role]['report'][key]
                previous, when = progress[key]
                if count < previous or (count == previous and now - when > 4):
                    raise RuntimeError(f'{label}: {key} regressed or stalled: {snap}')
                if count > previous:
                    progress[key] = (count, now)
            if now >= end:
                if any(snap[role]['report'][key] <= baseline[role]['report'][key]
                       for role, key in (('primary', 'data_rx'), ('backup', 'sync_rx'), ('backup', 'data_tx'))):
                    raise RuntimeError(f'{label}: no end-to-end counter progress: {snap}')
                self.log(label, snap, seconds=seconds)
                return snap
            time.sleep(0.025)

    def failover(self, primary, baseline, cycle, extra_hold):
        t_before = baseline['backup']
        takeovers = t_before['report']['takeovers']
        rejoins = t_before['report']['rejoins']
        addr = t_before['report']['id']

        def took_over(snap):
            t = snap['backup']
            if t['boots'] != t_before['boots']:
                raise RuntimeError(f'Backup rebooted during cycle {cycle}: {snap}')
            return (t['seq'] > t_before['seq'] and t['report'].get('role') == 1
                    and t['report'].get('id') == addr and t['report'].get('takeovers') == takeovers + 1)

        self.log('primary reset asserted', cycle=cycle, extra_hold_s=extra_hold)
        primary.rts = True
        try:
            down = self.wait_for('backup takeover', took_over, 4)
            self.log('backup took over', down, cycle=cycle)
            time.sleep(extra_hold)
            if not took_over(self.checked()):
                raise RuntimeError(f'Backup did not remain primary while original primary was held in reset: {self.snapshot()}')
        finally:
            primary.rts = False
            self.log('primary reset released', cycle=cycle)

        def recovered(snap):
            t = snap['backup']
            if t['boots'] != t_before['boots']:
                raise RuntimeError(f'Backup rebooted during recovery in cycle {cycle}: {snap}')
            return (self.paired(snap) and snap['primary']['boots'] == baseline['primary']['boots'] + 1
                    and t['seq'] > down['backup']['seq'] and t['report']['takeovers'] == takeovers + 1
                    and t['report']['rejoins'] == rejoins + 1)

        ready = self.wait_for('backup automatically rejoining', recovered, 14)
        self.log('backup rejoined and regained backup assignment', ready, cycle=cycle)
        return self.stable('link stable after recovery', ready, 2)

    def run(self):
        try:
            with ExitStack() as stack:
                boards = {}
                for role, name in (('primary', self.args.primary), ('backup', self.args.backup)):
                    port = serial.Serial(port=None, baudrate=115200, timeout=0.15)
                    port.port, port.dtr, port.rts = name, False, False
                    boards[role] = stack.enter_context(port)
                    port.reset_input_buffer()
                workers = [threading.Thread(target=self.read, args=(role, port), daemon=True)
                           for role, port in boards.items()]
                for worker in workers:
                    worker.start()
                try:
                    baseline = self.wait_for('initial pairing', self.paired, 15)
                    self.log('baseline', baseline)
                    baseline = self.stable('steady soak passed', baseline, self.args.soak)
                    holds = (0, 0.25, 1.5, 3)
                    for cycle in range(1, self.args.cycles + 1):
                        baseline = self.failover(boards['primary'], baseline, cycle, holds[(cycle - 1) % len(holds)])
                    final = self.stable('post-failover soak passed', baseline, self.args.final_soak)
                    self.log('PASS', final, cycles=self.args.cycles)
                finally:
                    self.stop.set()
                    for worker in workers:
                        worker.join(timeout=1)
        except BaseException as exc:
            self.log('FAIL', self.snapshot(), error=repr(exc))
            raise
        finally:
            self.summary.write_text(json.dumps(dict(events=self.events, logs={role: str(path) for role, path in self.paths.items()},
                                                   ports=dict(primary=self.args.primary, backup=self.args.backup)), indent=2), encoding='utf-8')
            print(f'Summary: {self.summary}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--primary', required=True, help='UART of the primary; RTS must hold it in reset')
    parser.add_argument('--backup', required=True, help='UART of the standby-capable terminal')
    parser.add_argument('--soak', type=positive_int, default=180, help='pre-failover observation seconds')
    parser.add_argument('--cycles', type=positive_int, default=12, help='number of primary resets')
    parser.add_argument('--final-soak', type=positive_int, default=30, help='post-failover observation seconds')
    parser.add_argument('--output-dir', type=Path, default=Path('build/hardware-esp32'))
    args = parser.parse_args()
    if args.primary.casefold() == args.backup.casefold():
        parser.error('the primary and backup must be on separate ports')
    TestRun(args).run()


if __name__ == '__main__':
    main()
