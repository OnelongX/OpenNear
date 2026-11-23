"""Collect simultaneous UART logs; host times are only for approximate matching."""
import argparse
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path
import threading
import time
import serial


def open_port(name):
    port = serial.Serial(port=None, baudrate=115200, timeout=0.2)
    port.port, port.dtr, port.rts = name, False, False
    return port


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--primary', required=True)
    p.add_argument('--terminal', required=True)
    p.add_argument('--seconds', type=int, default=60)
    p.add_argument('--output-dir', type=Path, default=Path('build/gpio-sync'))
    a = p.parse_args()
    if a.primary.casefold() == a.terminal.casefold() or not 5 <= a.seconds <= 300:
        p.error('use two distinct ports and a duration of 5..300 seconds')
    a.output_dir.mkdir(parents=True, exist_ok=True)
    prefix = a.output_dir.resolve() / datetime.now().strftime('act-trace-%Y%m%d-%H%M%S-%f')
    failures = []
    with ExitStack() as stack:
        ports = [stack.enter_context(open_port(port)) for port in (a.primary, a.terminal)]
        paths = [prefix.with_name(f'{prefix.name}-{role}.log') for role in ('primary', 'terminal')]
        files = [stack.enter_context(path.open('w', encoding='utf-8', buffering=1)) for path in paths]
        for port in ports:
            port.reset_input_buffer()
        start = time.monotonic_ns()
        deadline = start + a.seconds * 1_000_000_000

        def reader(port, dest):
            try:
                pending = bytearray()
                while time.monotonic_ns() < deadline:
                    # Read buffered bytes in batches. readline() can issue a serial
                    # read per byte, which falls behind when both ports are busy.
                    pending.extend(port.read(max(1, port.in_waiting)))
                    lines = pending.split(b'\n')
                    pending = lines.pop()
                    if lines:
                        stamp = (time.monotonic_ns() - start) / 1e9
                        dest.writelines(f'{stamp:.6f} {line.decode("ascii", "replace").strip()}\n'
                                        for line in lines if line.strip())
            except Exception as exc:
                failures.append(repr(exc))

        threads = [threading.Thread(target=reader, args=pair) for pair in zip(ports, files)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
    for path in paths:
        print(path)
    if failures:
        raise RuntimeError(failures)


if __name__ == '__main__':
    main()
