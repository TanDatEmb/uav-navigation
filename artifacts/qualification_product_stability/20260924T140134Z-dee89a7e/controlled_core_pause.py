#!/usr/bin/env python3
"""Run one diagnostic SITL and pause only its exact Core runtime PID once."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path('/home/letandat/Dev/uav-navigation-product-stability-20260924')
ARTIFACT = Path('/home/letandat/Dev/uav-navigation/.artifacts/runtime')
OUT = ROOT / '.artifacts/runtime/qpst-core-pause-index.json'
LOG = Path('/tmp/qpst-core-pause-run.log')


def core_pid(session: Path) -> int | None:
    registry = session / 'processes.json'
    if not registry.exists():
        return None
    try:
        groups = {int(r['pgid']) for r in json.loads(registry.read_text())['processes']
                  if r.get('role') == 'mapping'}
    except (OSError, ValueError, KeyError):
        return None
    matching = []
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            raw = (proc / 'stat').read_text()
            pgrp = int(raw[raw.rfind(')') + 2:].split()[2])
            cmd = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode().strip()
            if pgrp in groups and '/navigation_runtime_node' in cmd and 'ros2 run' not in cmd:
                matching.append(int(proc.name))
        except (OSError, ValueError):
            pass
    return matching[0] if len(matching) == 1 else None


def main() -> None:
    before = {p for p in ARTIFACT.glob('external-mode-check-*') if p.is_dir()}
    command = ['python3', 'tools/runtime/run_state_transport_cohort.py',
               '--count', '1', '--label', 'qpst-core-pause',
               '--gazebo-native-diagnostic', '--output', str(OUT)]
    with LOG.open('w') as log:
        process = subprocess.Popen(command, cwd=ROOT, stdout=log,
                                   stderr=subprocess.STDOUT)
        session = None
        file_offset = 0
        deadline = time.monotonic() + 240
        record = {'command': command, 'applied': False}
        try:
            while process.poll() is None and time.monotonic() < deadline:
                if session is None:
                    created = [p for p in ARTIFACT.glob('external-mode-check-*')
                               if p.is_dir() and p not in before]
                    if len(created) == 1:
                        session = created[0]
                        record['session'] = str(session)
                if session is not None:
                    scenario = session / 'scenario.jsonl'
                    if scenario.exists():
                        with scenario.open() as stream:
                            stream.seek(file_offset)
                            while line := stream.readline():
                                file_offset = stream.tell()
                                try:
                                    row = json.loads(line)
                                except ValueError:
                                    continue
                                if row.get('kind') != 'pva_command' or \
                                   int(row.get('payload', {}).get('request_id') or 0) < 2:
                                    continue
                                pid = core_pid(session)
                                if pid is None:
                                    continue
                                record['trigger_request_id'] = row['payload']['request_id']
                                record['core_pid'] = pid
                                record['pause_start_wall_ns'] = time.time_ns()
                                record['pause_start_steady_ns'] = time.monotonic_ns()
                                os.kill(pid, signal.SIGSTOP)
                                try:
                                    time.sleep(0.35)
                                finally:
                                    os.kill(pid, signal.SIGCONT)
                                record['pause_end_steady_ns'] = time.monotonic_ns()
                                record['pause_duration_ms'] = (
                                    record['pause_end_steady_ns'] -
                                    record['pause_start_steady_ns']) / 1e6
                                record['applied'] = True
                                break
                            if record['applied']:
                                break
                time.sleep(0.05)
            record['runner_exit_code'] = process.wait(timeout=240)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=20)
        print(json.dumps(record, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
