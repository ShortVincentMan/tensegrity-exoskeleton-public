#!/usr/bin/env python3
"""USB recorder: complete-line buffering, raw backup, validated CSV schema."""
import argparse
import csv
import queue
import threading
import time
from pathlib import Path
import serial

FIELDS = 'time_ms,mode,state,zeroed,sensors_fresh,upper_age_ms,forearm_age_ms,upper_yaw_deg,upper_pitch_deg,upper_roll_deg,forearm_yaw_deg,forearm_pitch_deg,forearm_roll_deg,relative_rotation_deg,filtered_angle_deg,velocity_deg_s,reference_deg,dip_deg,dip_pending,assist_active,assist_target_deg,commanded_position_steps,emitted_position_steps,commanded_rate_limit_steps_s,driver_enabled,assist_count,skipped_log_rows'.split(',')
STATES = {'DISABLED', 'CALIBRATING', 'SHADOW', 'ASSIST', 'RELEASE', 'FAULT'}

class Lines:
    """Keep bytes across serial timeouts; decode only complete newline frames."""
    def __init__(self): self.pending = bytearray()
    def feed(self, chunk):
        self.pending.extend(chunk)
        result = []
        while b'\n' in self.pending:
            end = self.pending.index(b'\n')
            result.append(bytes(self.pending[:end]).rstrip(b'\r').decode('utf-8', errors='replace'))
            del self.pending[:end + 1]
        if len(self.pending) > 65536:
            raise RuntimeError('Serial stream has no newline; check matching baud rates')
        return result

def parse_data(line):
    fields = next(csv.reader([line]))[1:]
    if len(fields) != len(FIELDS):
        raise ValueError(f'expected {len(FIELDS)} fields, received {len(fields)}')
    if fields[1] not in {'BASELINE', 'ASSISTED'} or fields[2] not in STATES:
        raise ValueError('invalid mode or controller state')
    for i, value in enumerate(fields):
        if i not in (1, 2): float(value)  # nan is retained for invalid sensor values.
    return fields

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--trial', required=True)
    p.add_argument('--load', type=float, required=True)
    a = p.parse_args()
    output = Path(a.out)
    backup = output.with_suffix('.raw.log')
    for target in (output, backup):
        if target.exists(): p.error(f'{target} already exists; choose a new --out filename')
    commands = queue.Queue()
    def keyboard():
        while True:
            try: commands.put(input().strip())
            except EOFError: return
    # Open USB first: a connection failure creates no output file.
    with serial.Serial(a.port, 115200, timeout=0.05) as port, output.open('x', newline='') as f, backup.open('xb') as raw:
        writer = csv.writer(f)
        writer.writerow(['trial_id', 'load_lb'] + FIELDS)
        f.flush()
        frames = Lines()
        rows = rejected = 0
        recording_started = False
        exit_at = None
        print('115200 baud. Select STOP. Commands: baseline, assisted, status, record, end, quit.')
        print(f'CSV: {output}; raw serial backup: {backup}')
        threading.Thread(target=keyboard, daemon=True).start()
        try:
            while True:
                while not commands.empty():
                    cmd = commands.get().lower()
                    if cmd == 'quit':
                        port.write(b'end\nstop\n')
                        exit_at = time.monotonic() + 0.5
                    elif cmd == 'record' and recording_started:
                        print('Already recorded in this file. Start a new file for another trial.')
                    else: port.write((cmd + '\n').encode())
                chunk = port.read(max(1, min(port.in_waiting, 4096)))
                if chunk:
                    raw.write(chunk)
                    raw.flush()
                for line in frames.feed(chunk):
                    if line.startswith('HEADER,'):
                        received = next(csv.reader([line]))[1:]
                        if received != FIELDS:
                            port.write(b'end\nstop\n')
                            raise RuntimeError('Firmware CSV header differs from this recorder. Raw data preserved; use matching files.')
                        recording_started = True
                        print('Recording started')
                    elif line.startswith('DATA,'):
                        try: values = parse_data(line)
                        except (ValueError, csv.Error) as error:
                            rejected += 1
                            if rejected <= 3 or rejected % 25 == 0:
                                print(f'Rejected row {rejected}: {error}. Original bytes saved in {backup}.')
                            continue
                        if not recording_started:
                            print('Header was not received; using matching 27-column firmware schema.')
                            recording_started = True
                        writer.writerow([a.trial, a.load] + values)
                        rows += 1
                        if rows % 25 == 0:
                            f.flush()
                        if rows == 1 or rows % 125 == 0: print(f'Saved {rows} rows')
                    elif line: print(line)
                if exit_at is not None and time.monotonic() >= exit_at: break
        except KeyboardInterrupt:
            port.write(b'end\nstop\n')
        finally:
            f.flush()
            print(f'Saved {rows} rows; rejected {rejected}. Raw backup: {backup}')

if __name__ == '__main__': main()
