#!/usr/bin/env python3
"""
Continuous GPS signal monitor using px4-listener.
Tracks sensor_gps (noise, jamming, satellites used) and
satellite_info (per-satellite SNR) simultaneously.

Usage: python3 monitor_gps.py
       Ctrl+C to exit.
"""

import subprocess
import threading
import re
import time
import sys
from datetime import datetime

# ── ANSI colours ──────────────────────────────────────────────────────────────
RESET  = '\033[0m'
BOLD   = '\033[1m'
DIM    = '\033[2m'
GREEN  = '\033[32m'
YELLOW = '\033[33m'
RED    = '\033[31m'
CYAN   = '\033[36m'

JAMMING_LABELS = {
    0: ('UNKNOWN',  YELLOW),
    1: ('OK',       GREEN),
    2: ('WARNING',  YELLOW),
    3: ('CRITICAL', RED),
}

# ── Shared state ──────────────────────────────────────────────────────────────
state = {
    'noise_per_ms':      None,
    'jamming_indicator': None,
    'jamming_state':     None,
    'satellites_used':   None,
    'gps_updated':       None,
    'sat_count':       None,
    'snr':             [],
    'sat_updated':     None,
}
lock    = threading.Lock()
running = True

# ── Parsers ───────────────────────────────────────────────────────────────────
def parse_sensor_gps(line):
    line = line.strip()
    for pattern, key in [
        (r'noise_per_ms:\s*(\d+)',      'noise_per_ms'),
        (r'jamming_indicator:\s*(\d+)', 'jamming_indicator'),
        (r'jamming_state:\s*(\d+)',     'jamming_state'),
        (r'satellites_used:\s*(\d+)',   'satellites_used'),
    ]:
        m = re.match(pattern, line)
        if m:
            with lock:
                state[key] = int(m.group(1))
                state['gps_updated'] = datetime.now()
            return


# satellite_info outputs arrays on a single line, e.g.:
#   count: 20
#   snr: [0, 0, 25, 31, 0, ...]
# count is stored when seen; snr triggers the state commit.
_current_count = None

def parse_satellite_info(line):
    global _current_count
    line = line.strip()

    m = re.match(r'count:\s*(\d+)', line)
    if m:
        _current_count = int(m.group(1))
        return

    m = re.match(r'snr:\s*\[([^\]]+)\]', line)
    if m:
        values = [int(v.strip()) for v in m.group(1).split(',')]
        n = _current_count if _current_count is not None else len(values)
        with lock:
            state['sat_count']   = n
            state['snr']         = values[:n]
            state['sat_updated'] = datetime.now()

# ── Reader threads ─────────────────────────────────────────────────────────────
def reader(topic, parse_fn):
    # -n 0 = unlimited messages; restart loop handles unexpected exits
    cmd = ['px4-listener', topic, '-n', '0']
    while running:
        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                universal_newlines=True, bufsize=1
            )
            while running:
                line = proc.stdout.readline()
                if not line:
                    break
                parse_fn(line)
            proc.terminate()
        except Exception as e:
            sys.stderr.write(f"Reader error ({topic}): {e}\n")
        if running:
            time.sleep(0.2)

# ── Display ───────────────────────────────────────────────────────────────────
def snr_bar(snr, width=24):
    filled = int(min(snr, 50) / 50 * width)
    color  = GREEN if snr >= 35 else YELLOW if snr >= 20 else RED
    return f"{color}{'█' * filled}{'░' * (width - filled)}{RESET}"


def display():
    with lock:
        snap     = dict(state)
        snr_list = list(snap['snr'])

    now = datetime.now().strftime('%H:%M:%S')
    print('\033[2J\033[H', end='')   # clear screen, move cursor home
    print(f"{BOLD}{CYAN}╔══ GPS Signal Monitor ══╗{RESET}  {DIM}{now}{RESET}\n")

    # ── sensor_gps ────────────────────────────────────────────────────────────
    print(f"{BOLD}sensor_gps{RESET}")

    sats = snap['satellites_used']
    print(f"  satellites_used : {BOLD}{sats if sats is not None else '--'}{RESET}")

    noise = snap['noise_per_ms']
    print(f"  noise_per_ms    : {noise if noise is not None else '--'}")

    ji = snap['jamming_indicator']
    print(f"  jamming_indicator: {ji if ji is not None else '--'}")

    js = snap['jamming_state']
    if js is not None:
        label, color = JAMMING_LABELS.get(js, ('UNKNOWN', YELLOW))
        print(f"  jamming_state   : {color}{BOLD}{label}{RESET}  ({js})")
    else:
        print(f"  jamming_state   : --")

    upd = snap['gps_updated']
    print(f"  {DIM}updated: {upd.strftime('%H:%M:%S') if upd else 'waiting...'}{RESET}")

    print()

    # ── satellite_info ────────────────────────────────────────────────────────
    count = snap['sat_count']
    print(f"{BOLD}satellite_info{RESET}  ({count if count is not None else '--'} satellites tracked)\n")

    if snr_list:
        print(f"  {'#':>3}  {'SNR':>5}  signal strength")
        print(f"  {'─'*3}  {'─'*5}  {'─'*24}")
        for i, snr in enumerate(snr_list):
            print(f"  {i:>3}  {snr:>3} dB  {snr_bar(snr)}")
    else:
        print(f"  {DIM}(waiting for data...){RESET}")

    upd = snap['sat_updated']
    print(f"\n  {DIM}updated: {upd.strftime('%H:%M:%S') if upd else 'waiting...'}{RESET}")

    print(f"\n{DIM}Ctrl+C to exit{RESET}")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global running

    print('\033[?25l', end='', flush=True)  # hide cursor

    t1 = threading.Thread(
        target=reader,
        args=('sensor_gps', parse_sensor_gps),
        daemon=True,
    )
    t2 = threading.Thread(
        target=reader,
        args=('satellite_info', parse_satellite_info),
        daemon=True,
    )
    t1.start()
    t2.start()

    try:
        while True:
            display()
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        print('\033[?25h', end='', flush=True)  # restore cursor
        print()


if __name__ == '__main__':
    main()
