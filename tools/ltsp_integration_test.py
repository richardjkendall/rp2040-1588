#!/usr/bin/env python3
"""
LTSP Integration Test — Automated capture with serial logging and scope phase measurement.

Reboots the RX device, captures serial data from both GM and RX,
measures 1PPS phase offset from scope waveform data at ~1 Hz,
and saves everything to a timestamped output directory.

Usage:
    python3 ltsp_integration_test.py [--duration 300] [--phase-interval 1]

Requires:
    - pyserial
    - python-vxi11  (for scope waveform capture)
    - numpy
    - picotool      (for RX reboot)

Output directory (tools/runs/<timestamp>/):
    - rx_raw.csv        Raw serial from RX (all lines including comments)
    - gm_raw.log        Raw serial from GM
    - capture.csv       Parsed RX CSV data (test harness format)
    - phase.csv         Phase measurements from scope waveforms
    - summary.txt       Final test harness dashboard + verdicts
"""

import math
import os
import re
import serial
import subprocess
import sys
import threading
import time
import argparse
from datetime import datetime

# Add tools dir to path so we can import from test harness
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS_DIR)

from collections import deque
from ltsp_test_harness import (
    TestHarness, parse_gm_line, parse_gm_stats, parse_rx_csv, eprint
)
from ltsp_dashboard import render_dashboard

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SCOPE_IP = '10.10.255.228'
RX_REBOOT_CMD = ['picotool', 'reboot', '-f', '--ser', 'E66420385F341931']
GM_BAUD = 115200
RX_BAUD = 115200

# Serial port patterns
GM_PORT_PATTERN = '/dev/cu.usbmodem1101'
RX_PORT_PATTERN = '/dev/cu.usbmodem21101'


def find_serial_port(pattern):
    """Find a serial port matching pattern. Returns path or None."""
    import glob as globmod
    matches = globmod.glob(pattern)
    if matches:
        return matches[0]
    base = pattern.rstrip('0123456789')
    matches = globmod.glob(base + '*')
    return matches[0] if matches else None


def reboot_rx():
    """Reboot the RX device using picotool."""
    eprint("[REBOOT] Rebooting RX device...")
    try:
        result = subprocess.run(
            RX_REBOOT_CMD,
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            eprint(f"[REBOOT] Success: {result.stdout.strip()}")
        else:
            eprint(f"[REBOOT] picotool returned {result.returncode}: "
                   f"{result.stderr.strip()}")
    except FileNotFoundError:
        eprint("[REBOOT] ERROR: picotool not found in PATH")
        return False
    except subprocess.TimeoutExpired:
        eprint("[REBOOT] WARNING: picotool timed out (device may still reboot)")
    return True


# ---------------------------------------------------------------------------
# Scope waveform phase measurement
# ---------------------------------------------------------------------------

def parse_waveform(raw):
    """Parse Siglent binary waveform block: #<ndigits><nbytes><data>"""
    import numpy as np
    idx = raw.index(b'#')
    ndigits = int(raw[idx+1:idx+2])
    nbytes = int(raw[idx+2:idx+2+ndigits])
    data = raw[idx+2+ndigits:idx+2+ndigits+nbytes]
    return np.frombuffer(data, dtype=np.int8).astype(float)


def parse_sara(sara_raw):
    """Parse SARA response like 'SARA 100.0MSa' -> float Hz."""
    sara_val = sara_raw.split()[-1]
    multiplier = 1.0
    if 'GSa' in sara_val:
        multiplier = 1e9
    elif 'MSa' in sara_val:
        multiplier = 1e6
    elif 'kSa' in sara_val:
        multiplier = 1e3
    sara_num = re.match(r'([0-9.eE+\-]+)', sara_val)
    return float(sara_num.group(1)) * multiplier


def find_rising_edge(waveform, threshold=None):
    """Find the first rising edge crossing the threshold.
    Returns fractional sample index via linear interpolation."""
    import numpy as np
    if threshold is None:
        lo, hi = np.min(waveform), np.max(waveform)
        threshold = lo + 0.5 * (hi - lo)

    below = waveform[:-1] < threshold
    above = waveform[1:] >= threshold
    crossings = np.where(below & above)[0]

    if len(crossings) == 0:
        return None

    i = crossings[0]
    frac = (threshold - waveform[i]) / (waveform[i+1] - waveform[i])
    return i + frac


def measure_scope_phase(instr):
    """Take one phase measurement from scope waveform data.
    Returns (phase_ns, sample_rate_hz) or (None, None).

    Rejects measurements where |phase| > half the capture window,
    which indicates the scope captured CH1 and CH2 across a 1PPS boundary
    (non-simultaneous channel acquisition)."""
    sara = parse_sara(instr.ask('SARA?'))
    sample_period_ns = 1e9 / sara

    instr.write('WFSU SP,0,NP,0,FP,0,SN,0')
    ch1 = parse_waveform(instr.ask_raw(b'C1:WF? DAT2\n'))
    ch2 = parse_waveform(instr.ask_raw(b'C2:WF? DAT2\n'))

    e1 = find_rising_edge(ch1)
    e2 = find_rising_edge(ch2)

    if e1 is None or e2 is None:
        return None, sara

    return (e2 - e1) * sample_period_ns, sara


class PhaseMeasurer:
    """Measures 1PPS phase offset from scope waveform data in a background thread."""

    def __init__(self, interval_s=1.0):
        self.interval_s = interval_s
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

        # Results
        self.measurements = []    # list of (elapsed_s, phase_ns)
        self.sample_rate = None
        self.errors = 0
        self.no_edge_count = 0

    def start(self, start_time):
        self.start_time = start_time
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def get_stats(self):
        """Return (measurements_copy, mean_ns, sigma_ns, min_ns, max_ns)."""
        import numpy as np
        with self.lock:
            meas = list(self.measurements)
        if not meas:
            return meas, 0, 0, 0, 0
        phases = [m[1] for m in meas]
        arr = np.array(phases)
        return meas, arr.mean(), arr.std(), arr.min(), arr.max()

    def _run(self):
        import vxi11

        while self.running:
            try:
                instr = vxi11.Instrument(SCOPE_IP)
                instr.open()
                try:
                    phase_ns, sara = measure_scope_phase(instr)
                    self.sample_rate = sara
                finally:
                    instr.close()

                elapsed = time.time() - self.start_time

                if phase_ns is not None:
                    with self.lock:
                        self.measurements.append((elapsed, phase_ns))
                else:
                    self.no_edge_count += 1

            except Exception:
                self.errors += 1

            time.sleep(self.interval_s)


# ---------------------------------------------------------------------------
# Serial reader threads
# ---------------------------------------------------------------------------

class SerialCapture:
    """Captures serial data to file and feeds parsed data to test harness."""

    def __init__(self, name, port, baud, raw_file, harness, parse_fn):
        self.name = name
        self.port = port
        self.baud = baud
        self.raw_file = raw_file
        self.harness = harness
        self.parse_fn = parse_fn
        self.line_count = 0
        self.recent_lines = deque(maxlen=50)
        self.running = True
        self.connected = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False

    def _run(self):
        while self.running:
            try:
                ser = serial.Serial(self.port, self.baud, timeout=1)
                self.connected = True
                eprint(f"[{self.name}] Connected to {self.port}")
            except serial.SerialException as e:
                eprint(f"[{self.name}] Cannot open {self.port}: {e}")
                time.sleep(2)
                continue

            try:
                while self.running:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    if not line:
                        continue
                    self.line_count += 1
                    self.recent_lines.append(line)
                    self.raw_file.write(line + '\n')
                    self.raw_file.flush()
                    self.parse_fn(line)

            except serial.SerialException:
                self.connected = False
                eprint(f"[{self.name}] Serial connection lost, retrying...")
                time.sleep(2)


def make_gm_parser(harness):
    """Create a GM line parser closure."""
    def parse(line):
        now = time.time()
        sample = parse_gm_line(line, now)
        if sample:
            harness.add_gm_sample(sample)
        parse_gm_stats(line, harness.gm_stats)
        if 'ERR:' in line:
            harness.gm_errors += 1
    return parse


def make_rx_parser(harness, csv_file):
    """Create an RX line parser closure that also writes parsed CSV."""
    def parse(line):
        now = time.time()
        if line.startswith('#'):
            eprint(f"[RX] {line}")
            return
        sample = parse_rx_csv(line, now)
        if sample:
            harness.add_rx_sample(sample)
            csv_file.write(
                f"{sample.seq},{sample.timestamp - harness.start_time:.3f},"
                f"{sample.d_total_ns},{sample.d_detrended_ns},{sample.offset_ns},"
                f"{sample.drift_ns_per_s:.1f},{sample.drift_sigma_ns:.1f},"
                f"{sample.gm_sigma_ns:.1f},{sample.gm_a1_ppb:.3f},"
                f"{sample.clock_error_ns},{sample.sync_state}\n"
            )
            csv_file.flush()
        if 'ERR:' in line:
            harness.rx_errors += 1
    return parse


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='LTSP Integration Test')
    parser.add_argument('--duration', type=int, default=300,
                        help='Test duration in seconds (default: 300)')
    parser.add_argument('--phase-interval', type=float, default=1.0,
                        help='Phase measurement interval in seconds (default: 1.0)')
    parser.add_argument('--no-reboot', action='store_true',
                        help='Skip RX reboot at start')
    parser.add_argument('--no-scope', action='store_true',
                        help='Skip scope phase measurements')
    parser.add_argument('--gm-port', type=str, default=None,
                        help=f'GM serial port (default: {GM_PORT_PATTERN})')
    parser.add_argument('--rx-port', type=str, default=None,
                        help=f'RX serial port (default: {RX_PORT_PATTERN})')
    args = parser.parse_args()

    # Create output directory
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = os.path.join(TOOLS_DIR, 'runs', timestamp)
    os.makedirs(run_dir, exist_ok=True)
    eprint(f"Output directory: {run_dir}")

    # Find serial ports
    gm_port = args.gm_port or find_serial_port(GM_PORT_PATTERN)
    rx_port = args.rx_port or find_serial_port(RX_PORT_PATTERN)

    if not gm_port:
        eprint(f"ERROR: GM serial port not found (tried {GM_PORT_PATTERN})")
        sys.exit(1)
    if not rx_port:
        eprint(f"ERROR: RX serial port not found (tried {RX_PORT_PATTERN})")
        sys.exit(1)

    eprint(f"GM port: {gm_port}")
    eprint(f"RX port: {rx_port}")
    eprint(f"Duration: {args.duration}s")
    if not args.no_scope:
        eprint(f"Phase: waveform measurement every {args.phase_interval}s")
    eprint()

    # Reboot RX
    if not args.no_reboot:
        reboot_rx()
        eprint("[REBOOT] Waiting 5s for RX to boot...")
        time.sleep(5)

    # Open output files
    gm_raw = open(os.path.join(run_dir, 'gm_raw.log'), 'w')
    rx_raw = open(os.path.join(run_dir, 'rx_raw.csv'), 'w')
    csv_out = open(os.path.join(run_dir, 'capture.csv'), 'w')
    phase_file = open(os.path.join(run_dir, 'phase.csv'), 'w')

    csv_out.write("# seq,elapsed_s,d_total_ns,d_detrended_ns,offset_ns,"
                  "drift_ns_per_s,drift_sigma_ns,gm_sigma_ns,gm_a1_ppb,"
                  "clock_error_ns,sync_state\n")
    csv_out.flush()

    phase_file.write("# elapsed_s,phase_ns\n")
    phase_file.flush()

    # Test harness (quiet=True: CSV written by make_rx_parser, not stdout)
    harness = TestHarness(quiet=True)

    # Start serial captures
    gm_capture = SerialCapture(
        'GM', gm_port, GM_BAUD, gm_raw, harness,
        make_gm_parser(harness)
    )
    rx_capture = SerialCapture(
        'RX', rx_port, RX_BAUD, rx_raw, harness,
        make_rx_parser(harness, csv_out)
    )

    gm_capture.start()
    rx_capture.start()

    # Start phase measurement
    phase_measurer = None
    if not args.no_scope:
        phase_measurer = PhaseMeasurer(interval_s=args.phase_interval)

    eprint("Capturing... (Ctrl+C to stop early)\n")

    # Main loop
    start_time = time.time()
    last_phase_write = 0  # track how many measurements we've written

    if phase_measurer:
        phase_measurer.start(start_time)

    try:
        while True:
            elapsed = time.time() - start_time

            if elapsed >= args.duration:
                eprint(f"\nDuration reached ({args.duration}s), stopping.")
                break

            # Write new phase measurements to CSV
            if phase_measurer:
                meas, mean_ns, sigma_ns, min_ns, max_ns = phase_measurer.get_stats()
                new_count = len(meas)
                if new_count > last_phase_write:
                    for el, ph in meas[last_phase_write:]:
                        phase_file.write(f"{el:.1f},{ph:.0f}\n")
                    phase_file.flush()
                    last_phase_write = new_count

            # Dashboard every 5s
            time.sleep(5)
            elapsed = time.time() - start_time

            eprint('\033[2J\033[H', end='')
            eprint(render_dashboard(
                harness, phase_measurer,
                list(gm_capture.recent_lines),
                list(rx_capture.recent_lines),
                elapsed, args.duration,
                gm_capture.line_count, rx_capture.line_count,
                run_dir,
            ))

    except KeyboardInterrupt:
        eprint("\n\nStopped by user.")

    # Stop captures
    gm_capture.stop()
    rx_capture.stop()
    if phase_measurer:
        phase_measurer.stop()

    # Write any remaining phase data
    if phase_measurer:
        meas, mean_ns, sigma_ns, min_ns, max_ns = phase_measurer.get_stats()
        if len(meas) > last_phase_write:
            for el, ph in meas[last_phase_write:]:
                phase_file.write(f"{el:.1f},{ph:.0f}\n")
            phase_file.flush()

    # Final dashboard
    final_dashboard = harness.dashboard()
    eprint("\n" + final_dashboard)

    # Phase summary
    phase_summary = ""
    if phase_measurer:
        meas, mean_ns, sigma_ns, min_ns, max_ns = phase_measurer.get_stats()
        n = len(meas)
        if n > 0:
            phases_ns = [m[1] for m in meas]

            # Compute drift via linear regression
            drift_str = ""
            if n > 10:
                import numpy as np
                ts = np.array([m[0] for m in meas])
                ps = np.array(phases_ns)
                # Linear fit: phase = a*t + b
                coeffs = np.polyfit(ts, ps, 1)
                drift_ns_per_s = coeffs[0]
                drift_str = f"  Drift: {drift_ns_per_s:+.1f} ns/s = {drift_ns_per_s/1000:+.3f} us/s"

            phase_summary = (
                f"\n1PPS Phase (from scope waveform, {parse_sara.__module__} @ "
                f"{phase_measurer.sample_rate/1e6:.0f} MSa/s):\n"
                f"  Measurements: {n}\n"
                f"  Mean:  {mean_ns:+.0f} ns = {mean_ns/1000:+.1f} us\n"
                f"  Sigma: {sigma_ns:.0f} ns = {sigma_ns/1000:.1f} us\n"
                f"  Min:   {min_ns:+.0f} ns = {min_ns/1000:+.1f} us\n"
                f"  Max:   {max_ns:+.0f} ns = {max_ns/1000:+.1f} us\n"
                f"  P-P:   {max_ns-min_ns:.0f} ns = {(max_ns-min_ns)/1000:.1f} us"
            )
            if drift_str:
                phase_summary += f"\n{drift_str}"
            if phase_measurer.no_edge_count > 0:
                phase_summary += f"\n  No-edge: {phase_measurer.no_edge_count}"

            eprint(phase_summary)

    # Save summary
    summary_path = os.path.join(run_dir, 'summary.txt')
    with open(summary_path, 'w') as f:
        f.write(f"LTSP Integration Test — {timestamp}\n")
        f.write(f"Duration: {time.time() - start_time:.0f}s\n")
        f.write(f"GM port: {gm_port}\n")
        f.write(f"RX port: {rx_port}\n")
        f.write(f"GM lines: {gm_capture.line_count}\n")
        f.write(f"RX lines: {rx_capture.line_count}\n")
        if phase_measurer:
            f.write(f"Phase measurements: {len(phase_measurer.measurements)}\n")
            f.write(f"Phase interval: {args.phase_interval}s\n")
            if phase_measurer.sample_rate:
                f.write(f"Scope sample rate: {phase_measurer.sample_rate/1e6:.0f} MSa/s\n")
        f.write(f"\n{final_dashboard}\n")
        if phase_summary:
            f.write(f"\n{phase_summary}\n")

    # Close files
    gm_raw.close()
    rx_raw.close()
    csv_out.close()
    phase_file.close()

    eprint(f"\nResults saved to: {run_dir}")
    eprint(f"  summary.txt   — Dashboard, verdicts, phase summary")
    eprint(f"  rx_raw.csv    — Raw RX serial ({rx_capture.line_count} lines)")
    eprint(f"  gm_raw.log    — Raw GM serial ({gm_capture.line_count} lines)")
    eprint(f"  capture.csv   — Parsed RX data")
    eprint(f"  phase.csv     — Phase measurements ({last_phase_write} points)")


if __name__ == '__main__':
    main()
