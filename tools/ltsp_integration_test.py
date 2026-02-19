#!/usr/bin/env python3
"""
LTSP Integration Test — Automated capture with serial logging and scope phase measurement.

Reboots the RX device, captures serial data from both GM and RX,
takes periodic scope screenshots, measures 1PPS phase offset from
the scope image, and saves everything to a timestamped output directory.

Usage:
    python3 ltsp_integration_test.py [--duration 300] [--scope-interval 60]

Requires:
    - pyserial
    - python-vxi11  [optional, for scope captures]
    - Pillow        [optional, for scope image analysis]
    - picotool      (for RX reboot)

Output directory (tools/runs/<timestamp>/):
    - rx_raw.csv        Raw serial from RX (all lines including comments)
    - gm_raw.log        Raw serial from GM
    - capture.csv       Parsed RX CSV data (test harness format)
    - scope_<NNN>s.bmp  Scope screenshots at configured intervals
    - phase.csv         Phase measurements from scope images
    - summary.txt       Final test harness dashboard + verdicts
"""

import sys
import os
import serial
import subprocess
import threading
import time
import argparse
import glob as globmod
from datetime import datetime

# Add tools dir to path so we can import from test harness
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS_DIR)

from ltsp_test_harness import (
    TestHarness, parse_gm_line, parse_gm_stats, parse_rx_csv, eprint
)

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

# Trace colors (exact RGB from scope BMP)
YELLOW_RGB = (255, 255, 0)    # CH1 = GM
PINK_RGB = (205, 0, 205)      # CH2 = RX


def find_serial_port(pattern):
    """Find a serial port matching pattern. Returns path or None."""
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
# Scope capture and phase measurement
# ---------------------------------------------------------------------------

def capture_scope_bmp(output_path):
    """Capture a scope screenshot via VXI-11. Returns True on success."""
    try:
        import vxi11
    except ImportError:
        eprint("[SCOPE] vxi11 not available, skipping screenshot")
        return False

    try:
        instr = vxi11.Instrument(SCOPE_IP)
        instr.open()
        data = instr.ask_raw(b'SCDP\n')
        with open(output_path, 'wb') as f:
            f.write(data)
        instr.close()
        return True
    except Exception as e:
        eprint(f"[SCOPE] Capture error: {e}")
        return False


def detect_grid_spacing(img):
    """
    Auto-detect the horizontal grid spacing (pixels per division) from
    gray grid-line dots along the center horizontal axis.

    Returns px_per_div or None if detection fails.
    """
    w, h = img.size
    pixels = img.load()
    mid_y = h // 2 + 60  # Slightly below center to hit the center grid line

    # Find gray pixels along this row (grid dots/dashes)
    gray_xs = []
    for x in range(w):
        r, g, b = pixels[x, mid_y][:3]
        if 30 < r < 140 and abs(r - g) < 10 and abs(g - b) < 10:
            gray_xs.append(x)

    # Keep only internal dots (exclude left/right border clusters)
    if len(gray_xs) < 4:
        return None
    left_edge = gray_xs[0]
    right_edge = gray_xs[-1]
    margin = (right_edge - left_edge) * 0.05
    internal = [x for x in gray_xs if x > left_edge + margin and x < right_edge - margin]

    if len(internal) < 3:
        return None

    # Cluster adjacent pixels (grid lines can be 1-3 px wide)
    clusters = []
    for x in internal:
        if not clusters or x - clusters[-1][-1] > 5:
            clusters.append([x])
        else:
            clusters[-1].append(x)

    centers = [sum(c) / len(c) for c in clusters]
    if len(centers) < 3:
        return None

    # Compute spacings between adjacent grid lines
    spacings = [centers[i + 1] - centers[i] for i in range(len(centers) - 1)]

    # Filter outliers (e.g. sub-division ticks) — keep spacings within 20% of median
    spacings.sort()
    median = spacings[len(spacings) // 2]
    valid = [s for s in spacings if abs(s - median) / median < 0.2]

    if not valid:
        return None

    return sum(valid) / len(valid)


def measure_phase_from_bmp(bmp_path, time_per_div_us):
    """
    Measure the 1PPS phase offset between yellow (GM) and pink (RX) traces
    by finding the rising edge of each in the scope BMP image.

    Grid spacing (px/div) is auto-detected from the gray grid dots.

    Returns (phase_us, yellow_edge_x, pink_edge_x) or (None, None, None) on failure.
    Phase is positive when RX rises after GM (RX lags).
    """
    try:
        from PIL import Image
    except ImportError:
        return None, None, None

    try:
        img = Image.open(bmp_path)
    except Exception:
        return None, None, None

    w, h = img.size
    pixels = img.load()

    # Auto-detect grid spacing
    px_per_div = detect_grid_spacing(img)
    if px_per_div is None:
        return None, None, None

    # For each x column, find the minimum Y for each trace color.
    # Min Y = topmost pixel = highest voltage point.
    # Baseline (LOW) has high min_y, pulse (HIGH) has low min_y.
    yellow_min_y = {}
    pink_min_y = {}

    for y in range(h):
        for x in range(w):
            r, g, b = pixels[x, y][:3]
            if (r, g, b) == YELLOW_RGB:
                if x not in yellow_min_y or y < yellow_min_y[x]:
                    yellow_min_y[x] = y
            elif (r, g, b) == PINK_RGB:
                if x not in pink_min_y or y < pink_min_y[x]:
                    pink_min_y[x] = y

    if not yellow_min_y or not pink_min_y:
        return None, None, None

    # Determine baseline and pulse Y levels for each trace
    yellow_ys = sorted(yellow_min_y.values())
    pink_ys = sorted(pink_min_y.values())

    yellow_baseline = yellow_ys[len(yellow_ys) * 3 // 4]  # 75th percentile
    yellow_pulse = yellow_ys[len(yellow_ys) // 10]         # 10th percentile
    pink_baseline = pink_ys[len(pink_ys) * 3 // 4]
    pink_pulse = pink_ys[len(pink_ys) // 10]

    # Threshold at midpoint between baseline and pulse
    yellow_thresh = (yellow_baseline + yellow_pulse) // 2
    pink_thresh = (pink_baseline + pink_pulse) // 2

    def find_rising_edge(min_y_by_x, threshold, baseline):
        """Find x of first rising edge (min_y drops below threshold)."""
        sorted_xs = sorted(min_y_by_x.keys())
        in_baseline = False
        for x in sorted_xs:
            y = min_y_by_x[x]
            if y >= baseline - 20:
                in_baseline = True
            elif in_baseline and y < threshold:
                return x
        return None

    yellow_edge = find_rising_edge(yellow_min_y, yellow_thresh, yellow_baseline)
    pink_edge = find_rising_edge(pink_min_y, pink_thresh, pink_baseline)

    if yellow_edge is None or pink_edge is None:
        return None, None, None

    # Convert pixel distance to time
    us_per_px = time_per_div_us / px_per_div
    phase_us = (pink_edge - yellow_edge) * us_per_px

    return phase_us, yellow_edge, pink_edge


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
    parser.add_argument('--scope-interval', type=int, default=60,
                        help='Scope screenshot interval in seconds (default: 60)')
    parser.add_argument('--no-reboot', action='store_true',
                        help='Skip RX reboot at start')
    parser.add_argument('--no-scope', action='store_true',
                        help='Skip scope screenshots')
    parser.add_argument('--time-div', type=float, default=1000.0,
                        help='Scope time base in us/div (default: 1000 = 1ms/div)')
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
    eprint(f"Scope: interval={args.scope_interval}s, time_div={args.time_div} us/div")
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
    phase_csv = open(os.path.join(run_dir, 'phase.csv'), 'w')

    csv_out.write("# seq,elapsed_s,d_total_ns,d_detrended_ns,offset_ns,"
                  "drift_ns_per_s,drift_sigma_ns,gm_sigma_ns,gm_a1_ppb,"
                  "clock_error_ns,sync_state\n")
    csv_out.flush()

    phase_csv.write("# elapsed_s,phase_us,yellow_edge_px,pink_edge_px,file\n")
    phase_csv.flush()

    # Test harness
    harness = TestHarness()

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

    eprint("Capturing... (Ctrl+C to stop early)\n")

    # Main loop
    start_time = time.time()
    last_scope_time = 0
    scope_count = 0
    phase_measurements = []

    try:
        while True:
            elapsed = time.time() - start_time

            if elapsed >= args.duration:
                eprint(f"\nDuration reached ({args.duration}s), stopping.")
                break

            # Scope capture + phase measurement at intervals
            if (not args.no_scope and
                    elapsed - last_scope_time >= args.scope_interval and
                    elapsed > 5):
                scope_count += 1
                scope_file = f'scope_{int(elapsed):04d}s.bmp'
                scope_path = os.path.join(run_dir, scope_file)

                if capture_scope_bmp(scope_path):
                    phase_us, yw_x, pk_x = measure_phase_from_bmp(
                        scope_path, args.time_div)

                    if phase_us is not None:
                        phase_measurements.append(phase_us)
                        phase_csv.write(
                            f"{elapsed:.1f},{phase_us:.1f},{yw_x},{pk_x},{scope_file}\n")
                        phase_csv.flush()
                        eprint(f"[SCOPE] {scope_file}: phase = {phase_us:+.0f} us "
                               f"(GM@px{yw_x}, RX@px{pk_x})")
                    else:
                        eprint(f"[SCOPE] {scope_file}: could not measure phase")
                        phase_csv.write(
                            f"{elapsed:.1f},,,, {scope_file}\n")
                        phase_csv.flush()

                last_scope_time = elapsed

            # Dashboard every 5s
            time.sleep(5)
            elapsed = time.time() - start_time

            eprint('\033[2J\033[H', end='')
            eprint(harness.dashboard())

            # Phase summary
            if phase_measurements:
                latest = phase_measurements[-1]
                import math
                mean_ph = sum(phase_measurements) / len(phase_measurements)
                if len(phase_measurements) > 1:
                    variance = sum((p - mean_ph)**2 for p in phase_measurements) / (
                        len(phase_measurements) - 1)
                    std_ph = math.sqrt(variance)
                else:
                    std_ph = 0.0
                eprint(f"\n SCOPE | Latest phase: {latest:+.0f} us  "
                       f"Mean: {mean_ph:+.0f} us  Sigma: {std_ph:.0f} us  "
                       f"({len(phase_measurements)} captures)")
            else:
                eprint(f"\n SCOPE | No phase measurements yet")

            eprint(f"\n [{elapsed:.0f}s / {args.duration}s]  "
                   f"GM: {gm_capture.line_count} lines  "
                   f"RX: {rx_capture.line_count} lines")
            eprint(f" Output: {run_dir}")

    except KeyboardInterrupt:
        eprint("\n\nStopped by user.")

    # Stop captures
    gm_capture.stop()
    rx_capture.stop()

    # Final dashboard
    final_dashboard = harness.dashboard()
    eprint("\n" + final_dashboard)

    # Phase summary
    phase_summary = ""
    if phase_measurements:
        import math
        mean_ph = sum(phase_measurements) / len(phase_measurements)
        if len(phase_measurements) > 1:
            variance = sum((p - mean_ph)**2 for p in phase_measurements) / (
                len(phase_measurements) - 1)
            std_ph = math.sqrt(variance)
        else:
            std_ph = 0.0
        min_ph = min(phase_measurements)
        max_ph = max(phase_measurements)
        phase_summary = (
            f"\n1PPS Phase (from scope):\n"
            f"  Measurements: {len(phase_measurements)}\n"
            f"  Mean:  {mean_ph:+.0f} us\n"
            f"  Sigma: {std_ph:.0f} us\n"
            f"  Min:   {min_ph:+.0f} us\n"
            f"  Max:   {max_ph:+.0f} us\n"
            f"  Drift: {(phase_measurements[-1] - phase_measurements[0]):.0f} us "
            f"over {args.duration}s"
            if len(phase_measurements) > 1 else ""
        )
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
        f.write(f"Scope captures: {scope_count}\n")
        f.write(f"Scope time base: {args.time_div} us/div\n")
        f.write(f"\n{final_dashboard}\n")
        if phase_summary:
            f.write(f"\n{phase_summary}\n")

    # Close files
    gm_raw.close()
    rx_raw.close()
    csv_out.close()
    phase_csv.close()

    eprint(f"\nResults saved to: {run_dir}")
    eprint(f"  summary.txt   — Dashboard, verdicts, phase summary")
    eprint(f"  rx_raw.csv    — Raw RX serial ({rx_capture.line_count} lines)")
    eprint(f"  gm_raw.log    — Raw GM serial ({gm_capture.line_count} lines)")
    eprint(f"  capture.csv   — Parsed RX data")
    eprint(f"  phase.csv     — Phase measurements from scope")
    eprint(f"  scope_*.bmp   — {scope_count} scope screenshots")


if __name__ == '__main__':
    main()
