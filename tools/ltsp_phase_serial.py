#!/usr/bin/env python3
"""
LTSP Phase Measurement — Serial capture with live dashboard.

Reads CSV phase data from USB serial (measurement device firmware),
logs to file, and displays a live dashboard with phase, TIE, and MTIE.

Usage:
    source tools/.env/bin/activate
    python3 tools/ltsp_phase_serial.py --port /dev/cu.usbmodemXXXX [--duration 3600]
"""

import argparse
import collections
import math
import os
import shutil
import signal
import sys
import time
from datetime import datetime

import serial

# Import braille chart renderer from dashboard module
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ltsp_dashboard import render_braille_chart

# ---------------------------------------------------------------------------
# Box drawing characters
# ---------------------------------------------------------------------------
BOX_H = "\u2500"
BOX_V = "\u2502"
BOX_TL = "\u250c"
BOX_TR = "\u2510"
BOX_BL = "\u2514"
BOX_BR = "\u2518"
BOX_LT = "\u251c"
BOX_RT = "\u2524"


# ---------------------------------------------------------------------------
# Welford's running statistics
# ---------------------------------------------------------------------------
class WelfordStats:
    __slots__ = ("n", "mean", "_m2", "min_val", "max_val")

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self._m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, x):
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self._m2 += d1 * d2
        if x < self.min_val:
            self.min_val = x
        if x > self.max_val:
            self.max_val = x

    @property
    def sigma(self):
        return math.sqrt(self._m2 / (self.n - 1)) if self.n > 1 else 0.0

    @property
    def pp(self):
        return self.max_val - self.min_val if self.n > 0 else 0.0


# ---------------------------------------------------------------------------
# MTIE computation (sliding window max peak-to-peak of TIE)
# ---------------------------------------------------------------------------
class MTIECalculator:
    """Incrementally compute MTIE for fixed window sizes over TIE series."""

    def __init__(self, windows=(1, 10, 100, 1000)):
        self.windows = windows
        # For each window size, track min/max of TIE using deques
        self._tie_buf = collections.deque()  # full TIE history (bounded)
        self._max_history = max(windows) + 1
        self._results = {w: 0.0 for w in windows}

    def update(self, tie_value):
        self._tie_buf.append(tie_value)
        # Keep buffer bounded
        if len(self._tie_buf) > self._max_history:
            self._tie_buf.popleft()

    def compute(self):
        """Recompute MTIE for all windows. Call periodically (not every sample)."""
        buf = list(self._tie_buf)
        n = len(buf)
        for w in self.windows:
            if n < w:
                self._results[w] = 0.0
                continue
            max_pp = 0.0
            # Slide window of size w across the buffer
            # Use a simple O(n*w) approach — fine for 1 Hz data up to 1000s
            for i in range(n - w + 1):
                window = buf[i:i + w]
                pp = max(window) - min(window)
                if pp > max_pp:
                    max_pp = pp
            self._results[w] = max_pp
        return self._results

    def get(self):
        return dict(self._results)


# ---------------------------------------------------------------------------
# Dashboard renderer
# ---------------------------------------------------------------------------
def render_dashboard(stats, phase_buf, tie_buf, tie_stats, mtie,
                     gps_locked, crystal_ns, rejected, elapsed,
                     run_dir, n_comments):
    """Render single-column box-drawing dashboard."""
    tw = min(shutil.get_terminal_size((80, 40)).columns, 80)
    inner = tw - 2  # content width between │ and │

    def hline(left, right, title=None):
        if title:
            t = f" {title} "
            pad = inner - len(t)
            lp = pad // 2
            rp = pad - lp
            return left + BOX_H * lp + t + BOX_H * rp + right
        return left + BOX_H * inner + right

    def row(text):
        t = text[:inner]
        return BOX_V + t.ljust(inner) + BOX_V

    lines = []

    # Top border
    lines.append(hline(BOX_TL, BOX_TR, "Phase Measurement"))

    # Header
    gps_str = "YES" if gps_locked else "NO"
    crystal_str = f"{crystal_ns:+d}" if crystal_ns is not None else "?"
    lines.append(row(
        f" n={stats.n}  elapsed={elapsed:.0f}s  "
        f"GPS: {gps_str}  Crystal: {crystal_str} ns"
        + (f"  rejected={rejected}" if rejected > 0 else "")
    ))

    # Phase section
    lines.append(hline(BOX_LT, BOX_RT))
    if stats.n > 0:
        lines.append(row(
            f" Phase: mean={stats.mean/1000:+.1f} us  "
            f"sigma={stats.sigma/1000:.1f} us  "
            f"P-P={stats.pp/1000:.1f} us"
        ))
        # Braille chart of last 300 phase samples (in µs)
        chart_data = [v / 1000.0 for v in list(phase_buf)[-300:]]
        if len(chart_data) >= 2:
            chart_lines = render_braille_chart(chart_data, width=min(40, inner - 8), height=3)
            for cl in chart_lines:
                lines.append(row(f" {cl}"))
    else:
        lines.append(row(" Phase: waiting for data..."))

    # TIE section
    lines.append(hline(BOX_LT, BOX_RT))
    if tie_stats.n > 0:
        lines.append(row(
            f" TIE:  max={tie_stats.max_val/1000:+.1f} us  "
            f"min={tie_stats.min_val/1000:+.1f} us  "
            f"P-P={tie_stats.pp/1000:.1f} us"
        ))
        # Braille chart of last 300 TIE samples
        tie_data = [v / 1000.0 for v in list(tie_buf)[-300:]]
        if len(tie_data) >= 2:
            chart_lines = render_braille_chart(tie_data, width=min(40, inner - 8), height=3)
            for cl in chart_lines:
                lines.append(row(f" {cl}"))
    else:
        lines.append(row(" TIE:  waiting for data..."))

    # MTIE section
    lines.append(hline(BOX_LT, BOX_RT))
    mtie_vals = mtie.get()
    mtie_parts = []
    for w in mtie.windows:
        v = mtie_vals.get(w, 0.0)
        if v > 0:
            mtie_parts.append(f"{w}s: {v/1000:.1f} us")
        else:
            mtie_parts.append(f"{w}s: --")
    lines.append(row(f" MTIE: {'   '.join(mtie_parts)}"))

    # Footer
    lines.append(hline(BOX_LT, BOX_RT))
    short_dir = "/".join(run_dir.rstrip("/").split("/")[-2:]) if run_dir else ""
    lines.append(row(f" Output: {short_dir}  ({n_comments} status lines)"))
    lines.append(hline(BOX_BL, BOX_BR))

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="LTSP phase measurement serial capture + dashboard")
    parser.add_argument("--port", required=True, help="Serial port path")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--duration", type=int, default=0,
                        help="Stop after N seconds (0 = run forever)")
    args = parser.parse_args()

    # Create run directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "runs", timestamp)
    os.makedirs(run_dir, exist_ok=True)
    csv_path = os.path.join(run_dir, "phase_serial.csv")

    # State
    stats = WelfordStats()
    tie_stats = WelfordStats()
    phase_buf = collections.deque(maxlen=1200)  # ~20 min at 1 Hz
    tie_buf = collections.deque(maxlen=1200)
    mtie = MTIECalculator(windows=(1, 10, 100, 1000))
    first_phase = None
    gps_locked = False
    crystal_ns = None
    rejected = 0
    n_comments = 0
    last_render = 0.0

    # Graceful shutdown
    running = True

    def handle_signal(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print(f"Opening {args.port} at {args.baud} baud...")
    print(f"Logging to {csv_path}")
    print("Press Ctrl+C to stop.\n")

    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    start_time = time.monotonic()

    with open(csv_path, "w") as f:
        # Write CSV header
        f.write(f"# LTSP Phase Measurement Serial Capture\n")
        f.write(f"# Started: {datetime.now().isoformat()}\n")
        f.write(f"# Port: {args.port}\n")
        f.write(f"# Columns: seq,phase_ns,gm_to_slave_ns,slave_to_gm_ns,"
                f"scale_factor,gm_first,crystal_error_ns\n")
        f.flush()

        while running:
            elapsed = time.monotonic() - start_time
            if args.duration > 0 and elapsed >= args.duration:
                break

            # Read a line from serial
            try:
                raw = ser.readline()
            except serial.SerialException:
                print("\nSerial connection lost.", file=sys.stderr)
                break

            if not raw:
                # Timeout — no data, still refresh dashboard
                now = time.monotonic()
                if now - last_render >= 1.0 and stats.n > 0:
                    _refresh_dashboard(stats, phase_buf, tie_buf, tie_stats,
                                       mtie, gps_locked, crystal_ns, rejected,
                                       elapsed, run_dir, n_comments)
                    last_render = now
                continue

            try:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                continue

            if not line:
                continue

            # Log all raw lines
            f.write(line + "\n")
            f.flush()

            if line.startswith("#"):
                # Status/comment line — extract useful info
                n_comments += 1
                _parse_comment(line)
                # Extract GPS and crystal info from stats lines
                if "GPS crystal:" in line:
                    try:
                        # Format: "# GPS crystal: +29424 ns (+0.029 ppm)"
                        parts = line.split("GPS crystal:")[1].strip().split()
                        crystal_ns = int(parts[0])
                        gps_locked = True
                    except (IndexError, ValueError):
                        pass
                elif "GPS:" in line and "YES" in line:
                    gps_locked = True
                continue

            # Parse CSV data line
            try:
                fields = line.split(",")
                if len(fields) < 7:
                    continue
                seq = int(fields[0])
                phase_ns = float(fields[1])
                gm_to_slave_ns = float(fields[2])
                slave_to_gm_ns = float(fields[3])
                scale_factor = float(fields[4])
                gm_first = int(fields[5])
                crystal_error = int(fields[6])
            except (ValueError, IndexError):
                # Not a valid CSV line
                print(f"# parse error: {line}", file=sys.stderr)
                continue

            crystal_ns = crystal_error
            if crystal_ns != 0:
                gps_locked = True

            # Track if this was a clamped/rejected measurement
            if phase_ns <= 50.0:
                rejected += 1

            # Phase stats
            stats.update(phase_ns)
            phase_buf.append(phase_ns)

            # TIE: phase(n) - phase(0)
            if first_phase is None:
                first_phase = phase_ns
            tie_val = phase_ns - first_phase
            tie_stats.update(tie_val)
            tie_buf.append(tie_val)
            mtie.update(tie_val)

            # Refresh dashboard at ~1 Hz
            now = time.monotonic()
            if now - last_render >= 1.0:
                # Recompute MTIE every 10s to save CPU
                if stats.n % 10 == 0:
                    mtie.compute()
                _refresh_dashboard(stats, phase_buf, tie_buf, tie_stats,
                                   mtie, gps_locked, crystal_ns, rejected,
                                   elapsed, run_dir, n_comments)
                last_render = now

    ser.close()
    print(f"\n\nCapture complete. {stats.n} measurements logged to {csv_path}")
    if stats.n > 0:
        print(f"Phase: mean={stats.mean/1000:+.1f} us  "
              f"sigma={stats.sigma/1000:.1f} us  "
              f"P-P={stats.pp/1000:.1f} us")


def _refresh_dashboard(stats, phase_buf, tie_buf, tie_stats, mtie,
                       gps_locked, crystal_ns, rejected, elapsed,
                       run_dir, n_comments):
    """Clear screen and render dashboard."""
    dashboard = render_dashboard(
        stats, phase_buf, tie_buf, tie_stats, mtie,
        gps_locked, crystal_ns, rejected, elapsed,
        run_dir, n_comments)
    # Move cursor to top-left and clear screen
    sys.stdout.write("\033[H\033[J")
    sys.stdout.write(dashboard)
    sys.stdout.write("\n")
    sys.stdout.flush()


def _parse_comment(line):
    """Parse # comment lines for useful metadata (placeholder for future use)."""
    pass


if __name__ == "__main__":
    main()
