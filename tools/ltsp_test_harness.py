#!/usr/bin/env python3
"""
LTSP Test Harness — Dual USB Serial Monitor

Reads GM and Receiver USB serial simultaneously, correlates by sequence
number, and provides live validation of the LTSP protocol chain.

Usage:
    python3 ltsp_test_harness.py /dev/tty.usbmodemXXXX /dev/tty.usbmodemYYYY

    First argument = GM port, second = Receiver port.
    Use `ls /dev/tty.usbmodem*` to find them.

Output:
    - Live dashboard to stderr (updated every 5 seconds)
    - Raw correlated data to stdout (pipe to file for later analysis)
"""

import sys
import serial
import threading
import time
import re
import math
from collections import deque
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Parsed data types
# ---------------------------------------------------------------------------

@dataclass
class GMSample:
    seq: int
    tx_ts_type: str          # "HW" or "SW"
    tx_latency_ns: int
    a1_ppb: float
    sigma_ns: float
    timestamp: float         # host wall clock

@dataclass
class RXSample:
    seq: int
    t_rx_ns: int
    t_tx_ns: int
    d_total_ns: int
    d_wire_ns: int
    d_gm_local_ns: int
    d_rx_local_ns: int
    offset_ns: int
    sigma_ns: float
    a0_ns: float
    a1_ppb: float
    interval_ticks: int
    timestamp: float         # host wall clock

@dataclass
class GMStats:
    locked: bool = False
    pps_count: int = 0
    crystal_err_ns: int = 0
    interp_err_ns: int = 0
    regression_valid: bool = False
    a0: float = 0.0
    a1_ppb: float = 0.0
    sigma_ns: float = 0.0

# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

# GM log patterns
GM_PDU_RE = re.compile(
    r'\[LTSP-GM\] PDU #(\d+) seq=(\d+) TX_TS=(HW|SW) '
    r'\(lat=([+-]?\d+) ns\) a1=([+-]?\d+\.\d+) ppb sigma=(\d+\.\d+) ns'
)
GM_GPS_RE = re.compile(
    r'\[Core 0\] GPS: Lock=(YES|NO) PPS=(\d+) crystal_err=([+-]?\d+) ns '
    r'interp_err=([+-]?\d+) ns'
)
GM_REG_RE = re.compile(
    r'\[LTSP-GM\] Regression: a0=([+-]?\d+\.?\d*) ticks '
    r'a1=([+-]?\d+\.\d+) ppb sigma=(\d+\.\d+) ns'
)

def parse_gm_line(line: str, now: float):
    """Parse a GM serial line. Returns GMSample, or updates stats, or None."""
    m = GM_PDU_RE.search(line)
    if m:
        return GMSample(
            seq=int(m.group(2)),
            tx_ts_type=m.group(3),
            tx_latency_ns=int(m.group(4)),
            a1_ppb=float(m.group(5)),
            sigma_ns=float(m.group(6)),
            timestamp=now,
        )
    return None

def parse_gm_stats(line: str, stats: GMStats):
    m = GM_GPS_RE.search(line)
    if m:
        stats.locked = m.group(1) == "YES"
        stats.pps_count = int(m.group(2))
        stats.crystal_err_ns = int(m.group(3))
        stats.interp_err_ns = int(m.group(4))
    m = GM_REG_RE.search(line)
    if m:
        stats.regression_valid = True
        stats.a0 = float(m.group(1))
        stats.a1_ppb = float(m.group(2))
        stats.sigma_ns = float(m.group(3))

def parse_rx_csv(line: str, now: float):
    """Parse a receiver CSV line. Returns RXSample or None."""
    line = line.strip()
    if not line or line.startswith('#'):
        return None
    parts = line.split(',')
    if len(parts) < 12:
        return None
    try:
        return RXSample(
            seq=int(parts[0]),
            t_rx_ns=int(parts[1]),
            t_tx_ns=int(parts[2]),
            d_total_ns=int(parts[3]),
            d_wire_ns=int(parts[4]),
            d_gm_local_ns=int(parts[5]),
            d_rx_local_ns=int(parts[6]),
            offset_ns=int(parts[7]),
            sigma_ns=float(parts[8]),
            a0_ns=float(parts[9]),
            a1_ppb=float(parts[10]),
            interval_ticks=int(parts[11]),
            timestamp=now,
        )
    except (ValueError, IndexError):
        return None

# ---------------------------------------------------------------------------
# Running statistics (Welford's online algorithm)
# ---------------------------------------------------------------------------

class RunningStats:
    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float('inf')
        self.max_val = float('-inf')

    def update(self, x):
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.m2 += delta * delta2
        self.min_val = min(self.min_val, x)
        self.max_val = max(self.max_val, x)

    @property
    def std(self):
        if self.n < 2:
            return 0.0
        return math.sqrt(self.m2 / (self.n - 1))

    def summary(self, unit=""):
        if self.n == 0:
            return "no data"
        return (f"mean={self.mean:+.0f}{unit} std={self.std:.0f}{unit} "
                f"min={self.min_val:+.0f}{unit} max={self.max_val:+.0f}{unit} "
                f"n={self.n}")

# ---------------------------------------------------------------------------
# Linear fit (for drift estimation)
# ---------------------------------------------------------------------------

class LinearFit:
    """Online linear regression: y = slope * x + intercept"""
    def __init__(self):
        self.n = 0
        self.sx = 0.0
        self.sy = 0.0
        self.sxx = 0.0
        self.sxy = 0.0

    def update(self, x, y):
        self.n += 1
        self.sx += x
        self.sy += y
        self.sxx += x * x
        self.sxy += x * y

    @property
    def slope(self):
        if self.n < 2:
            return 0.0
        denom = self.n * self.sxx - self.sx * self.sx
        if abs(denom) < 1e-30:
            return 0.0
        return (self.n * self.sxy - self.sx * self.sy) / denom

    @property
    def slope_ppb(self):
        """Slope as parts-per-billion (ns/ns = dimensionless, * 1e9 for ppb)"""
        return self.slope * 1e9

# ---------------------------------------------------------------------------
# Test harness state
# ---------------------------------------------------------------------------

class TestHarness:
    def __init__(self):
        self.lock = threading.Lock()

        # Received samples (ring buffers for recent history)
        self.gm_samples = deque(maxlen=1000)
        self.rx_samples = deque(maxlen=1000)

        # GM stats (latest)
        self.gm_stats = GMStats()

        # Correlation
        self.gm_seqs_seen = set()
        self.rx_seqs_seen = set()
        self.matched_count = 0

        # Statistics
        self.d_wire_stats = RunningStats()
        self.d_total_stats = RunningStats()
        self.d_gm_local_stats = RunningStats()
        self.d_rx_local_stats = RunningStats()
        self.offset_stats = RunningStats()
        self.tx_latency_stats = RunningStats()

        # Drift estimation: offset_ns vs host_time
        self.drift_fit = LinearFit()
        self.first_rx_time = None

        # Packet loss
        self.rx_seq_gaps = 0
        self.last_rx_seq = None

        # Error tracking
        self.gm_errors = 0
        self.rx_errors = 0

        # Timing
        self.start_time = time.time()

    def add_gm_sample(self, s: GMSample):
        with self.lock:
            self.gm_samples.append(s)
            self.gm_seqs_seen.add(s.seq)
            self.tx_latency_stats.update(s.tx_latency_ns)

    def add_rx_sample(self, s: RXSample):
        with self.lock:
            self.rx_samples.append(s)
            self.rx_seqs_seen.add(s.seq)

            # Sequence gap detection
            if self.last_rx_seq is not None:
                expected = (self.last_rx_seq + 1) & 0xFFFF
                if s.seq != expected:
                    self.rx_seq_gaps += 1
            self.last_rx_seq = s.seq

            # Update statistics
            self.d_wire_stats.update(s.d_wire_ns)
            self.d_total_stats.update(s.d_total_ns)
            self.d_gm_local_stats.update(s.d_gm_local_ns)
            self.d_rx_local_stats.update(s.d_rx_local_ns)
            self.offset_stats.update(s.offset_ns)

            # Drift fit: elapsed seconds vs offset_ns
            if self.first_rx_time is None:
                self.first_rx_time = s.timestamp
            elapsed = s.timestamp - self.first_rx_time
            self.drift_fit.update(elapsed, s.offset_ns)

            # Emit correlated CSV to stdout
            print(f"{s.seq},{s.timestamp - self.start_time:.3f},"
                  f"{s.t_rx_ns},{s.t_tx_ns},{s.d_total_ns},"
                  f"{s.d_wire_ns},{s.d_gm_local_ns},{s.d_rx_local_ns},"
                  f"{s.offset_ns},{s.sigma_ns},{s.a1_ppb}",
                  flush=True)

    def dashboard(self):
        """Return a multi-line dashboard string."""
        with self.lock:
            elapsed = time.time() - self.start_time
            lines = []
            lines.append(f"{'='*72}")
            lines.append(f" LTSP Test Harness  |  {elapsed:.0f}s elapsed")
            lines.append(f"{'='*72}")

            # GM status
            gs = self.gm_stats
            lines.append(f" GM  | GPS Lock: {'YES' if gs.locked else 'NO'}  "
                         f"PPS: {gs.pps_count}  "
                         f"Crystal: {gs.crystal_err_ns:+d} ns  "
                         f"Interp: {gs.interp_err_ns:+d} ns")
            if gs.regression_valid:
                lines.append(f"      | Regression: a1={gs.a1_ppb:+.3f} ppb  "
                             f"sigma={gs.sigma_ns:.1f} ns")
            lines.append(f"      | TX latency: {self.tx_latency_stats.summary(' ns')}")

            lines.append(f"{'-'*72}")

            # Receiver status
            lines.append(f" RX  | Packets: {self.offset_stats.n}  "
                         f"Seq gaps: {self.rx_seq_gaps}  "
                         f"Errors: GM={self.gm_errors} RX={self.rx_errors}")

            # Delay decomposition
            lines.append(f"      | D_total:    {self.d_total_stats.summary(' ns')}")
            lines.append(f"      | D_wire:     {self.d_wire_stats.summary(' ns')}")
            lines.append(f"      | D_gm_local: {self.d_gm_local_stats.summary(' ns')}")
            lines.append(f"      | D_rx_local: {self.d_rx_local_stats.summary(' ns')}")

            lines.append(f"{'-'*72}")

            # Clock offset + drift
            lines.append(f" CLK | Offset:     {self.offset_stats.summary(' ns')}")
            if self.drift_fit.n >= 10:
                drift = self.drift_fit.slope
                lines.append(f"      | Drift rate: {drift:+.1f} ns/s "
                             f"= {drift/1000:+.3f} us/s "
                             f"= {drift/1000:+.3f} ppm")
            else:
                lines.append(f"      | Drift rate: (need >= 10 samples)")

            # Verdicts
            lines.append(f"{'='*72}")
            verdicts = []
            if self.offset_stats.n >= 10:
                if self.d_wire_stats.std < 10000:  # < 10 us
                    verdicts.append("PASS: D_wire jitter < 10 us")
                else:
                    verdicts.append(f"WARN: D_wire jitter = {self.d_wire_stats.std:.0f} ns")

                if self.rx_seq_gaps == 0:
                    verdicts.append("PASS: Zero packet loss")
                else:
                    verdicts.append(f"WARN: {self.rx_seq_gaps} sequence gaps")

                if gs.regression_valid and gs.sigma_ns < 50:
                    verdicts.append(f"PASS: GM regression sigma = {gs.sigma_ns:.1f} ns")

                if self.drift_fit.n >= 30:
                    drift_ppm = abs(self.drift_fit.slope / 1000)
                    if drift_ppm < 100:  # < 100 ppm is normal for crystal
                        verdicts.append(f"PASS: Drift = {drift_ppm:.1f} ppm (normal crystal)")
                    else:
                        verdicts.append(f"WARN: Drift = {drift_ppm:.1f} ppm (high)")
            else:
                verdicts.append("... collecting samples ...")

            for v in verdicts:
                lines.append(f" {v}")
            lines.append(f"{'='*72}")

            return '\n'.join(lines)

# ---------------------------------------------------------------------------
# Serial reader threads
# ---------------------------------------------------------------------------

def read_gm(port: str, harness: TestHarness):
    """Read GM serial port in a thread."""
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        eprint(f"[GM] Connected to {port}")
    except serial.SerialException as e:
        eprint(f"[GM] FAILED to open {port}: {e}")
        return

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if not line:
                continue
            now = time.time()

            sample = parse_gm_line(line, now)
            if sample:
                harness.add_gm_sample(sample)

            parse_gm_stats(line, harness.gm_stats)

            if 'ERR:' in line:
                harness.gm_errors += 1

        except serial.SerialException:
            eprint("[GM] Serial connection lost, retrying...")
            time.sleep(2)
            try:
                ser = serial.Serial(port, 115200, timeout=1)
            except serial.SerialException:
                pass

def read_rx(port: str, harness: TestHarness):
    """Read Receiver serial port in a thread."""
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        eprint(f"[RX] Connected to {port}")
    except serial.SerialException as e:
        eprint(f"[RX] FAILED to open {port}: {e}")
        return

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if not line:
                continue
            now = time.time()

            # Stats/comment lines from receiver
            if line.startswith('#'):
                eprint(f"[RX] {line}")
                continue

            sample = parse_rx_csv(line, now)
            if sample:
                harness.add_rx_sample(sample)

            if 'ERR:' in line:
                harness.rx_errors += 1

        except serial.SerialException:
            eprint("[RX] Serial connection lost, retrying...")
            time.sleep(2)
            try:
                ser = serial.Serial(port, 115200, timeout=1)
            except serial.SerialException:
                pass

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def clear_screen():
    eprint('\033[2J\033[H', end='')

def main():
    if len(sys.argv) < 3:
        eprint("Usage: python3 ltsp_test_harness.py <GM_PORT> <RX_PORT>")
        eprint("")
        eprint("Example:")
        eprint("  python3 ltsp_test_harness.py /dev/tty.usbmodem1101 /dev/tty.usbmodem1201")
        eprint("")
        eprint("Find ports with: ls /dev/tty.usbmodem*")
        eprint("")
        eprint("Pipe stdout to a file for later analysis:")
        eprint("  python3 ltsp_test_harness.py GM_PORT RX_PORT > capture.csv")
        sys.exit(1)

    gm_port = sys.argv[1]
    rx_port = sys.argv[2]

    harness = TestHarness()

    # CSV header to stdout
    print("# seq,elapsed_s,t_rx_ns,t_tx_ns,d_total_ns,d_wire_ns,"
          "d_gm_local_ns,d_rx_local_ns,offset_ns,sigma_ns,a1_ppb",
          flush=True)

    # Start reader threads
    gm_thread = threading.Thread(target=read_gm, args=(gm_port, harness),
                                  daemon=True)
    rx_thread = threading.Thread(target=read_rx, args=(rx_port, harness),
                                  daemon=True)
    gm_thread.start()
    rx_thread.start()

    eprint("Waiting for data...\n")

    # Dashboard update loop
    try:
        while True:
            time.sleep(5)
            clear_screen()
            eprint(harness.dashboard())
    except KeyboardInterrupt:
        eprint("\n\nFinal results:")
        eprint(harness.dashboard())
        eprint("\nDone.")

if __name__ == '__main__':
    main()
