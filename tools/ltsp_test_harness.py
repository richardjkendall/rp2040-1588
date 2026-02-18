#!/usr/bin/env python3
"""
LTSP Test Harness v0.3 — Dual USB Serial Monitor

Reads GM and Receiver USB serial simultaneously, correlates by sequence
number, and provides live validation of the LTSP protocol chain.

Receiver v0.3 CSV format (11 columns):
    seq, d_total_ns, d_detrended_ns, offset_ns, drift_ns_per_s,
    drift_sigma_ns, gm_sigma_ns, gm_a1_ppb, 1pps_interval_ticks,
    clock_error_ns, sync_state

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
    d_total_ns: int
    d_detrended_ns: int
    offset_ns: int
    drift_ns_per_s: float
    drift_sigma_ns: float
    gm_sigma_ns: float
    gm_a1_ppb: float
    interval_ticks: int
    clock_error_ns: int
    sync_state: str
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
    """Parse a GM serial line. Returns GMSample or None."""
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
    """Parse a receiver v0.3 CSV line. Returns RXSample or None."""
    line = line.strip()
    if not line or line.startswith('#'):
        return None
    parts = line.split(',')
    if len(parts) < 11:
        return None
    try:
        return RXSample(
            seq=int(parts[0]),
            d_total_ns=int(parts[1]),
            d_detrended_ns=int(parts[2]),
            offset_ns=int(parts[3]),
            drift_ns_per_s=float(parts[4]),
            drift_sigma_ns=float(parts[5]),
            gm_sigma_ns=float(parts[6]),
            gm_a1_ppb=float(parts[7]),
            interval_ticks=int(parts[8]),
            clock_error_ns=int(parts[9]),
            sync_state=parts[10].strip(),
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

        # Statistics
        self.d_total_stats = RunningStats()
        self.d_detrended_stats = RunningStats()
        self.offset_stats = RunningStats()
        self.clock_error_stats = RunningStats()
        self.tx_latency_stats = RunningStats()

        # Latest receiver-reported values
        self.last_drift_ns_per_s = 0.0
        self.last_drift_sigma_ns = 0.0
        self.last_sync_state = "INIT"
        self.converged = False  # True once receiver reports non-zero d_detrended

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
            self.tx_latency_stats.update(s.tx_latency_ns)

    def add_rx_sample(self, s: RXSample):
        with self.lock:
            self.rx_samples.append(s)

            # Sequence gap detection
            if self.last_rx_seq is not None:
                expected = (self.last_rx_seq + 1) & 0xFFFF
                if s.seq != expected:
                    self.rx_seq_gaps += 1
            self.last_rx_seq = s.seq

            # Track convergence: non-zero d_detrended means regression is full
            if s.d_detrended_ns != 0 or s.offset_ns != 0:
                self.converged = True

            # Update statistics (only after convergence)
            if self.converged:
                self.d_detrended_stats.update(s.d_detrended_ns)
                self.offset_stats.update(s.offset_ns)

            # Clock error stats (only when clock is disciplined)
            if s.sync_state in ("ACQUIRING", "LOCKED") and s.clock_error_ns != 0:
                self.clock_error_stats.update(s.clock_error_ns)

            self.d_total_stats.update(s.d_total_ns)
            self.last_drift_ns_per_s = s.drift_ns_per_s
            self.last_drift_sigma_ns = s.drift_sigma_ns
            self.last_sync_state = s.sync_state

            # Emit CSV to stdout
            print(f"{s.seq},{s.timestamp - self.start_time:.3f},"
                  f"{s.d_total_ns},{s.d_detrended_ns},{s.offset_ns},"
                  f"{s.drift_ns_per_s:.1f},{s.drift_sigma_ns:.1f},"
                  f"{s.gm_sigma_ns:.1f},{s.gm_a1_ppb:.3f},"
                  f"{s.clock_error_ns},{s.sync_state}",
                  flush=True)

    def dashboard(self):
        """Return a multi-line dashboard string."""
        with self.lock:
            elapsed = time.time() - self.start_time
            lines = []
            lines.append(f"{'='*72}")
            lines.append(f" LTSP Test Harness v0.3  |  {elapsed:.0f}s elapsed")
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
            total_rx = self.d_total_stats.n
            lines.append(f" RX  | Packets: {total_rx}  "
                         f"Seq gaps: {self.rx_seq_gaps}  "
                         f"Errors: GM={self.gm_errors} RX={self.rx_errors}  "
                         f"State: {self.last_sync_state}")

            # Drift characterisation (from receiver's regression)
            drift_ppm = self.last_drift_ns_per_s / 1000.0
            lines.append(f"      | Drift: {self.last_drift_ns_per_s:+.1f} ns/s "
                         f"= {drift_ppm:+.1f} ppm  "
                         f"sigma={self.last_drift_sigma_ns:.0f} ns")

            lines.append(f"{'-'*72}")

            # De-trended jitter (only meaningful after convergence)
            if self.converged:
                lines.append(f" JIT | Detrended: {self.d_detrended_stats.summary(' ns')}")
                lines.append(f"      | Offset:    {self.offset_stats.summary(' ns')}")
            else:
                lines.append(f" JIT | Waiting for drift regression to converge "
                             f"(~60 samples)...")

            lines.append(f"{'-'*72}")

            # Clock discipline
            if self.clock_error_stats.n > 0:
                lines.append(f" CLK | Error:  {self.clock_error_stats.summary(' ns')}")
                clk_err_us = self.clock_error_stats.std / 1000.0
                lines.append(f"      | Sigma:  {clk_err_us:.1f} us  "
                             f"Mean: {self.clock_error_stats.mean/1000.0:+.1f} us")
            else:
                lines.append(f" CLK | Clock not yet disciplined")

            # Verdicts
            lines.append(f"{'='*72}")
            verdicts = []
            if self.converged and self.d_detrended_stats.n >= 10:
                jitter_us = self.d_detrended_stats.std / 1000.0
                if jitter_us < 100:
                    verdicts.append(f"PASS: Jitter sigma = {jitter_us:.1f} us")
                else:
                    verdicts.append(f"WARN: Jitter sigma = {jitter_us:.1f} us (high)")

                if self.rx_seq_gaps == 0:
                    verdicts.append("PASS: Zero packet loss")
                else:
                    verdicts.append(f"WARN: {self.rx_seq_gaps} sequence gaps")

                if gs.regression_valid and gs.sigma_ns < 50:
                    verdicts.append(f"PASS: GM regression sigma = {gs.sigma_ns:.1f} ns")

                drift_ppm = abs(self.last_drift_ns_per_s / 1000.0)
                if drift_ppm < 100:
                    verdicts.append(f"PASS: RX crystal = {drift_ppm:.1f} ppm")
                else:
                    verdicts.append(f"WARN: RX crystal = {drift_ppm:.1f} ppm (high)")

                sigma_ns = self.last_drift_sigma_ns
                if sigma_ns < 50000:
                    verdicts.append(f"PASS: Drift sigma = {sigma_ns:.0f} ns")
                else:
                    verdicts.append(f"WARN: Drift sigma = {sigma_ns:.0f} ns (high)")

                # Clock error verdict
                if self.clock_error_stats.n >= 10:
                    clk_sigma_us = self.clock_error_stats.std / 1000.0
                    clk_mean_us = abs(self.clock_error_stats.mean / 1000.0)
                    if clk_sigma_us < 100 and clk_mean_us < 100:
                        verdicts.append(f"PASS: Clock error sigma = {clk_sigma_us:.1f} us, "
                                       f"mean = {self.clock_error_stats.mean/1000.0:+.1f} us")
                    else:
                        verdicts.append(f"WARN: Clock error sigma = {clk_sigma_us:.1f} us, "
                                       f"mean = {self.clock_error_stats.mean/1000.0:+.1f} us")

                if self.last_sync_state == "LOCKED":
                    verdicts.append("PASS: Sync state = LOCKED")
                elif self.last_sync_state == "ACQUIRING":
                    verdicts.append(f"INFO: Sync state = ACQUIRING")
                else:
                    verdicts.append(f"WARN: Sync state = {self.last_sync_state}")
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
    print("# seq,elapsed_s,d_total_ns,d_detrended_ns,offset_ns,"
          "drift_ns_per_s,drift_sigma_ns,gm_sigma_ns,gm_a1_ppb,"
          "clock_error_ns,sync_state",
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
