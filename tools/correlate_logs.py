#!/usr/bin/env python3
"""
Correlate slave UART logs with measurement device CSV data.

Aligns slave discipline events with phase offset measurements by timestamp.
"""

import csv
import sys
import re
from dataclasses import dataclass
from typing import List, Optional
import argparse


@dataclass
class SlaveEvent:
    """Slave discipline event from UART log"""
    timestamp_us: int
    seq: int
    offset_ns: int
    correction_ns: int
    kalman_offset: float
    safety: bool
    hw_rx: bool
    hw_tx: bool
    scale: float

    @classmethod
    def parse_line(cls, line: str) -> Optional['SlaveEvent']:
        """Parse DISC| log line"""
        if not line.startswith('DISC|'):
            return None

        parts = line.strip().split('|')
        if len(parts) != 10:
            return None

        try:
            return cls(
                timestamp_us=int(parts[1]),
                seq=int(parts[2]),
                offset_ns=int(parts[3]),
                correction_ns=int(parts[4]),
                kalman_offset=float(parts[5]),
                safety=bool(int(parts[6])),
                hw_rx=bool(int(parts[7])),
                hw_tx=bool(int(parts[8])),
                scale=float(parts[9])
            )
        except (ValueError, IndexError):
            return None


@dataclass
class MeasurementEvent:
    """Phase measurement from CSV"""
    seq: int
    timestamp_us: int
    phase_ns: float
    gm_to_slave_ns: float
    slave_to_gm_ns: float
    scale_factor: float
    gm_first: bool
    crystal_error_ns: float
    received_at: str


def parse_slave_log(filepath: str) -> List[SlaveEvent]:
    """Parse slave UART log file"""
    events = []
    with open(filepath, 'r') as f:
        for line in f:
            event = SlaveEvent.parse_line(line)
            if event:
                events.append(event)
    return events


def parse_measurement_csv(filepath: str) -> List[MeasurementEvent]:
    """Parse measurement CSV file"""
    events = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                events.append(MeasurementEvent(
                    seq=int(row['seq']),
                    timestamp_us=int(row['timestamp_us']),
                    phase_ns=float(row['phase_ns']),
                    gm_to_slave_ns=float(row['gm_to_slave_ns']),
                    slave_to_gm_ns=float(row['slave_to_gm_ns']),
                    scale_factor=float(row['scale_factor']),
                    gm_first=row['gm_first'] == 'true',
                    crystal_error_ns=float(row['crystal_error_ns']),
                    received_at=row['received_at']
                ))
            except (KeyError, ValueError):
                continue
    return events


def correlate(slave_events: List[SlaveEvent],
              measurement_events: List[MeasurementEvent],
              tolerance_us: int = 2000000) -> List[tuple]:
    """
    Correlate slave and measurement events by timestamp.

    Args:
        slave_events: List of slave discipline events
        measurement_events: List of phase measurements
        tolerance_us: Maximum time difference for correlation (default 2s)

    Returns:
        List of (slave_event, measurement_event) tuples
    """
    correlated = []

    # For each measurement, find closest slave event within tolerance
    for meas in measurement_events:
        best_slave = None
        best_delta = float('inf')

        for slave in slave_events:
            delta = abs(slave.timestamp_us - meas.timestamp_us)
            if delta < tolerance_us and delta < best_delta:
                best_delta = delta
                best_slave = slave

        if best_slave:
            correlated.append((best_slave, meas, best_delta))

    return correlated


def write_correlated_csv(correlated: List[tuple], output_file: str):
    """Write correlated data to CSV"""
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)

        # Header
        writer.writerow([
            'meas_seq', 'meas_timestamp_us', 'phase_ns', 'phase_us',
            'slave_seq', 'slave_timestamp_us', 'time_delta_us',
            'slave_offset_ns', 'slave_correction_ns', 'slave_kalman_offset_ns',
            'safety_active', 'hw_rx', 'hw_tx', 'slave_scale', 'meas_scale',
            'offset_vs_phase_delta_ns', 'gm_to_slave_ns', 'slave_to_gm_ns',
            'crystal_error_ns'
        ])

        # Data
        for slave, meas, delta in correlated:
            writer.writerow([
                meas.seq,
                meas.timestamp_us,
                meas.phase_ns,
                meas.phase_ns / 1000,  # Convert to µs
                slave.seq,
                slave.timestamp_us,
                delta,
                slave.offset_ns,
                slave.correction_ns,
                slave.kalman_offset,
                1 if slave.safety else 0,
                1 if slave.hw_rx else 0,
                1 if slave.hw_tx else 0,
                slave.scale,
                meas.scale_factor,
                slave.offset_ns - meas.phase_ns,  # How much do they differ?
                meas.gm_to_slave_ns,
                meas.slave_to_gm_ns,
                meas.crystal_error_ns
            ])


def main():
    parser = argparse.ArgumentParser(
        description='Correlate slave UART logs with measurement CSV data'
    )
    parser.add_argument('slave_log', help='Slave UART log file')
    parser.add_argument('measurement_csv', help='Measurement CSV file')
    parser.add_argument('-o', '--output', default='correlated.csv',
                       help='Output CSV file (default: correlated.csv)')
    parser.add_argument('-t', '--tolerance', type=int, default=2000000,
                       help='Time correlation tolerance in µs (default: 2000000 = 2s)')

    args = parser.parse_args()

    print(f"Loading slave log: {args.slave_log}")
    slave_events = parse_slave_log(args.slave_log)
    print(f"  Found {len(slave_events)} discipline events")

    print(f"Loading measurement CSV: {args.measurement_csv}")
    measurement_events = parse_measurement_csv(args.measurement_csv)
    print(f"  Found {len(measurement_events)} measurements")

    print(f"Correlating with tolerance ±{args.tolerance/1000000:.1f}s...")
    correlated = correlate(slave_events, measurement_events, args.tolerance)
    print(f"  Matched {len(correlated)} events")

    print(f"Writing correlated data to: {args.output}")
    write_correlated_csv(correlated, args.output)

    # Summary statistics
    if correlated:
        deltas = [delta for _, _, delta in correlated]
        avg_delta = sum(deltas) / len(deltas)
        max_delta = max(deltas)
        print(f"\nCorrelation quality:")
        print(f"  Average time delta: {avg_delta/1000:.1f}ms")
        print(f"  Max time delta: {max_delta/1000:.1f}ms")

        # Check offset vs phase correlation
        diffs = []
        for slave, meas, _ in correlated:
            diff = abs(slave.offset_ns - meas.phase_ns)
            diffs.append(diff)

        avg_diff = sum(diffs) / len(diffs)
        max_diff = max(diffs)
        print(f"\nOffset vs Phase comparison:")
        print(f"  Average difference: {avg_diff/1000:.1f}µs")
        print(f"  Max difference: {max_diff/1000:.1f}µs")

        # Safety margin activation
        safety_count = sum(1 for slave, _, _ in correlated if slave.safety)
        print(f"\nSafety margin active: {safety_count}/{len(correlated)} ({100*safety_count/len(correlated):.1f}%)")

        # SW timestamp events
        sw_count = sum(1 for slave, _, _ in correlated if not (slave.hw_rx and slave.hw_tx))
        print(f"SW timestamp events: {sw_count}/{len(correlated)} ({100*sw_count/len(correlated):.1f}%)")

    print(f"\nDone! Correlated data saved to {args.output}")


if __name__ == '__main__':
    main()
