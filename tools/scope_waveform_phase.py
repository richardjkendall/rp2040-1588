#!/usr/bin/env python3
"""Measure 1PPS phase offset between CH1 and CH2 using scope waveform data."""

import re
import sys
import time

import numpy as np
import vxi11

SCOPE_IP = '10.10.255.228'


def parse_waveform(raw):
    """Parse Siglent binary waveform block: #<ndigits><nbytes><data>"""
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


def measure_once(instr):
    """Take one phase measurement. Returns phase_ns or None."""
    sara = parse_sara(instr.ask('SARA?'))
    sample_period_ns = 1e9 / sara

    instr.write('WFSU SP,0,NP,0,FP,0,SN,0')
    ch1 = parse_waveform(instr.ask_raw(b'C1:WF? DAT2\n'))
    ch2 = parse_waveform(instr.ask_raw(b'C2:WF? DAT2\n'))

    e1 = find_rising_edge(ch1)
    e2 = find_rising_edge(ch2)

    if e1 is None or e2 is None:
        return None

    return (e2 - e1) * sample_period_ns


def measure_phase():
    """Single detailed measurement with diagnostics."""
    instr = vxi11.Instrument(SCOPE_IP)
    instr.open()

    try:
        idn = instr.ask('*IDN?')
        print(f"Scope: {idn}")

        tdiv_raw = instr.ask('TDIV?')
        tdiv_s = float(tdiv_raw.split()[-1].rstrip('Ss'))
        print(f"Time/div: {tdiv_s*1e6:.1f} us")

        sara = parse_sara(instr.ask('SARA?'))
        sample_period_ns = 1e9 / sara
        print(f"Sample rate: {sara/1e6:.1f} MSa/s ({sample_period_ns:.1f} ns/sample)")

        instr.write('WFSU SP,0,NP,0,FP,0,SN,0')

        ch1 = parse_waveform(instr.ask_raw(b'C1:WF? DAT2\n'))
        ch2 = parse_waveform(instr.ask_raw(b'C2:WF? DAT2\n'))

        print(f"CH1: {len(ch1)} samples, range [{ch1.min():.0f}, {ch1.max():.0f}]")
        print(f"CH2: {len(ch2)} samples, range [{ch2.min():.0f}, {ch2.max():.0f}]")

        e1 = find_rising_edge(ch1)
        e2 = find_rising_edge(ch2)

        if e1 is None:
            print("No rising edge found on CH1!")
            return None
        if e2 is None:
            print("No rising edge found on CH2!")
            return None

        lag_samples = e2 - e1
        phase_ns = lag_samples * sample_period_ns

        print(f"\nCH1 edge at sample {e1:.1f}, CH2 edge at sample {e2:.1f}")
        print(f"Phase: {phase_ns:+.0f} ns = {phase_ns/1000:+.1f} us")

        return phase_ns

    finally:
        instr.close()


if __name__ == '__main__':
    if '--loop' not in sys.argv:
        measure_phase()
    else:
        interval = 1.0
        phases = []
        try:
            while True:
                instr = vxi11.Instrument(SCOPE_IP)
                instr.open()
                try:
                    phase_ns = measure_once(instr)
                finally:
                    instr.close()

                if phase_ns is not None:
                    phases.append(phase_ns)
                    arr = np.array(phases)
                    print(f"Phase: {phase_ns:+8.0f} ns  "
                          f"mean: {arr.mean():+8.0f} ns  "
                          f"sigma: {arr.std():6.0f} ns  "
                          f"n={len(phases)}")
                else:
                    print("No edge detected")

                time.sleep(interval)
        except KeyboardInterrupt:
            if phases:
                arr = np.array(phases)
                print(f"\n--- Summary: mean={arr.mean():+.0f} ns  "
                      f"sigma={arr.std():.0f} ns  "
                      f"min={arr.min():+.0f} ns  max={arr.max():+.0f} ns  "
                      f"n={len(phases)} ---")
