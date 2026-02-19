#!/usr/bin/env python3
"""Detailed waveform analysis of scope CH1 (GM) and CH2 (RX) 1PPS signals."""

import numpy as np
import vxi11
import re
import sys

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


def analyze():
    instr = vxi11.Instrument(SCOPE_IP)
    instr.open()

    print(f'Scope: {instr.ask("*IDN?")}')
    tdiv_raw = instr.ask('TDIV?')
    sara_raw = instr.ask('SARA?')
    print(f'TDIV: {tdiv_raw}')
    print(f'SARA: {sara_raw}')

    sara = parse_sara(sara_raw)
    sample_period_ns = 1e9 / sara
    print(f'Sample period: {sample_period_ns:.1f} ns')

    instr.write('WFSU SP,0,NP,0,FP,0,SN,0')
    ch1 = parse_waveform(instr.ask_raw(b'C1:WF? DAT2\n'))
    ch2 = parse_waveform(instr.ask_raw(b'C2:WF? DAT2\n'))
    instr.close()

    print(f'\nCH1 (GM): {len(ch1)} samples, min={ch1.min():.0f}, max={ch1.max():.0f}')
    print(f'CH2 (RX): {len(ch2)} samples, min={ch2.min():.0f}, max={ch2.max():.0f}')

    e1 = find_rising_edge(ch1)
    e2 = find_rising_edge(ch2)

    if e1 is not None:
        i1 = int(e1)
        print(f'\nCH1 rising edge at sample {e1:.1f} '
              f'({e1*sample_period_ns/1000:.1f} us from start)')
        print(f'  Samples around edge: '
              f'{ch1[max(0,i1-3):i1+4].astype(int).tolist()}')
    else:
        print('\nCH1: no rising edge found')

    if e2 is not None:
        i2 = int(e2)
        print(f'CH2 rising edge at sample {e2:.1f} '
              f'({e2*sample_period_ns/1000:.1f} us from start)')
        print(f'  Samples around edge: '
              f'{ch2[max(0,i2-3):i2+4].astype(int).tolist()}')
    else:
        print('CH2: no rising edge found')

    if e1 is not None and e2 is not None:
        phase_ns = (e2 - e1) * sample_period_ns
        print(f'\nPhase: {phase_ns:+.0f} ns = {phase_ns/1000:+.1f} us')
        print(f'  (negative = RX fires before GM)')

    # Pulse widths
    print(f'\nPulse analysis:')
    for name, wf, edge in [('CH1/GM', ch1, e1), ('CH2/RX', ch2, e2)]:
        if edge is None:
            continue
        lo, hi = np.min(wf), np.max(wf)
        thresh = lo + 0.5 * (hi - lo)
        i_start = int(edge)
        above = wf[i_start:] >= thresh
        if np.any(~above):
            fall = i_start + np.argmin(above)
            width_ns = (fall - edge) * sample_period_ns
            print(f'  {name}: pulse width = {width_ns:.0f} ns '
                  f'({width_ns/1000:.1f} us)')

    # Position in capture window
    window_ns = len(ch1) * sample_period_ns
    print(f'\nCapture window: {window_ns/1000:.1f} us')
    if e1 is not None:
        print(f'  CH1 edge at {e1*sample_period_ns/window_ns*100:.1f}% of window')
    if e2 is not None:
        print(f'  CH2 edge at {e2*sample_period_ns/window_ns*100:.1f}% of window')


if __name__ == '__main__':
    analyze()
