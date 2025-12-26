# Log Correlation Guide

This guide explains how to correlate slave discipline logs with measurement device data for debugging and analysis.

## Overview

The correlation system allows you to align:
- **Slave UART logs** (what the slave thinks it's doing)
- **Measurement CSV data** (what we observe the slave is actually doing)

This helps answer questions like:
- "When phase spiked to 260µs, what did slave think the offset was?"
- "When safety margin activated, did phase measurements improve?"
- "Do SW timestamp events correlate with measurement spikes?"

## Quick Start

### 1. Collect Data

**Slave UART log:**
```bash
# Connect to slave via UART and capture output
screen /dev/tty.usbserial-* 115200 | tee slave_log.txt
# Or use minicom, cu, etc.
```

The slave will output lines like:
```
DISC|1234567890|42|+105234|-21047|+105234.0|0|1|1|0.999998432
```

**Measurement CSV:**
Already being collected by telemetry server to `tests/testN.csv`

### 2. Run Correlation Script

```bash
cd /Users/rjk/Code/pico-gps-1588/tools

./correlate_logs.py \
    ~/slave_uart.log \
    ../tests/test2_long.csv \
    -o correlated_analysis.csv
```

### 3. Analyze Results

Open `correlated_analysis.csv` in your favorite tool. Key columns:

- `phase_ns`: What measurement device observed
- `slave_offset_ns`: What slave calculated
- `slave_correction_ns`: How much slave corrected
- `safety_active`: Whether safety margin was engaged (1=yes, 0=no)
- `hw_rx`, `hw_tx`: Whether HW timestamps were used
- `offset_vs_phase_delta_ns`: Difference between slave's view and reality

## Log Format Reference

### Slave DISC Log Format

```
DISC|timestamp_us|seq|offset_ns|correction_ns|kalman_offset|safety|hw_rx|hw_tx|scale
```

Fields:
- `timestamp_us`: Monotonic microseconds since slave boot
- `seq`: Discipline sequence number (increments each PTP cycle)
- `offset_ns`: Calculated offset from master (IEEE 1588 equation)
- `correction_ns`: Clock correction applied this cycle
- `kalman_offset`: Kalman-filtered offset estimate
- `safety`: Safety margin active (1=yes, 0=no)
- `hw_rx`: RX hardware timestamp valid (1=yes, 0=no)
- `hw_tx`: TX hardware timestamp valid (1=yes, 0=no)
- `scale`: Frequency scale factor

Example:
```
DISC|5234567890|142|+98234|-19647|+98450.2|0|1|1|0.999998567
```
- At 5234.5 seconds since boot
- Discipline cycle #142
- Offset: +98.2µs
- Applied -19.6µs correction
- Safety margin: not active
- HW timestamps: both valid
- Scale: 1.567ppm slow

### Measurement CSV Format

Already documented - see existing CSV files.

## Correlation Algorithm

The script:
1. Parses both logs
2. For each measurement, finds closest slave discipline event
3. Matches within tolerance window (default: ±2 seconds)
4. Outputs merged data with time delta

**Note:** Timestamps are from different clocks:
- Slave uses its own (PTP-disciplined) clock
- Measurements use host/network time
- Correlation by timestamp has ~1-2s jitter but sequence-based alignment works well

## Usage Examples

### Basic Correlation

```bash
./correlate_logs.py slave.log measurement.csv
```

### Custom Output File

```bash
./correlate_logs.py slave.log measurement.csv -o my_analysis.csv
```

### Tighter Correlation Window

```bash
# Only match events within ±500ms
./correlate_logs.py slave.log measurement.csv -t 500000
```

### Help

```bash
./correlate_logs.py --help
```

## Analysis Tips

### Finding Safety Margin Events

```bash
# In the correlated CSV, filter for safety_active=1
awk -F',' '$11==1' correlated.csv | head
```

### Finding SW Timestamp Events

```bash
# Filter for rows where hw_rx=0 or hw_tx=0
awk -F',' '$12==0 || $13==0' correlated.csv
```

### Checking Offset vs Phase Correlation

```python
import pandas as pd
df = pd.read_csv('correlated.csv')

# How well do slave's offset and measured phase correlate?
correlation = df['slave_offset_ns'].corr(df['phase_ns'])
print(f"Correlation: {correlation:.3f}")

# When do they diverge significantly?
df['delta_us'] = (df['slave_offset_ns'] - df['phase_ns']) / 1000
print(df[df['delta_us'].abs() > 50])  # More than 50µs difference
```

### Finding Problem Patterns

```python
import pandas as pd
df = pd.read_csv('correlated.csv')

# When phase spiked, was safety margin active?
spikes = df[df['phase_us'] > 200]
print(f"Safety active during spikes: {spikes['safety_active'].mean():.1%}")

# Were SW timestamps involved?
print(f"SW timestamps during spikes: {(~(spikes['hw_rx'] & spikes['hw_tx'])).mean():.1%}")
```

## Troubleshooting

### No matches found

- Check that slave log contains `DISC|` lines
- Verify timestamps are reasonable (not zero, not wildly different)
- Try increasing tolerance: `-t 10000000` (10 seconds)

### Poor correlation quality

- If average time delta > 2s, clocks may have drifted significantly
- Consider using sequence numbers for alignment instead
- Slave's clock is being disciplined, so timestamp drift is expected

### Script errors

- Ensure both files are readable
- Check CSV has proper header row
- Verify slave log is UTF-8 text

## Next Steps

After correlating logs, you can:

1. **Visualize** with Python/matplotlib:
   ```python
   import pandas as pd
   import matplotlib.pyplot as plt

   df = pd.read_csv('correlated.csv')

   fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

   # Phase offset over time with safety margin overlay
   ax1.plot(df['meas_seq'], df['phase_us'], label='Phase')
   ax1.scatter(df[df['safety_active']==1]['meas_seq'],
               df[df['safety_active']==1]['phase_us'],
               color='red', label='Safety Active', marker='x')
   ax1.set_ylabel('Phase Offset (µs)')
   ax1.legend()

   # Correction amounts
   ax2.plot(df['slave_seq'], df['slave_correction_ns']/1000)
   ax2.set_ylabel('Correction (µs)')
   ax2.set_xlabel('Sequence')

   plt.tight_layout()
   plt.savefig('correlation_analysis.png')
   ```

2. **Export statistics** for reports

3. **Identify patterns** to guide improvements

4. **Verify fixes** work as expected
