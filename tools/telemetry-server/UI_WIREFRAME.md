# PTP Phase Offset Telemetry - UI Redesign Wireframe

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ PTP Phase Offset Telemetry                                    ● Connected       │
│ GPS: -444 ns (-0.000 ppm, SF: 0.999999556) │ Batches: 4 │ Dropped: 0           │
└─────────────────────────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────────────────────────┐
│ Phase Offset Statistics                                                           │
├──────────────────┬─────────────┬─────────────┬─────────────┬─────────────────────┤
│                  │  All-Time   │   5 Min     │   15 Min    │   30 Min            │
├──────────────────┼─────────────┼─────────────┼─────────────┼─────────────────────┤
│ Samples          │     240     │     30      │     90      │    180              │
│ Mean (ns)        │  125424.3   │  125450.2   │  125438.7   │  125430.1           │
│ Std Dev (ns)     │   10483.4   │    8234.1   │    9102.5   │    9801.3           │
│ Min (ns)         │  105300.0   │  112400.0   │  110200.0   │  108500.0           │
│ Max (ns)         │  148824.0   │  138200.0   │  142100.0   │  145300.0           │
│ Range (ns)       │   43524.0   │   25800.0   │   31900.0   │   36800.0           │
│ GM First         │       0     │       0     │       0     │       0             │
│ Slave First      │     240     │      30     │      90     │     180             │
└──────────────────┴─────────────┴─────────────┴─────────────┴─────────────────────┘

┌─────────────────────────────────┬─────────────────────────────────────────────────┐
│ Time Interval Error (TIE)       │ Phase Offset Histogram                          │
│                                 │                                                 │
│   160 ┤                         │   60 ┤                                         │
│       │     ╭─╮                 │      │         ╭─╮                             │
│   140 ┤    ╭╯ ╰╮  ╭╮            │   40 ┤       ╭─╯ ╰─╮                           │
│       │   ╭╯   ╰──╯╰╮           │      │     ╭─╯     ╰─╮                         │
│   120 ┤ ╭─╯         ╰─╮         │   20 ┤   ╭─╯         ╰─╮                       │
│       │╭╯             ╰─╮       │      │ ╭─╯             ╰─╮                     │
│   100 ┼─────────────────────────│    0 ┼─────────────────────────────────────────│
│  (µs) │         Time (min)      │      │  100k   120k   140k   160k  (ns)        │
│       0     5    10    15    20 │                                                 │
└─────────────────────────────────┴─────────────────────────────────────────────────┘

┌─────────────────────────────────┬─────────────────────────────────────────────────┐
│ Max Time Interval Error (MTIE)  │ Allan Deviation                                 │
│                                 │                                                 │
│   25 ┤                         │  100 ┤                                          │
│      │                  ╭──    │      │╲                                         │
│   20 ┤              ╭───╯      │   10 ┤ ╲                                        │
│      │          ╭───╯          │      │  ╲╲                                      │
│   15 ┤      ╭───╯              │    1 ┤   ╲─╲___                                │
│      │  ╭───╯                  │      │       ╲___╲____                          │
│   10 ┼──────────────────────── │  0.1 ┼────────────────────────────────────────  │
│  (µs)│  Observation (s)        │ (µs) │      τ (seconds)                         │
│      1    10   100  1000       │      1    10   100  1000  10k                   │
└─────────────────────────────────┴─────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────────────────────────┐
│ Recent Measurements (Last 60s)                                                    │
│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│   #240  125424.3 ns  (S first)  │  #239  125401.2 ns  │  #238  125438.9 ns     │
│   #237  125455.1 ns  (S first)  │  #236  125420.3 ns  │  #235  125447.8 ns     │
└───────────────────────────────────────────────────────────────────────────────────┘
```

## Key Design Features

### 1. **Compact Header Bar**
- Connection status with visual indicator
- GPS crystal error, PPM, scale factor (critical reference info)
- Batch count and dropped packet count
- Always visible, doesn't take up vertical space

### 2. **Statistics Table** (Replaces 4 separate panels)
- Single table with time windows as columns
- All metrics in rows for easy comparison across windows
- ~50% less vertical space than current design
- Easier to spot trends (mean increasing over time, etc.)

### 3. **Chart Grid** (2×2 layout)
- **TIE (Time Interval Error)**: Shows phase offset over time
  - Essential for seeing drift, jumps, oscillations
  - X-axis: time, Y-axis: phase offset

- **MTIE (Maximum Time Interval Error)**: Worst-case deviation
  - Shows maximum phase excursion over different observation windows
  - ITU-T G.810/G.8261 compliance metric
  - X-axis: observation interval (log scale), Y-axis: max deviation

- **Histogram**: Distribution of measurements (already have this)
  - Quick view of measurement clustering
  - Spot multimodal distributions or outliers

- **Allan Deviation**: Clock stability metric
  - Gold standard for oscillator characterization
  - Shows stability at different averaging times
  - X-axis: averaging time τ (log scale), Y-axis: Allan dev (log scale)

### 4. **Recent Measurements Scroll**
- Quick debug view of last minute of data
- Can see individual measurements without scrolling
- Useful for spotting sudden changes

## Responsive Behavior

- On narrow screens, stack charts vertically
- Statistics table remains horizontal (scroll if needed)
- Header compresses to multi-line if needed

## Benefits

1. **More data visible**: 4 charts + statistics vs current 3 panels
2. **Less scrolling**: Compact table design
3. **Professional**: Looks like telecom test equipment UI
4. **Standards-compliant**: MTIE/Allan dev are industry standard metrics
5. **Better analysis**: Multiple complementary views of same data
