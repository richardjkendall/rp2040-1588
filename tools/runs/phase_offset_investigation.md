# Phase Offset Investigation Plan

## Problem
RX 1PPS fires ~90 µs BEFORE GM 1PPS consistently across boots.
This means the RX clock (gps_now) is ~90 µs ahead of true GPS time.

## Current Performance (to preserve)
- Sigma: 3.5 µs (settled), drift: ~0 ns/s
- Architecture: PIO-domain filtered anchor, alpha=0.05, single-sample init
- Scope: 50 µs/div, 12.5 MSa/s waveform capture at ~1 Hz

## Hypothesis: TX HW Timestamp Overshoot

`prev_tx_timestamp = get_gps_time_ns() + tx_latency_ns`

The `tx_latency_ns` is measured from PIO counter before SPI send to the
TX-complete INT assertion. This includes time AFTER the frame leaves the wire:
- Frame transmission time on wire: ~70 bytes × 8 / 100 Mbps = ~5.6 µs
- W5500 post-TX processing + INT assertion latency: unknown
- Total post-wire component: unknown, could be significant

Meanwhile on RX side, `hw_counter_at_rx` is captured at RX INT assertion,
which is AFTER wire arrival by: frame RX time + W5500 processing + INT latency.

The asymmetry between TX post-wire delay (included in prev_tx_timestamp)
and RX post-wire delay (in hw_counter_at_rx) could explain the offset.

## Test Plan

### Test 1: Characterize GM TX Timing
**Goal:** Understand what tx_latency_ns actually measures.

- Read GM serial logs: `tx_latency_ns` is reported every 10 packets
- From test harness: "TX latency: mean=+252 µs std=1.5 µs"
- **Question:** How much of this 252 µs is pre-wire vs post-wire?
- **Method:** On the GM, capture the PIO counter at two points:
  1. Before SPI send (already done: `counter_before`)
  2. At TX INT assertion (already done: `latest_hw_counter`)
  - Also capture: after `w5500_send_frame()` returns (SPI complete)
  - This splits tx_latency into: SPI_time + W5500_TX_time_and_INT
  - The W5500_TX_time_and_INT portion is the post-wire component

### Test 2: Characterize RX INT Latency
**Goal:** Measure time from wire arrival to INT assertion on RX.

- **Method:** Use the scope to measure the delay between:
  - CH1: GM TX data pin (SPI MOSI or W5500 TX pin) — hard to probe
  - Alternative: Use the GM's 1PPS as reference, compute expected packet
    arrival time from TX latency + cable delay, compare to RX INT timing
- **Simpler method:** The W5500 datasheet should specify the INT assertion
  latency after frame reception. Look up the W5500 timing specs.

### Test 3: Measure Asymmetry via Scope
**Goal:** Directly measure the offset source.

- Add a GPIO toggle on GM at the moment `get_gps_time_ns()` is called (before send)
- Add a GPIO toggle on RX at the moment INT fires (hw_counter_at_rx capture)
- Measure the delay between these two GPIO pulses on the scope
- Compare to the reported tx_latency_ns
- The difference reveals the RX-side INT latency

### Test 4: Subtract GM Processing Mean
**Goal:** Quick software fix if the offset is from known delays.

The GM already computes `gm_local_processing_mean` (EMA of tx_latency)
and sends it in the PDU. The RX currently ignores this field.

- **Experiment:** On the RX, subtract `pdu.gm_local_processing_mean` from
  `prev_tx_timestamp` before computing `gps_now`. This would remove the
  GM's processing delay from the clock estimate.
- `gps_now = (pdu.prev_tx_timestamp - pdu.gm_local_processing_mean) + elapsed * sf`
- This changes prev_tx_timestamp from "GPS time at TX INT" to approximately
  "GPS time at send intent" (before SPI), which is closer to the actual
  GPS time when the packet started its journey.
- **Risk:** This might overcorrect. The true wire TX time is somewhere between
  get_gps_time_ns() and get_gps_time_ns() + tx_latency.

### Test 5: Split TX Latency on GM
**Goal:** Precisely determine when bits hit the wire.

- Modify GM to capture an additional PIO counter reading after
  `w5500_send_frame()` returns (SPI transfer complete)
- Report two values: SPI_time and post_SPI_time
- The actual wire TX starts after SPI transfer + small W5500 buffer delay
- Subtract (post_SPI_time + frame_wire_time/2) from tx_latency to get
  the offset correction

### Test 6: Empirical Calibration
**Goal:** If analytical methods are inconclusive, calibrate empirically.

- Add a configurable `phase_offset_ns` constant to the RX
- Set it to +90000 ns (to shift gps_now backward by 90 µs)
- Verify 1PPS alignment improves on scope
- Fine-tune across multiple boots
- This is the PTP "asymmetry correction" approach

## Recommended Order
1. **Test 4 first** — quickest to try, uses data already in the PDU
2. **Test 1** — read existing GM logs to understand tx_latency breakdown
3. **Test 5** — if Test 4 overcorrects, split the latency for precision
4. **Test 6** — fallback if analytical approaches don't converge

## Key Files
- GM TX timing: `ltsp/gm/main.c:89-181` (send_ltsp_pdu)
- RX clock init: `ltsp/receiver/main.c:290-310`
- RX anchor: `ltsp/receiver/main.c:312-330`
- PDU fields: `ltsp/common/ltsp_pdu.h:28` (prev_tx_timestamp, gm_local_processing_mean)
- Test scripts: `tools/scope_waveform_phase.py`, `tools/scope_waveform_detail.py`

## Current Architecture Summary (for context recovery)
- PIO-domain filtered anchor: gps_now filtered with alpha=0.05 IIR
- Prediction uses elapsed_pio_ns * scale_factor (PIO domain, no time_us_64)
- Single-sample clock init, filter handles convergence
- Core 0 schedules 1PPS from PIO counter + anchor, self-calibrating comp delay
- No re-anchoring of clock position (ltsp_clock_advance for CSV only)
- No PI servo, frequency from regression only
