# LTSP Receiver — Session Status

**Date:** 2026-02-28
**Branch:** `ltsp`
**Firmware on devices:** v1.0 with 5-minute holdover (Test 3d build)

---

## What Was Done This Session

### Three holdover bugs fixed (v1.0)

All changes in `ltsp/receiver/main.c` and `ltsp/receiver/ltsp_clock.h`.

**Fix 1 — Holdover timer on Core 0:**
Core 0 now calls `ltsp_clock_update_state(&clock_state, false, false, ...)` every 1ms
to enable LOCKED→HOLDOVER transition when no packets arrive. Guarded to skip INIT
state (INIT's `!has_packet` handler resets `valid_packet_count`, preventing Core 1
from reaching the 3-packet threshold for INIT→ACQUIRING).

**Fix 2 — Pipeline reset after sequence gap/restart:**
On gap/restart detection, the entire measurement pipeline is now cleared:
- `ltsp_regression_init()` — clears 60-sample regression window
- `ltsp_min_filter_reset()` — clears min filter
- `drift_sample_count = 0` — restarts sample counting
- `d_total_ref_set = false` — resets reference
- `clock_initialized = false` — forces clock re-init from fresh data

**Fix 3 — Holdover coasting with re-anchor:**
During outage, PPS keeps running on last known `scale_factor`. Core 0 re-anchors
itself every ~40s (before 51.5s 32-bit PIO counter wrap) to extend coasting
indefinitely. `LTSP_HOLDOVER_TIMEOUT_US` extended from 30s to 300s (5 minutes).

### Test results (progressive improvement)

| Metric | Test 3 (v0.9) | Test 3b (v1.0) | Test 3c (coast) | Test 3d (5min) |
|--------|--------------|----------------|-----------------|----------------|
| PPS coasting | 50s (crash) | 0s (disabled) | 59s (smooth) | **96s (full)** |
| PPS outage | 423s | 118s | 94s | **0s** |
| Max excursion | -8207 µs | +393 µs | +409 µs | **-212 µs** |
| Recovery to 20 µs | 669s | 199s | 205s | **7s** |
| Post sigma | 4.5 µs | 5.0 µs | 5.9 µs | **5.6 µs** |

Test 3d (90s cable disconnect with 5-minute holdover) achieved zero PPS outage
and 7-second recovery. PPS coasted the entire outage on crystal, drifting smoothly
from +4.8 to -212 µs at -2.1 µs/s. Pipeline reset + PPS reseed on reconnect
gave instant recovery.

---

## What's Next

### Remaining adverse condition tests

From `tools/runs/adverse_test_plan.md`:

1. **Test 6: GM GPS loss** — HIGH PRIORITY, no extra equipment needed
   - Disconnect GPS antenna from GM while system is locked
   - Tests GM holdover flag propagation to RX
   - Tests RX behavior when GM sets LTSP_FLAG_HOLDOVER in PDU
   - Expected: RX transitions to HOLDOVER, PPS coasts on crystal

2. **Test 4: PDV (Packet Delay Variation)** — LOW, needs switch + cross-traffic
3. **Test 5: Burst loss** — LOW, needs switch/bridge
4. **Test 7: Alpha recovery timing** — may already be solved (7s recovery in Test 3d)

### After adverse tests

- Consider GM-side improvements (mentioned in conversation: "before we touch the GM")
- Long-term stability testing (multi-hour/multi-day)
- Temperature sensitivity testing

---

## Key Files Modified

- `ltsp/receiver/main.c` — Gap handler pipeline reset, Core 0 holdover timer,
  holdover re-anchor, removed HOLDOVER PPS disable
- `ltsp/receiver/ltsp_clock.h` — `LTSP_HOLDOVER_TIMEOUT_US` changed to 300000000ULL (5 min)

## Test Run Data

- `tools/runs/20260227_100012/` — Test 3 (unfixed, 90s pull)
- `tools/runs/20260227_112917/` — Test 3b (holdover fixes)
- `tools/runs/20260227_143124/` — Test 3c (holdover coasting, 30s timeout)
- `tools/runs/20260227_155200/` — Test 3d (holdover coasting, 5-min timeout) **← best result**

## Documentation

- `tools/runs/adverse_test_plan.md` — Full test plan with all results documented
- `tools/runs/holdover_fixes.md` — Fix design documentation (written before implementation)
