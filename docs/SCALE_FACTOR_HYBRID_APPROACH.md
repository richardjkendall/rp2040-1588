# Scale Factor Hybrid Approach for 1PPS Scheduling
**Date:** 2025-12-25
**Status:** Proposed (Not Yet Implemented)

## Problem

The 1PPS scheduler needs an accurate scale_factor to compensate for crystal frequency error when calculating ticks to the next second boundary.

Current implementation uses PI integral:
```c
scale_factor = 1.0 - (state.freq_offset_ppb / 1000000000.0);
```

**Limitations:**
- Lags behind temperature changes (PI integral is filtered/smoothed)
- Takes minutes to adapt to environmental changes
- Not responsive during initial convergence

## Proposed Solution: Hybrid Approach

Use **crystal measurement during convergence**, then **PI integral when stable**:

```c
double ptp_discipline_get_scale_factor(void) {
    if (state.discipline_updates < 200) {
        // During convergence: use direct crystal measurement
        // Crystal measurement is real-time, tracks temperature changes
        double crystal_ppm = (double)state.crystal_error_ns / 1000.0; // ns/ms = ppm
        return 1.0 - (crystal_ppm / 1000000.0);
    } else {
        // After convergence: use PI integral (smoother, filtered)
        return 1.0 - (state.freq_offset_ppb / 1000000000.0);
    }
}
```

## How Crystal Error is Measured

Already implemented in `ptp_discipline.c` around line 713:

```c
// Compare actual crystal ticks vs expected ticks based on master time
uint64_t master_delta_ns = ptp_sync_data.t1_master_ns - state.prev_t1_master_ns;
uint64_t expected_ticks = (master_delta_ns * EXPECTED_TICKS_PER_SECOND) / 1000000000ULL;
int64_t crystal_error_ticks = (int64_t)elapsed_ticks - (int64_t)expected_ticks;
state.crystal_error_ns = crystal_error_ticks * 12; // 12ns per tick

// Positive error = crystal runs fast
```

This gives us a **real-time crystal frequency measurement** independent of PI servo!

## Benefits

1. **✅ Fast Initial Convergence**
   - Uses crystal measurement immediately (no waiting for PI integral to settle)
   - 1PPS accurate from first 200 updates (~3 minutes)

2. **✅ Temperature Adaptive**
   - Crystal measurement updates every PTP sync (~1 second)
   - Tracks temperature changes in real-time
   - No hardcoded values

3. **✅ Smooth Steady-State**
   - Transitions to filtered PI integral after convergence
   - Reduces jitter from measurement noise
   - Stable long-term operation

4. **✅ Robust**
   - If PI servo has issues, falls back to crystal measurement
   - Independent verification of frequency error

## Why scale_factor Sign is Negative

**Critical:** `scale_factor = 1.0 - (error / 1e6)` NOT `1.0 + (error / 1e6)`

**Scheduler divides by scale_factor:**
```c
ticks = (ns_until_second / 12.0) / scale_factor
```

**With crystal running fast (+34 ppm):**
- Need **MORE ticks** to count 1 real second
- `scale_factor = 1.0 - 0.000034 = 0.999966`
- `ticks = (1e9 / 12.0) / 0.999966 = 83,336,166` ✓ More ticks!

**If we used addition (WRONG):**
- `scale_factor = 1.0 + 0.000034 = 1.000034`
- `ticks = (1e9 / 12.0) / 1.000034 = 83,330,500` ❌ Fewer ticks!
- 1PPS drifts ~3000 ticks/second → gaps after ~20 seconds

## Implementation Notes

### Current State (2025-12-25)

**File:** `slave/ptp_discipline.c:375`

```c
double ptp_discipline_get_scale_factor(void) {
    return 1.0 - (state.freq_offset_ppb / 1000000000.0);
}
```

### Proposed Change

```c
double ptp_discipline_get_scale_factor(void) {
    // Hybrid: Use crystal measurement early, PI integral later
    if (state.discipline_updates < 200) {
        // Early convergence: Real-time crystal measurement
        // More responsive to temperature, faster convergence
        double crystal_ppm = (double)state.crystal_error_ns / 1000.0;
        return 1.0 - (crystal_ppm / 1000000.0);
    } else {
        // Steady state: Filtered PI integral
        // Smoother, less jitter from measurement noise
        return 1.0 - (state.freq_offset_ppb / 1000000000.0);
    }
}
```

### Tuning Parameters

- **Switch threshold:** 200 updates (~3 minutes)
  - Can adjust based on convergence speed
  - Could use lock stability instead of fixed count

- **Filtering:** Consider EMA for crystal measurement
  ```c
  static double filtered_crystal_ppm = 0.0;
  double alpha = 0.1; // 10 second time constant
  filtered_crystal_ppm = filtered_crystal_ppm * (1.0 - alpha) + crystal_ppm * alpha;
  ```

## Testing Plan

1. **Verify crystal measurement accuracy**
   - Compare `state.crystal_error_ns` vs `state.freq_offset_ppb`
   - Should converge to similar values

2. **Test 1PPS continuity**
   - Monitor for gaps in measurement data
   - Should have continuous 1PPS from boot onwards

3. **Test temperature response**
   - Artificially heat/cool device
   - Verify 1PPS stays locked during temperature changes

4. **Compare convergence speed**
   - Current: Time to first stable 1PPS
   - Hybrid: Should be faster (crystal measurement available immediately)

## Open Questions

1. **Crystal measurement noise:**
   - How much jitter in `crystal_error_ns`?
   - Need filtering for 1PPS scheduler?

2. **Optimal switch threshold:**
   - 200 updates too early/late?
   - Use lock stability instead?

3. **Fallback strategy:**
   - If PI integral diverges, revert to crystal measurement?
   - Add sanity checks on scale_factor range?

## References

- **1PPS Scheduler:** `slave/pps_scheduler.c:48-95`
- **Crystal Measurement:** `slave/ptp_discipline.c:713`
- **Current scale_factor:** `slave/ptp_discipline.c:375`
- **PI Servo:** `slave/ptp_discipline.c:452-484`

---

**Status:** Documented for future implementation
**Next Steps:**
1. Investigate telemetry gaps (possible connection issues)
2. If telemetry stable, implement hybrid approach
3. Test and compare performance
